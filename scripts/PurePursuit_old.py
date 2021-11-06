#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math
# import for ros function
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from nav_msgs.msg import Path, Odometry

from jsk_rviz_plugins.msg import *
from std_msgs.msg import ColorRGBA, Float32

import matplotlib.pyplot as plt

#######################################
# Simple Path follower (Pure Pursuit) #
#######################################
class Simple_path_follower():

    ##################
    # Initialization #
    ##################
    def __init__(self):

        rospy.init_node('Simple_Path_Follower', anonymous=True)
        self.r = rospy.Rate(50)  # 50hz

        self.target_speed_max = 1.0            #target speed [km/h]
        self.target_speed_min = 0.01
        self.target_LookahedDist = 0.5      #Lookahed distance for Pure Pursuit[m]

        #first flg (for subscribe global path topic)
        self.path_first_flg = False
        self.odom_first_flg = False
        self.position_search_flg = False
        self.last_indx = 0

        #initialize publisher
        self.cmdvel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=50)
        self.lookahed_pub = rospy.Publisher("/lookahed_marker", Marker, queue_size=50)
        self.value_pub1 = rospy.Publisher("CMD_Vx", Float32, queue_size=1)
        self.value_pub2 = rospy.Publisher("CMD_Az", Float32, queue_size=1)
        self.value_pub3 = rospy.Publisher("Curvature_val", Float32, queue_size=1)

        #initialize subscriber
        self.path_sub = rospy.Subscriber("/path", Path, self.cb_get_path_topic_subscriber)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        #走行経路のパスを配信
        self.path = Path()
        #self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        #self.odom_sub = rospy.Subscriber('/F11Robo/diff_drive_controller/odom', Odometry, self.odom_cb)
        self.odom_sub = rospy.Subscriber('/fusion/odom', Odometry, self.odom_cb)
        self.path_pub = rospy.Publisher('/path_hist', Path, queue_size=10)

        self.cflag=False
        self.target_yaw=0
        self.target_lookahed_x=0
        self.target_lookahed_y=0

        self.oldspeed=0
        self.dist=0
        self.gflag=False
        self.cur_diff=0.0
        self.nowCV=0
        self.tld=[]

    def map(self,x,in_min,in_max,out_min,out_max):
        value=(x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        if value>out_max:
            value=out_max
        if value<out_min:
            value=out_min
        return value

    def publish_lookahed_marker(self,x,y,yaw_euler):

        marker_data = Marker()
        marker_data.header.frame_id = "map"
        marker_data.header.stamp = rospy.Time.now()

        marker_data.ns = "my_name_space"
        marker_data.id = 0

        marker_data.action = Marker.ADD

        marker_data.pose.position.x = x
        marker_data.pose.position.y = y
        marker_data.pose.position.z = 0.0

        temp_quaternion = tf.transformations.quaternion_from_euler(0,0,yaw_euler)

        marker_data.pose.orientation.x = temp_quaternion[0]
        marker_data.pose.orientation.y = temp_quaternion[1]
        marker_data.pose.orientation.z = temp_quaternion[2]
        marker_data.pose.orientation.w = temp_quaternion[3]

        marker_data.color.r = 1.0
        marker_data.color.g = 0.0
        marker_data.color.b = 0.0
        marker_data.color.a = 1.0

        marker_data.scale.x = 0.1
        marker_data.scale.y = 0.1
        marker_data.scale.z = 0.1

        marker_data.lifetime = rospy.Duration()
        marker_data.type = 0

        self.lookahed_pub.publish(marker_data)

 
    ###################
    # Update cmd_vel  #
    ###################
    def update_cmd_vel(self):
        self.cb_get_odometry()
        speed=0
        yaw_rate = 0.0
        if self.path_first_flg == True and self.odom_first_flg == True:

            dist_from_current_pos_np = np.sqrt(np.power((self.path_x_np-self.current_x),2) + np.power((self.path_y_np-self.current_y),2))
            min_indx = dist_from_current_pos_np.argmin()
            nearest_x = self.path_x_np[min_indx]
            nearest_y = self.path_y_np[min_indx]
            # Get nearest Path point at first time123
            if self.position_search_flg == False:
                self.pass_flg_np[0:min_indx] = 1    #Set pass flg
                self.position_search_flg = True
            else:
                # Check pass flg from vehicle position
                for indx in range (self.last_indx,self.path_x_np.shape[0]):
                    if dist_from_current_pos_np[indx] < 0.1:
                        self.pass_flg_np[indx] = 1
                    else:
                        break
            self.last_indx = min_indx

            #check goal
            if self.pass_flg_np[self.path_x_np.shape[0]-1] == 1 or self.gflag:
                cmd_vel = Twist()
                self.cmdvel_pub.publish(cmd_vel)
                self.path_first_flg = False
                rospy.loginfo("goal!!")
                return
            #calculate target point
            dist_sp_from_nearest = 0.0
            dist_sp_from_nearest_N=0.0
            target_lookahed_x = nearest_x
            target_lookahed_y = nearest_y
            for indx in range (self.last_indx,self.path_x_np.shape[0]):
                dist_sp_from_nearest = self.path_st_np[indx] - self.path_st_np[self.last_indx]
                #tld=math.fabs(self.target_LookahedDist-self.map(self.nowCV,0,np.amax(self.curvature_val),0,self.target_LookahedDist-0.01))
                speed=math.fabs(self.target_LookahedDist-self.map(self.nowCV,0,np.amax(self.curvature_val),self.target_speed_min/3.6,self.target_speed_max/3.6))
                self.nowCV=self.curvature_val[indx]#debug
                #print(indx,tld,dist_sp_from_nearest_N,dist_sp_from_nearest)
                if self.cur_diff>=0.7:
                    speed=math.fabs(self.target_LookahedDist-self.map(self.curvature_val[indx],0,np.amax(self.curvature_val),self.target_speed_min/3.6,self.target_speed_max/3.6))
                else:
                    speed=self.target_speed_max/3.6
                #if tld>=self.target_LookahedDist:
                #    tld=self.target_LookahedDist
                if (dist_sp_from_nearest) >= self.tld[indx]:
                    print "indx",indx,self.target_LookahedDist,"tld:",self.tld[indx],"curv:",self.curvature_val[indx],dist_sp_from_nearest_N,dist_sp_from_nearest
                    self.target_lookahed_x = self.path_x_np[indx]
                    self.target_lookahed_y = self.path_y_np[indx]
                    self.cflag=True
                    break
            target_lookahed_x=self.target_lookahed_x
            target_lookahed_y=self.target_lookahed_y
            #calculate target yaw rate
            if self.cflag:
                self.target_yaw = math.atan2(target_lookahed_y-self.current_y,target_lookahed_x-self.current_x)
                self.oldspeed=speed
                self.cflag=False
            else:
                self.dist=math.sqrt((self.target_lookahed_x-self.current_x)**2+(self.target_lookahed_y-self.current_y)**2)
                speed=self.map(self.dist,0,self.target_LookahedDist,self.target_speed_min/3.6,self.oldspeed)
                if self.dist < 0.1:
                    self.gflag=True
            target_yaw=self.target_yaw

            yaw_diff = target_yaw - self.current_yaw_euler

            if yaw_diff > math.pi:
                yaw_diff = -2*math.pi+yaw_diff
            elif yaw_diff < -math.pi:
                yaw_diff = 2*math.pi+yaw_diff



            sample_sec = dist_sp_from_nearest/(speed)
            if sample_sec != 0.0:
                yaw_rate = math.fabs(yaw_diff)/sample_sec
            else:
                yaw_rate = 0.0

            # check vehicle orientation and target yaw
            if math.fabs(target_yaw - self.current_yaw_euler) < math.pi:
                if (target_yaw) < (self.current_yaw_euler):
                    yaw_rate = yaw_rate * (-1.0)
            elif math.fabs(target_yaw - self.current_yaw_euler) > math.pi:
                if (target_yaw) > (self.current_yaw_euler):
                    yaw_rate = yaw_rate * (-1.0)

            #Set Cmdvel
            if self.first and math.fabs(yaw_diff)<(math.pi/60):
                self.first=False
            elif not self.first:
                if speed>(self.target_speed_max/3.6):
                    speed=self.target_speed_max/3.6
                elif speed<(self.target_speed_min/3.6):
                    speed=self.target_speed_min/3.6
            else:
                speed=0

            cmd_vel = Twist()
            cmd_vel.linear.x = speed    #[m/s]
            cmd_vel.linear.y = 0.0
            cmd_vel.linear.z = 0.0
            cmd_vel.angular.x = 0.0
            cmd_vel.angular.y = 0.0
            cmd_vel.angular.z = yaw_rate
            self.cmdvel_pub.publish(cmd_vel)

            #rospy.loginfo(str(self.first)+","+str(yaw_diff*180/math.pi)+","+str(target_yaw*180/math.pi)+","+str(self.current_yaw_euler*180/math.pi)+",yaw_rate:"+str(yaw_rate)+",Vx:"+str(speed)+","+"dist:"+str(self.dist))

            #publish maker
            self.publish_lookahed_marker(target_lookahed_x,target_lookahed_y,target_yaw)
            #print("cmd_vel_update")
        #debug
        self.value_pub1.publish(speed)
        self.value_pub2.publish(yaw_rate)
        self.value_pub3.publish(self.nowCV)
        self.r.sleep()



    ####################################
    # Callback for receiving Odometry  #
    ####################################

    def odom_cb(self,data):
        if self.path_first_flg:
            self.path.header = data.header
            pose = PoseStamped()
            pose.header = data.header
            pose.pose = data.pose.pose
            self.path.poses.append(pose)
            self.path_pub.publish(self.path)
        else:
            self.path = Path()

    def cb_get_odometry(self):
        try:
            t = self.tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
            self.current_x=t.transform.translation.x
            self.current_y=t.transform.translation.y
            e = tf.transformations.euler_from_quaternion((t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w))
            yaw_euler = e[2]
            self.current_yaw_euler = yaw_euler
            if not self.odom_first_flg:
                rospy.loginfo("get odom")
            self.odom_first_flg = True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as er:
            print(er)

    ######################################
    # Callback for receiving path topic  #
    ######################################
    def cb_get_path_topic_subscriber(self,msg):
        if self.path_first_flg != True:
            self.path_x_np = np.zeros([len(msg.poses)])
            self.path_y_np = np.zeros([len(msg.poses)])
            self.path_st_np = np.zeros([len(msg.poses)])
            self.pass_flg_np = np.zeros([len(msg.poses)])
            last_x = 0.0
            last_y = 0.0
            last_st = 0.0
            for indx in range(len(msg.poses)):
                self.path_x_np[indx] = msg.poses[indx].pose.position.x
                self.path_y_np[indx] = msg.poses[indx].pose.position.y
                self.path_st_np[indx] = last_st + math.sqrt((self.path_x_np[indx]-last_x)**2 + (self.path_y_np[indx]-last_y)**2)
                last_x = self.path_x_np[indx]
                last_y = self.path_y_np[indx]
                last_st = self.path_st_np[indx]
            #曲率計算
            x_t = np.gradient(self.path_x_np)
            y_t = np.gradient(self.path_y_np)
            xx_t = np.gradient(x_t)
            yy_t = np.gradient(y_t)
            self.curvature_val = np.abs(xx_t * y_t - x_t * yy_t) / (x_t * x_t + y_t * y_t)**1.5
            print self.curvature_val
            #plt.plot(self.curvature_val)
            #plt.show()
            self.tld=[]
            for i in self.curvature_val:
                self.tld.append(math.fabs(self.target_LookahedDist-self.map(i,0,np.amax(self.curvature_val),0,self.target_LookahedDist-0.01)))
            #plt.plot(self.path_st_np)
            plt.plot(self.curvature_val)
            plt.plot(self.tld)
            plt.show()
            self.cur_diff=np.amax(self.curvature_val)-np.amin(self.curvature_val)#曲率最大最小の差
            self.first = self.path_first_flg = True
            self.gflag=False
            rospy.loginfo("get path")

if __name__ == '__main__':
    test = Simple_path_follower()
    rospy.loginfo('Path following is Started...')
    try:
        while not rospy.is_shutdown():
            test.update_cmd_vel()
    except KeyboardInterrupt:
        print("finished!")
