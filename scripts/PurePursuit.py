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
from std_msgs.msg import ColorRGBA, Float32 ,Bool

import matplotlib.pyplot as plt

from enum import Enum

#######################################
# Simple Path follower (Pure Pursuit) #
#######################################

class mode(Enum):
    ROTATION = 0 #With a rotating
    POSFIXED = 1 #Posture fixed

class Simple_path_follower():

    ##################
    # Initialization #
    ##################
    def __init__(self):

        rospy.init_node('Simple_Path_Follower', anonymous=True)
        self.r = rospy.Rate(50)  # 50hz

        self.target_speed_max = 20.0            #target speed [km/h]
        self.target_speed_min = 10.0
        self.target_LookahedDist = 0.5      #Lookahed distance for Pure Pursuit[m]

        self.GOAL_LIMIT = 0.02
        self.MODE = mode.POSFIXED

        #first flg (for subscribe global path topic)
        self.path_first_flg = False
        self.odom_first_flg = False
        self.position_search_flg = False
        self.last_indx = 0

        #initialize publisher
        self.cmdvel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=50)
        self.lookahed_pub = rospy.Publisher("/lookahed_marker", Marker, queue_size=50)
        self.value_pub1 = rospy.Publisher("CMD_Vx", Float32, queue_size=1)
        self.value_pub2 = rospy.Publisher("CMD_Vy", Float32, queue_size=1)
        self.value_pub3 = rospy.Publisher("CMD_Az", Float32, queue_size=1)
        self.value_pub4 = rospy.Publisher("Curvature_val", Float32, queue_size=1)
        self.end_pub = rospy.Publisher("move_end", Bool, queue_size=1)

        #initialize subscriber
        self.path_sub = rospy.Subscriber("/path", Path, self.cb_get_path_topic_subscriber)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        #走行経路のパスを配信
        self.path = Path()
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
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
        self.maxCV=0
        self.minCV=0
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

    def mode1(self):
        speed=0
        yaw_rate = 0.0
        if self.path_first_flg == True and self.odom_first_flg == True:

            #Target point calculation
            dist_from_current_pos_np = np.sqrt(np.power((self.path_x_np-self.current_x),2) + np.power((self.path_y_np-self.current_y),2))
            for i in range(len(self.tld)):
                print (i,dist_from_current_pos_np[i])
                dist_sp_from_nearest=self.tld[i]
                self.nowCV=self.curvature_val[i]
                speed=math.fabs(self.target_LookahedDist-self.map(self.nowCV,self.minCV,self.maxCV,self.target_speed_min,self.target_speed_max))
                if (dist_from_current_pos_np[i]) > self.tld[i]:
                    self.target_lookahed_x = self.path_x_np[i]
                    self.target_lookahed_y = self.path_y_np[i]
                    self.path_x_np=self.path_x_np[i:len(self.path_x_np)]
                    self.path_y_np=self.path_y_np[i:len(self.path_y_np)]
                    self.tld=self.tld[i:len(self.tld)]
                    self.curvature_val=self.curvature_val[i:len(self.curvature_val)]
                    self.cflag=True
                    break
                if self.cflag==False and np.amax(dist_sp_from_nearest)<=self.GOAL_LIMIT:#check goal
                    self.gflag=True
            target_lookahed_x=self.target_lookahed_x
            target_lookahed_y=self.target_lookahed_y
            #End processing
            if self.gflag:
                cmd_vel = Twist()
                self.cmdvel_pub.publish(cmd_vel)
                self.path_first_flg = False
                rospy.loginfo("goal!!")
                return
            #calculate target yaw rate
            if self.cflag:
                self.target_yaw = math.atan2(target_lookahed_y-self.current_y,target_lookahed_x-self.current_x)
                self.oldspeed=speed
                self.cflag=False
            else:
                self.dist=math.sqrt((self.target_lookahed_x-self.current_x)**2+(self.target_lookahed_y-self.current_y)**2)
                speed=self.map(self.dist,0,self.target_LookahedDist,self.target_speed_min,self.oldspeed)
                if self.dist <= self.GOAL_LIMIT:
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
            if self.first and math.fabs(yaw_diff)<(math.pi/4):
                self.first=False
            elif not self.first:
                if speed>(self.target_speed_max):
                    speed=self.target_speed_max
                elif speed<(self.target_speed_min):
                    speed=self.target_speed_min
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

            rospy.loginfo(str(self.first)+","+str(yaw_diff*180/math.pi)+","+str(target_yaw*180/math.pi)+","+str(self.current_yaw_euler*180/math.pi)+",tld size:"+str(len(self.tld))+",dist:"+str(self.dist))
            #publish maker
            self.publish_lookahed_marker(target_lookahed_x,target_lookahed_y,target_yaw)
        #debug
        self.value_pub1.publish(speed)
        self.value_pub2.publish(0.0)
        self.value_pub3.publish(yaw_rate)
        self.value_pub4.publish(self.nowCV)

    def mode2(self):
        speed=0
        Vx=0.0
        Vy=0.0
        yaw_rate = 0.0
        if self.path_first_flg == True and self.odom_first_flg == True:

            #Target point calculation
            dist_from_current_pos_np = np.sqrt(np.power((self.path_x_np-self.current_x),2) + np.power((self.path_y_np-self.current_y),2))
            for i in range(len(self.tld)):
                print (i,dist_from_current_pos_np[i])
                dist_sp_from_nearest=self.tld[i]
                self.nowCV=self.curvature_val[i]
                speed=math.fabs(self.target_LookahedDist-self.map(self.nowCV,self.minCV,self.maxCV,self.target_speed_min,self.target_speed_max))
                if (dist_from_current_pos_np[i]) > self.tld[i]:
                    self.target_lookahed_x = self.path_x_np[i]
                    self.target_lookahed_y = self.path_y_np[i]
                    self.path_x_np=self.path_x_np[i:len(self.path_x_np)]
                    self.path_y_np=self.path_y_np[i:len(self.path_y_np)]
                    self.tld=self.tld[i:len(self.tld)]
                    self.curvature_val=self.curvature_val[i:len(self.curvature_val)]
                    self.cflag=True
                    break
                if self.cflag==False and np.amax(dist_sp_from_nearest)<=self.GOAL_LIMIT:#check goal
                    self.gflag=True
            target_lookahed_x=self.target_lookahed_x
            target_lookahed_y=self.target_lookahed_y
            #calculate target yaw rate
            self.target_yaw = math.atan2(target_lookahed_y-self.current_y,target_lookahed_x-self.current_x)
            if self.cflag:
                self.oldspeed=speed
                self.cflag=False
            else:
                self.dist=math.sqrt((self.target_lookahed_x-self.current_x)**2+(self.target_lookahed_y-self.current_y)**2)
                speed=self.map(self.dist,0,self.target_LookahedDist,self.target_speed_min,self.oldspeed)
                if self.dist <= self.GOAL_LIMIT:
                    self.gflag=True
            target_yaw=self.target_yaw
            #End processing
            if self.gflag:
                cmd_vel = Twist()
                self.cmdvel_pub.publish(cmd_vel)
                self.path_first_flg = False
                rospy.loginfo("dist:"+str(self.dist)+"[m]")
                rospy.loginfo("goal!!")
                return

            #Set Cmdvel
            if self.first:
                self.first=False
            elif not self.first:
                if speed>(self.target_speed_max):
                    speed=self.target_speed_max
                elif speed<(self.target_speed_min):
                    speed=self.target_speed_min
            else:
                speed=0

            Vx=speed*math.cos(target_yaw)
            Vy=speed*math.sin(target_yaw)

            cmd_vel = Twist()
            cmd_vel.linear.x = Vx    #[m/s]
            cmd_vel.linear.y = Vy
            cmd_vel.linear.z = 0.0
            cmd_vel.angular.x = 0.0
            cmd_vel.angular.y = 0.0
            cmd_vel.angular.z = 0.0
            self.cmdvel_pub.publish(cmd_vel)

            rospy.loginfo("Speed:"+str(speed)+"[m/s],Yaw:"+str(target_yaw)+"[rad],tld size:"+str(len(self.tld))+",dist:"+str(self.dist)+"[m]")
            #publish maker
            self.publish_lookahed_marker(target_lookahed_x,target_lookahed_y,target_yaw)
        #debug
        self.value_pub1.publish(Vx)
        self.value_pub2.publish(Vy)
        self.value_pub3.publish(0.0)
        self.value_pub4.publish(self.nowCV)

    ###################
    # Update cmd_vel  #
    ###################
    def update_cmd_vel(self):
        self.cb_get_odometry()
        if self.MODE == mode.ROTATION:
            self.mode1()
        else:
            self.mode2()
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
            self.tld=[]
            for i in self.curvature_val:
                self.tld.append(math.fabs(self.target_LookahedDist-self.map(i,0,np.amax(self.curvature_val),0,self.target_LookahedDist-0.2)))
            #plt.plot(self.curvature_val)
            #plt.plot(self.tld)
            #plt.show()
            self.cur_diff=np.amax(self.curvature_val)-np.amin(self.curvature_val)#曲率最大最小の差
            self.maxCV=np.amax(self.curvature_val)
            self.minCV=np.amin(self.curvature_val)
            self.first = self.path_first_flg = True
            self.end_pub.publish(True)
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
