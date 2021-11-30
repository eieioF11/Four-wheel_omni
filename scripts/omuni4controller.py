#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from math import sin, cos, pi,sqrt
# import for ros function
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from control_msgs.msg import JointControllerState

from jsk_rviz_plugins.msg import *
from std_msgs.msg import ColorRGBA, Float32, Float64
from nav_msgs.msg import Odometry

class omuni4():
    def __init__(self):
        # Parameter
        self.R=0.03
        self.L=0.1*sqrt(2) #0.1414...
        self.Adjust=50.0 #1.0
        # Initialization
        rospy.init_node('Simple_Path_Follower', anonymous=True)
        rospy.loginfo("R:%f,L:%f",self.R,self.L)
        self.r = rospy.Rate(50)  # 50hz
        #initialize publisher
        #self.cmdvel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=50)#実機使用時
        self.value_pub1 = rospy.Publisher("CMD_Vx", Float32, queue_size=1)
        self.value_pub2 = rospy.Publisher("CMD_Az", Float32, queue_size=1)
        self.value_pub3 = rospy.Publisher("Curvature_val", Float32, queue_size=1)

        self.c0 = rospy.Publisher("/OmuniRobot/joint_controller0/command", Float64, queue_size=5)
        self.c1 = rospy.Publisher("/OmuniRobot/joint_controller1/command", Float64, queue_size=5)
        self.c2 = rospy.Publisher("/OmuniRobot/joint_controller2/command", Float64, queue_size=5)
        self.c3 = rospy.Publisher("/OmuniRobot/joint_controller3/command", Float64, queue_size=5)

        self.odom = rospy.Publisher("/odom", Odometry, queue_size=5)
        self.odom_wheel = rospy.Publisher("/OmuniRobot/odom", Odometry, queue_size=5)
        #initialize subscriber
        self.cmd_sub = rospy.Subscriber("/cmd_vel",Twist, self.get_cmd)
        #self.state0_sub = rospy.Subscriber("/OmuniRobot/joint_controller0/state",JointControllerState, self.get_state0)
        #self.state1_sub = rospy.Subscriber("/OmuniRobot/joint_controller1/state",JointControllerState, self.get_state1)
        #self.state2_sub = rospy.Subscriber("/OmuniRobot/joint_controller2/state",JointControllerState, self.get_state2)
        #self.state3_sub = rospy.Subscriber("/OmuniRobot/joint_controller3/state",JointControllerState, self.get_state3)
        self.gazebo_states_sub = rospy.Subscriber("/tracker",Odometry, self.get_gazebo_states)

        self.w0=0.0
        self.w1=0.0
        self.w2=0.0
        self.w3=0.0
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vth = 0.0
        self.last_time=rospy.Time.now()
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.move(0,0,0)

    def move(self,Vx,Vy,Angular):
        w0=Vy+Angular
        w1=-Vx+Angular
        w2=-Vy+Angular
        w3=Vx+Angular
        #rospy.loginfo("w(%f,%f,%f,%f)",w0,w1,w2,w3)
        self.c0.publish(w0)
        self.c1.publish(w1)
        self.c2.publish(w2)
        self.c3.publish(w3)

    def move_90(self,Vx,Vy,Angular):
        Angular=Angular/10.0
        w0=(( Vx+Vy)/2.0-Angular)*self.Adjust
        w1=((-Vx+Vy)/2.0-Angular)*self.Adjust
        w2=((-Vx-Vy)/2.0-Angular)*self.Adjust
        w3=(( Vx-Vy)/2.0-Angular)*self.Adjust
        #rospy.loginfo("w(%f,%f,%f,%f)",w0,w1,w2,w3)
        self.c0.publish(w0)
        self.c1.publish(w1)
        self.c2.publish(w2)
        self.c3.publish(w3)

    def get_cmd(self,value):
        #rospy.loginfo("LinerX:%f",value.linear.x)
        #rospy.loginfo("LinerY:%f",value.linear.y)
        #rospy.loginfo("Angular:%f",value.angular.z)
        self.move_90(value.linear.x,value.linear.y,value.angular.z)

    def get_state0(self,value):
        #rospy.loginfo("state0:%f",value.process_value)
        self.w0=value.process_value
    def get_state1(self,value):
        #rospy.loginfo("state1:%f",value.process_value)
        self.w1=value.process_value
    def get_state2(self,value):
        #rospy.loginfo("state2:%f",value.process_value)
        self.w2=value.process_value
    def get_state3(self,value):
        #rospy.loginfo("state3:%f",value.process_value)
        self.w3=value.process_value

    def get_gazebo_states(self,value):
        current_time = rospy.Time.now()
        #rospy.loginfo("state3:%f",value.process_value)
        self.tracker=value
        #print(value.pose.pose.position.x,value.pose.pose.position.y)
        # first, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (value.pose.pose.position.x,value.pose.pose.position.y, 0.),
            (value.pose.pose.orientation.x, value.pose.pose.orientation.y, value.pose.pose.orientation.z, value.pose.pose.orientation.w),
            current_time,
            "base_link",
            "odom"
        )

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        # set the position
        odom.pose.pose = value.pose.pose

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = value.twist.twist

        # publish the message
        self.odom.publish(odom)

    def calculate_odom(self):
        current_time = rospy.Time.now()
        self.vx=self.R*((self.w0-self.w2)+(self.w3-self.w1))*2.0
        self.vy=self.R*((self.w0-self.w2)-(self.w3-self.w1))*2.0
        self.vth=(self.R/(4.0*self.L))*(self.w0+self.w1+self.w2+self.w3)
        # compute odometry in a typical way given the velocities of the robot
        dt = (current_time - self.last_time).to_sec()
        delta_x = (self.vx * cos(self.th) - self.vy * sin(self.th)) * dt
        delta_y = (self.vx * sin(self.th) + self.vy * cos(self.th)) * dt
        delta_th = self.vth * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        ## first, we'll publish the transform over tf
        #self.odom_broadcaster.sendTransform(
        #    (self.x, self.y, 0.),
        #    odom_quat,
        #    current_time,
        #    "base_link",
        #    "odom"
        #)

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        # set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(self.vx, self.vy, 0), Vector3(0, 0, self.vth))

        # publish the message
        self.odom_wheel.publish(odom)

        self.last_time = current_time
        self.r.sleep()

    def update(self):
        self.calculate_odom()



if __name__ == '__main__':
    rospy.loginfo('Omuni4 controller start!')
    wheel=omuni4()
    try:
        while not rospy.is_shutdown():
            wheel.update()
    except KeyboardInterrupt:
        print("finished!")
