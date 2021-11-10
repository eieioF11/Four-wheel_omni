#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import math
# import for ros function
import rospy
import tf
import tf2_ros
from geometry_msgs.msg import PoseStamped, Twist

from jsk_rviz_plugins.msg import *
from std_msgs.msg import ColorRGBA, Float32, Float64

class omuni4():
    def __init__(self):
        rospy.init_node('Simple_Path_Follower', anonymous=True)
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

        #initialize subscriber
        self.path_sub = rospy.Subscriber("/cmd_vel",Twist, self.get_cmd)

        self.move(0,0,0)

    def move(self,Vx,Vy,Angular):
        w0=Vy+Angular
        w1=-Vx+Angular
        w2=-Vy+Angular
        w3=Vx+Angular
        rospy.loginfo("w(%f,%f,%f,%f)",w0,w1,w2,w3)
        self.c0.publish(w0)
        self.c1.publish(w1)
        self.c2.publish(w2)
        self.c3.publish(w3)

    def move_90(self,Vx,Vy,Angular):
        w0=(-Vx+Vy)/2.0+Angular
        w1=( Vx+Vy)/2.0+Angular
        w2=( Vx-Vy)/2.0+Angular
        w3=(-Vx-Vy)/2.0+Angular
        rospy.loginfo("w(%f,%f,%f,%f)",w0,w1,w2,w3)
        self.c0.publish(w0)
        self.c1.publish(w1)
        self.c2.publish(w2)
        self.c3.publish(w3)

    def get_cmd(self,value):
        rospy.loginfo("LinerX:%f",value.linear.x)
        rospy.loginfo("LinerY:%f",value.linear.y)
        rospy.loginfo("Angular:%f",value.angular.z)
        self.move_90(value.linear.x,value.linear.y,value.angular.z)

    def update(self):
        pass



if __name__ == '__main__':
    rospy.loginfo('Omuni4 controller start!')
    wheel=omuni4()
    try:
        while not rospy.is_shutdown():
            wheel.update()
    except KeyboardInterrupt:
        print("finished!")
