#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import rospy
import numpy as np
import matplotlib.pyplot as plt
from PathCreator import path_creator
#Import message type, OccupancyGrid is the message type
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
import tf2_ros

odom=[0,0]
rgoal=[0,0]
readgoal=False
def goal_cb(data):#ゴール受信関数
	global readgoal
	readgoal=True
	rgoal[0]=data.pose.position.x
	rgoal[1]=data.pose.position.y

#ROS初期設定
rospy.init_node('PathGenerator')
path_pub = rospy.Publisher('/path', Path, queue_size=10)
goal_sub = rospy.Subscriber('/move_base_simple/goal',PoseStamped, goal_cb)
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

#Map
class Map(object):
	def __init__(self,f):
	#rospySubscribe to the map topic, the second is the data type, the third is the callback function
	# Pass the subscribed data to the callback function, which is the mapmsg variable
	#If a topic comes, call the callback function directly
		self.map_sub = rospy.Subscriber("map",OccupancyGrid, self.callback)
		print "get map~"
		#The output below is the address, not the data
		print self.map_sub
		self.f=f

	#The definition of the callback function, passed mapmsg
	def callback(self,mapmsg):
		#obstacleList = np.empty(3)
		try:
			print "into callback"
			#Mainly want to get data, here is stored map information
			map = mapmsg.data
			# Below is the tuple type
			#Change to numpy format that can draw pictures
			map = np.array(map)
			#The following output is (368466,), obviously can not draw
			#Need to reshape, factor the above numbers online, then calculate the two largest factors
			#So it's probably like this:
			map = map.reshape((mapmsg.info.height,mapmsg.info.width))
			#You can see that most of the values ​​are -1, so you need to regularize the values
			row,col = map.shape
			tem = np.zeros((row,col))
			for i in range(row):
				for j in range(col):
					if(map[i,j]==-1):
						tem[j,i]=255
					else:
						tem[j,i]=map[i,j]

			print "resol",mapmsg.info.resolution,"h",mapmsg.info.height,"w",mapmsg.info.width
			print "orizin x:",mapmsg.info.origin.position.x,"y:",mapmsg.info.origin.position.y
			ox=mapmsg.info.origin.position.x
			oy=mapmsg.info.origin.position.y
			index_ox=int(-1*ox/mapmsg.info.resolution)
			index_oy=int(-1*oy/mapmsg.info.resolution)
		except Exception,e:
			print e
			rospy.loginfo('convert rgb image error')
		tem[int(-1*ox/mapmsg.info.resolution),int(-1*oy/mapmsg.info.resolution)]=-255
		print("Field:",self.f)
		Pcreat=path_creator(path_pub,index_ox,index_oy,mapmsg.info.resolution,[int(-1*ox/mapmsg.info.resolution),int(-1*oy/mapmsg.info.resolution)],self.f)
		global readgoal
		rospy.loginfo('Map conversion completed')
		Pcreat.create(tem)
		return

	def getImage():
		return self.rgb_image


def main(_):
	args = sys.argv
	print(args[1])
	v=Map(str(args[1]))
	rospy.spin()

if __name__=='__main__':
	main('_')