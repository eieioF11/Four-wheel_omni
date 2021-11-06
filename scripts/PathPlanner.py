#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy

import numpy as np

#Import message type, OccupancyGrid is the message type
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
import tf2_ros
import matplotlib.pyplot as plt

odom=[0,0]
rgoal=[0,0]
readgoal=False
def goal_cb(data):#ゴール受信関数
	global readgoal
	readgoal=True
	rgoal[0]=data.pose.position.x
	rgoal[1]=data.pose.position.y

#ROS初期設定
rospy.init_node('PathPlanner')
path_pub = rospy.Publisher('/path', Path, queue_size=10)
goal_sub = rospy.Subscriber('/move_base_simple/goal',PoseStamped, goal_cb)
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

#path generation
def path_generation(sPath,ox,oy,resolution):#ROSにpath形式のデータを配信
	#Initialize odometry header
	global path_pub
	global head
	path = Path()
	path_header = Header()
	path_header.seq = 0
	path_header.stamp = rospy.Time.now()
	path_header.frame_id = "map"

	for i in range(0,len(sPath)):
		temp_pose = PoseStamped()
		temp_pose.pose.position.x = (sPath[i][0]-ox)*resolution
		temp_pose.pose.position.y = (sPath[i][1]-oy)*resolution
		temp_pose.pose.position.z = 0
		temp_pose.header = path_header
		temp_pose.header.seq = i
		path.poses.append(temp_pose)
	#print path.poses
	path.header = path_header
	path_pub.publish(path)
	rospy.loginfo('End path generation')


#path planner
import a_star
from scipy.special import comb

def bernstein_poly(i, n, t):
    """
     The Bernstein polynomial of n, i as a function of t
    """

    return comb(n, i) * ( t**(n-i) ) * (1 - t)**i


def bezier_curve(points, nTimes=1000):
    """
       Given a set of control points, return the
       bezier curve defined by the control points.

       points should be a list of lists, or list of tuples
       such as [ [1,1], 
                 [2,3], 
                 [4,5], ..[Xn, Yn] ]
        nTimes is the number of time steps, defaults to 1000

        See http://processingjs.nihongoresources.com/bezierinfo/
    """

    nPoints = len(points)
    xPoints = np.array([p[0] for p in points])
    yPoints = np.array([p[1] for p in points])

    t = np.linspace(0.0, 1.0, nTimes)

    polynomial_array = np.array([ bernstein_poly(i, nPoints-1, t) for i in range(0, nPoints)   ])

    xvals = np.dot(xPoints, polynomial_array)
    yvals = np.dot(yPoints, polynomial_array)

    return xvals, yvals

def a_star_pathplanner(start,goal,grid):
	test_planner = a_star.PathPlanner(grid,False)
	init,path=test_planner.a_star(start,goal)
	path=np.vstack((init,path))#初期位置をpathに追加
	xvals, yvals = bezier_curve(path, nTimes=1000)#ベジェ曲線で経路を滑らかにする
	cpath=np.flipud(np.array(list(map(list, zip(xvals,yvals)))))#xvalsとyvalsの結合と反転
	rospy.loginfo('path calculation completed')
	#結果表示
	print path
	print cpath
	plt.imshow(grid)
	plt.plot(path[:,1],path[:,0])
	plt.plot(cpath[:,1],cpath[:,0],color = "red")
	plt.show()
	return cpath

#Map
class Map(object):
	def __init__(self):
	#rospySubscribe to the map topic, the second is the data type, the third is the callback function
	# Pass the subscribed data to the callback function, which is the mapmsg variable
	#If a topic comes, call the callback function directly
		self.map_sub = rospy.Subscriber("map",OccupancyGrid, self.callback)
		print "get map~"
		#The output below is the address, not the data
		print self.map_sub

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
			grid = np.zeros((row,col))
			for i in range(row):
				for j in range(col):
					if(map[i,j]==-1):
						tem[j,i]=255
					else:
						tem[j,i]=map[i,j]
			#cost map作成
			n=10
			rn=3+2*(n-1)
			print rn
			for i in range(col):
				for j in range(row):
					if tem[i,j]==100:
						for k in range(rn):
							for l in range(rn):
								val=tem[i-n+k,j-n+l]
								if val==0:
									tem[i-n+k,j-n+l]=200
			#経路計算用のマップ作成
			for i in range(col):
				for j in range(row):
					grid[i,j]=0
					if tem[i,j]>0 and tem[i,j]!=255:
						grid[i,j]=1

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
		plt.imshow(tem)
		plt.show()
		global readgoal
		rospy.loginfo('Map conversion completed')
		while not rospy.is_shutdown():
			if readgoal:#ゴールを受信
				rospy.loginfo('Receive goal')
				#Robotの座標取得
				try:
					t = tfBuffer.lookup_transform('map', 'base_link', rospy.Time())
					odom[0]=t.transform.translation.x
					odom[1]=t.transform.translation.y
				except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
					rospy.loginfo(e)
				print "start",odom,"goal",rgoal
				#座標変換
				start=[int((odom[1]/mapmsg.info.resolution+index_ox)),int((odom[0]/mapmsg.info.resolution+index_oy))]
				goal=[int((rgoal[1]/mapmsg.info.resolution+index_ox)),int((rgoal[0]/mapmsg.info.resolution+index_oy))]
				if not grid[goal[1],goal[0]] and not grid[start[1],start[0]]:
					#経路計算
					path=a_star_pathplanner(start,goal,grid.tolist())
					#経路配信
					path_generation(path,index_ox,index_oy,mapmsg.info.resolution)
				else:
					#Error
					if grid[goal[1],goal[0]]:
						plt.text(-3,-3, "Unreachable goal",color="red")
						rospy.logerr('Unreachable goal')
						grid[goal[1],goal[0]]=2
						plt.imshow(grid)
						plt.show()
						grid[goal[1],goal[0]]=1
					if grid[start[1],start[0]]:
						plt.text(-3,-3, "Unreachable start",color="red")
						rospy.logerr('Unreachable start')
						grid[start[1],start[0]]=2
						plt.imshow(grid)
						plt.show()
						grid[start[1],start[0]]=1
				readgoal=False

	def getImage():
		return self.rgb_image


def main(_):
	v=Map()
	rospy.spin()

if __name__=='__main__':
	main('_')