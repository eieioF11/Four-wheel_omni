#!/usr/bin/env python
# -*- coding: utf-8 -*-
import numpy as np
from scipy.special import comb
import pandas as pd
import matplotlib.pyplot as plt

import glob
import os

import rospy
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped

class path_creator():
    def __init__(self,path_pub,index_ox,index_oy,resolution,start,field):
        self.path_pub=path_pub
        self.index_ox=index_ox
        self.index_oy=index_oy
        self.resolution=resolution
        self.dot=[]
        self.end=False
        self.start=start
        self.field=field
        self.boxsize=10

    def save_csv(self):
        # Save CSV path file
        cols = ["x", "y"]
        df = pd.DataFrame(self.ros_path,columns=cols)
        print(df)
        fpath=os.environ['HOME']+"/catkin_ws/src/Four-wheel_omni/scripts/csv/"
        fname=[]
        for f in glob.glob(fpath+'*.csv'):
            fname.append(int(os.path.splitext(os.path.basename(f))[0]))
        print(fname)
        n=0
        if len(fname):
            n=max(fname)+1
        df.to_csv(fpath+str(n)+".csv",index=False)

    def motion(self,event):
        x = event.xdata
        y = event.ydata
        self.ln_v.set_xdata(x)
        self.ln_h.set_ydata(y)

        self.rects.set_x(x-self.boxsize//2)
        self.rects.set_y(y-self.boxsize//2)

        if event.button == 1:
            pass
        if event.button == 3:
            pass
        plt.draw()

    def release(self,event):
        x = int(event.xdata)
        y = int(event.ydata)
        if event.button == 1:
            if not self.map[y,x] and not self.end:
                plt.title("Initial position addition with i key")
                if self.path==[]:
                    self.path=np.array([y,x])
                    dot,=plt.plot(self.path[1],self.path[0],"o",c="red")
                    self.dot.append(dot)
                else:
                    self.path=np.vstack((self.path,[y,x]))
                    dot,=plt.plot(self.path[:,1],self.path[:,0],"o",c="red")
                    self.dot.append(dot)
            else:
                plt.title("Error")
            print(self.dot,self.path)
        if event.button == 3:
            pass
            print(x,y)
        plt.draw()

    def onkey(self,event):
        print('you pressed', event.key, event.xdata, event.ydata)
        if not self.end:
            if event.key == 'p':
                self.line1,=plt.plot(self.path[:,1],self.path[:,0],c="red")
                self.path=self.pathconverter(self.path)
                self.line2,=plt.plot(self.path[:,1],self.path[:,0],c="Cyan")
                #????????????
                self.ros_path=self.path_generation(self.path,self.index_ox,self.index_oy,self.resolution)
                self.end=True
                self.save_csv()
                plt.title("Path generation is completed!")
            if event.key == 'i':
                if self.path==[]:
                    self.path=np.array(self.start)
                    dot,=plt.plot(self.path[1],self.path[0],"o",c="red")
                    self.dot.append(dot)
                else:
                    self.path=np.vstack((self.path,self.start))
                    dot,=plt.plot(self.path[:,1],self.path[:,0],"o",c="red")
                    self.dot.append(dot)
            if event.key == 'd':
                if self.path!=[]:
                    self.dot[-1].remove()
                    del self.dot[-1]
                    self.path=np.delete(self.path,-1, 0)
                    print(self.dot,self.path)
                else:
                    plt.title("Error")
        if event.key == 'n':
            if self.end:
                self.path=[self.path[-1,0],self.path[-1,1]]
                self.line1.remove()
                self.line2.remove()
                for d in self.dot:
                    d.remove()
                self.dot=[]
                self.end=False
                dot,=plt.plot(self.path[1],self.path[0],"o",c="Blue")
                self.dot.append(dot)
                plt.title("Initial position addition with i key")
            else:
                plt.title("Press P key")

        plt.draw()

    #path conversion

    def bernstein_poly(self,i, n, t):
        return comb(n, i) * ( t**(n-i) ) * (1 - t)**i

    def bezier_curve(self,points, nTimes=1000):
        nPoints = len(points)
        xPoints = np.array([p[0] for p in points])
        yPoints = np.array([p[1] for p in points])

        t = np.linspace(0.0, 1.0, nTimes)

        polynomial_array = np.array([ self.bernstein_poly(i, nPoints-1, t) for i in range(0, nPoints)   ])

        xvals = np.dot(xPoints, polynomial_array)
        yvals = np.dot(yPoints, polynomial_array)

        return xvals, yvals

    def pathconverter(self,path):
        xvals, yvals = self.bezier_curve(path, nTimes=1000)#?????????????????????????????????????????????
        cpath=np.flipud(np.array(list(map(list, zip(xvals,yvals)))))#xvals???yvals??????????????????
        #????????????
        print path
        print cpath
        return cpath

    #path generation
    def path_generation(self,sPath,ox,oy,resolution):#ROS???path???????????????????????????
        #Initialize odometry header
        global path_pub
        global head
        path = Path()
        path_header = Header()
        path_header.seq = 0
        path_header.stamp = rospy.Time.now()
        path_header.frame_id = "map"
        ros_path=[]
        for i in range(0,len(sPath)):
            temp_pose = PoseStamped()
            temp_pose.pose.position.x = (sPath[i][0]-ox)*resolution
            temp_pose.pose.position.y = (sPath[i][1]-oy)*resolution
            temp_pose.pose.position.z = 0
            temp_pose.header = path_header
            temp_pose.header.seq = i
            path.poses.append(temp_pose)
            ros_path.append([(sPath[i][0]-ox)*resolution,(sPath[i][1]-oy)*resolution])
        #print path.poses
        path.header = path_header
        self.path_pub.publish(path)
        rospy.loginfo('End path generation')
        return ros_path


    def create(self,map):
        self.map=map
        fig=plt.figure(figsize=(8,8))
        ax = fig.add_subplot(111)
        self.ln_v = plt.axvline(0)
        self.ln_h = plt.axhline(0)
        self.rects = plt.Rectangle((0,0),self.boxsize,self.boxsize,color='m',fill=False)
        ax.add_artist(self.rects)

        self.path=[]
        x=y=0
        if self.field=="r":
            x=970
            y=970
            w=270
            h=150
            plt.xlim(x,x+w)
            plt.ylim(y+h,y)
        elif self.field=="b":
            x=970
            y=880
            w=270
            h=150
            plt.xlim(x,x+w)
            plt.ylim(y+h,y)
        plt.text(x+10,y+10, "d:delete p:path generation n:create new path i:initial position" )
        #plt.imshow(self.map[y:y+h,x:x+w])
        plt.imshow(self.map)
        plt.title("Initial position addition with i key")
        plt.connect('motion_notify_event', self.motion)
        plt.connect('button_release_event', self.release)
        plt.connect('key_press_event', self.onkey)
        plt.show()
        print("end path create!")
        return


#Pcreat=path_creator()
#map = np.zeros((1000,1000))
#for i in range(100):
#    map[500+i,:]=255
#path=Pcreat.create(map)
#plt.imshow(map)
#plt.plot(path[:,1],path[:,0],c="red")
#plt.show()
