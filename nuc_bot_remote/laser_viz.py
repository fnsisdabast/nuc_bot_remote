#!/usr/bin/env python

import rospy
import roslib
import time
import os


import numpy as np
import math
import matplotlib.pyplot as plt

from std_msgs.msg import Int16, Float32MultiArray

clear = lambda: os.system('clear')

class Laser_viz():

        def __init__(self):
                rospy.init_node('laser_viz')
                self.nodename=rospy.get_name()
                rospy.loginfo("%s started" % self.nodename)
     
                self.angles=[11.25, 33.75, 56.25, 78.75, 101.25, 123.75, 146.25, 168.75, 191.25, 213.75,
                236.25, 258.75, 281.25, 303.75, 326.25, 348.75]
   
                self.oct_array=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

                self.fig = plt.figure()
                self.ax = plt.subplot(111)
                self.ax.set_ylim([-3, 3])   # set the bounds to be 10, 10
                self.ax.set_xlim([-3, 3])
                self.ax.set_autoscale_on(False)
                plt.ion()
                plt.show()
               
                rospy.Subscriber('octant_dist', Float32MultiArray, self.laser_oct_callback)

        def spin(self):
                while not rospy.is_shutdown():
                        self.spinOnce()
        
        def spinOnce(self):
                self.ax.clear()
                self.plot_point((0,0),0,2.5)
                self.plot_point((0,0),90,2.5)
                self.plot_point((0,0),180,2.5)
                self.plot_point((0,0),270,2.5)
                clear()
                for angle, octant in zip (self.angles, self.oct_array):
                        self.plot_point((0,0),angle, octant)
                        print "angle %0.2f dist %0.2f" % (angle, octant)
                self.fig.canvas.draw()
 
                

        def laser_oct_callback(self,msg):
                self.oct_array=msg.data

        def plot_point(self, point, angle, length):
                '''
                point - Tuple (x, y)
                angle - Angle you want your end point at in degrees.
                length - Length of the line you want to plot.

                Will plot the line on a 10 x 10 plot.
                '''

                # unpack the first point
                x, y = point

                # find the end point
                endy = length * math.sin(math.radians(angle))
                endx = length * math.cos(math.radians(angle))

                # plot the points

                self.ax.plot([x, endx], [y, endy])
 



if __name__ =='__main__':
        laser_viz=Laser_viz()
        laser_viz.spin()
        

     
     
