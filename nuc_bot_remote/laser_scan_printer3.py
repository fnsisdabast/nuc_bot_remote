#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import LaserScan

def laser_callback(scan):
    depths = []
    for dist in scan.ranges:                    #get all of the depths from laser scanner
        depths.append(dist)
    depth_octants=[]
    depth_octants.append(depths[90:112])        #divide them into 16 bins
    depth_octants.append(depths[112:135])
    depth_octants.append(depths[135:157])
    depth_octants.append(depths[157:180])
    depth_octants.append(depths[180:203])
    depth_octants.append(depths[203:225])
    depth_octants.append(depths[225:248])
    depth_octants.append(depths[248:270])
    depth_octants.append(depths[270:293])
    depth_octants.append(depths[293:315])
    depth_octants.append(depths[315:338])
    depth_octants.append(depths[338:360])
    depth_octants.append(depths[0:23])
    depth_octants.append(depths[23:45])
    depth_octants.append(depths[45:68])
    depth_octants.append(depths[68:90])
    octant_avg=[]
    for octants in depth_octants:               #average each bin
        if len(octants)>0:
            depth_running=0
            num_depths=0
            for depth_vals in octants:
                if not np.isnan(depth_vals):    #if it is a number
                    if not np.isinf(depth_vals):  #and is not infinity, add it to the sum
                        depth_running=depth_running+depth_vals
                    else:                            #otherwise just skip it
                        num_depths=num_depths-1
                    num_depths=num_depths+1
            octant_avg.append(depth_running/num_depths)  #append to array of averages
    array_to_pub=Float32MultiArray(data=octant_avg)      #publish array of averages
    pub.publish(array_to_pub)

def laser_subscriber():
    rospy.init_node('laser_subscriber')
    sub=rospy.Subscriber('scan', LaserScan, laser_callback)
    pub=rospy.Publisher('octant_dist',Float32MultiArray, queue_size=1)
    rospy.spin()

if __name__ == '__main__':
    laser_subscriber()
