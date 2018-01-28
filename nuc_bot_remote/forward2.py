#!/usr/bin/env python

import roslib
import rospy
import PyKDL
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi


rospy.init_node('forward', anonymous=False)
cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
tf_listener = tf.TransformListener()
odom_frame = '/odom'
tf_listener.waitForTransform(odom_frame, '/base_link', rospy.Time(), rospy.Duration(1.0))
base_frame='/base_link'
position=Point()
move_cmd=Twist()
move_cmd.linear.x=.5

def quat_to_angle(quat):
    kdl_rot=PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return kdl_rot.GerRPY()[2]
    

def get_odom():
    (trans, rot)=tf_listener.lookupTransform(odom_frame, base_frame, rospy.Time(0))
    tf_quat=Quaternion(*rot)
    return (Point(*trans), quat_to_angle(tf_quat))




(position, rotoation)=get_odom()
x_start = position.x
y_start = position.y
ang_start = rotation

distance=0

while distance<.5:
    cmd_vel.publish(move_cmd)
    (position, rotoation)=get_odom()
    distance = sqrt(pow((position.x - x_start), 2) + 
                                pow((position.y - y_start), 2))


move_cmd = Twist()
cmd_vel.publish(move_cmd)
rospy.sleep(1)
    
