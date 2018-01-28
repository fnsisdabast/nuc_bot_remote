#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16, Float32MultiArray
from geometry_msgs.msg import Twist

t=Twist()
right_flame_val=0
center_flame_val=0
left_flame_val=0
oct_array=[0,0,0,0,0,0,0,0]



def r_callback(msg):
    print('r cb')
    right_flame_val=msg.data

def c_callback(msg):
    center_flame_val=msg.data

def l_callback(msg):
    left_flame_val=msg.data

def laser_oct_callback(msg):
    print('oct callback')
    oct_array=msg.data

def flame_present():
    if (right_flame_val>100) or (left_flame_val>100) or (center_flame_val>100):
        return True
    return False

def close_enough():
    for dist in oct_array:
        if dist <.1:
            return True
        print ('close enough')
    return false

def hungup():
    for dist in oct_array:
        if dist <.25:
            return True
    print ('not hung up')
    return False

def move():
    print oct_array
    if oct_array[0] >.5:
        go_right();
    elif oct_array[4] >.5:
        go_straight();
    else:
        go_left();
    



def escape():
    t.angular.z=2
    t.linear.x=-.4
    twist_pub.publish(t)

def go_right():
    t.angular.z=2
    t.linear.x=.4
    twist_pub.publish(t)
    
def go_left():
    t.angular.z=2
    t.linear.x=.4
    twist_pub.publish(t)

def go_striaght():
    t.angular.z=2
    t.linear.x=.4
    twist_pub.publish(t)
    





rospy.init_node('find_flame')

right_flame_sub=rospy.Subscriber('right_flame_val', Int16, r_callback)
center_flame_sub=rospy.Subscriber('center_flame_val', Int16, c_callback)
left_flame_sub=rospy.Subscriber('left_flame_val', Int16, l_callback)
laser_oct_sub=rospy.Subscriber('octant_dist', Float32MultiArray, laser_oct_callback)
twist_pub=rospy.Publisher('cmd_vel', Twist, queue_size=1)

while not rospy.is_shutdown():
    if flame_present() and not close_enough():
        print ('found flame')
        if (right_flame_val >100) :
            go_right()
        elif (left_flame_val >100):
            go_left()
        else:
            go_straight()
    else:
        if not hungup():
            print ('searching')
            move ()
        else:
            print ('oh no!!')
            escape ()
            
            
    
    
