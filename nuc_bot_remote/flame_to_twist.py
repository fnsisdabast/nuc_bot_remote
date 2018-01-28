#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist



def keys_cb(msg,twist_pub):
    if len(msg.data)==0 or not key_mapping.has_key(msg.data[0]):
        return # unknown key
    vels=key_mapping[msg.data[0]]
    t=Twist()
    t.angular.z=vels[0]
    t.linear.x=vels[1]

    twist_pub.publish(t)

if __name__ == '__main__' :
    rospy.init_node('ir_to_twist')
    twist_pub=rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.Subscriber('flame_sub', String, keys_cb,twist_pub)
    rospy.spin()
