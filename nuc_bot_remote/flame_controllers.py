#!/usr/bin/env python

import rospy
import roslib
import time

from std_msgs.msg import Int16, Float32MultiArray
from geometry_msgs.msg import Twist


######################################################
######################################################
class IR_flame():
######################################################
######################################################


    #####################################################
    def __init__(self):
    #####################################################
        rospy.init_node('find_flame')
        self.nodename=rospy.get_name()
        rospy.loginfo("%s started" % self.nodename)

        ### initialize variables
	self.dist=0
        self.t=Twist()
        self.right_flame_val=0
        self.center_flame_val=0
        self.left_flame_val=0
        self.robot_angle_val=0
        self.oct_array=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

        self.circle_array=[]
        self.range_of_angles=[]
        for i in range(0,360):
            self.range_of_angles.append(i)
        self.circle_array=3*self.range_of_angles

        ### subscribers/publishers
        rospy.Subscriber('right_flame_val', Int16, self.r_callback)
        rospy.Subscriber('center_flame_val', Int16, self.c_callback)
        rospy.Subscriber('left_flame_val', Int16, self.l_callback)
        rospy.Subscriber('robot_angle_val', Int16, self.a_callback)
        rospy.Subscriber('octant_dist', Float32MultiArray, self.laser_oct_callback)
        self.twist_pub=rospy.Publisher('cmd_vel', Twist, queue_size=1)

    #####################################################
    def spin(self):
    #####################################################
#        print "angle test"
#        print "nudge"
#        self.nudge_right()
#        time.sleep(3)
#        print "45"
#        self.go_angle(45)
#        time.sleep(3)
#        print "90"
#        self.go_angle(90)
#        time.sleep(3)
#        print "180"
#        self.go_angle(180)
#        time.sleep(3)
        while not rospy.is_shutdown():
          #  raw_input("Press Enter to continue...")
            self.spinOnce()

    #####################################################
    def spinOnce(self):
    #####################################################
	self.right_avg=(self.oct_array[15]+self.oct_array[14])/2		
	self.left_avg=(self.oct_array[8]+self.oct_array[9])/2
	self.midright_avg=(self.oct_array[1]+self.oct_array[0])/2		
	self.midleft_avg=(self.oct_array[6]+self.oct_array[7])/2
	if (self.oct_array[3]>.3 and self.oct_array[4]>.3):	#if way ahead is clear and not too close to sides, go straight        
		if self.midright_avg <.2:
                    print "Nudge to left"
                    self.nudge_left()
                elif self.midleft_avg <.2:
                    print "Nudge to right"
                    self.nudge_right()


                elif self.oct_array[4]>3*self.oct_array[3]:
                    print "Slight left"
                    self.nudge_left()
                elif self.oct_array[3]>3*self.oct_array[4]:
                    print "Slight right"
                    self.nudge_right()              

                else:
                    print "Going Straight"
                    self.go_straight()
	else:									#if not
                if self.midright_avg>.6:
                    print "45 degrees right"
                    self.go_angle(45)
                elif self.midleft_avg>.6:
                    print "45 degrees left"
                    self.go_angle(-45)
		elif self.right_avg>.6:						#if can go, go right preferentially 
                    print "90 degrees right"
		    self.go_angle(90)
		elif self.left_avg>.6:
                    print "90 degrees left"
                    self.go_angle(-90)
                else:
                    print "Turn Around"
                    self.go_angle(180)

	
        
    #####################################################                     
    ### Callbacks
    #####################################################
    def r_callback(self, msg):
        self.right_flame_val=msg.data

    def c_callback(self, msg):
        self.center_flame_val=msg.data

    def l_callback(self, msg):
        self.left_flame_val=msg.data

    def a_callback(self, msg):
        self.robot_angle_val=msg.data

    def laser_oct_callback(self, msg):
        self.oct_array=msg.data
    #####################################################

    #####################################################
    ### Robot Movement Actions
    #####################################################
    def escape(self):
        self.t.angular.z=2
        self.t.linear.x=0
        self.twist_pub.publish(self.t)

    def go_right(self):
        self.t.angular.z=2
        self.t.linear.x=.4
        self.twist_pub.publish(self.t)

    def rotate_right(self):
        self.t.angular.z=2
        self.t.linear.x=0
        self.twist_pub.publish(self.t)
    
    def go_left(self):
        self.t.angular.z=-2
        self.t.linear.x=.4
        self.twist_pub.publish(self.t)

    def rotate_left(self):
        self.t.angular.z=-2
        self.t.linear.x=0
        self.twist_pub.publish(self.t)

    def go_straight(self):
        self.t.angular.z=0
        self.t.linear.x=.05
        self.twist_pub.publish(self.t)

    def nudge_right(self):
        self.t.angular.z=-1
        self.t.linear.x=.05
        self.twist_pub.publish(self.t)
        time.sleep(.3)
        self.t.angular.z=0
        self.t.linear.x=.05
        self.twist_pub.publish(self.t)

    def nudge_left(self):
        self.t.angular.z=1
        self.t.linear.x=0.05
        self.twist_pub.publish(self.t)
        time.sleep(.2)
        self.t.angular.z=0
        self.t.linear.x=0.05
        self.twist_pub.publish(self.t)

    def angle_converter(self, start_angle, angle_to_turn):
        end_angle_val=start_angle+angle_to_turn
        if (end_angle_val <0):
            end_angle_val=end_angle_val+360
        if (end_angle_val >=360):
            end_angle_val=end_angle_val-360
        return end_angle_val
    
    def go_angle(self, angle_to_turn):
        robot_start_angle=self.robot_angle_val
        robot_end_angle=self.angle_converter(robot_start_angle, angle_to_turn)
        end_index=robot_end_angle+340
        end_range=self.circle_array[end_index-3:end_index+4]
 #       print"start angle:", robot_start_angle, "end angle:", robot_end_angle, "range:", end_range
 #       last_print=1000
        while (1):
            if self.robot_angle_val in end_range:
                self.t.angular.z=0
                self.t.linear.x=0
                self.twist_pub.publish(self.t)
                return(0)
               # print"end angle:",self.robot_angle_val
               # quit()
         #   if self.robot_angle_val != last_print:
          #      last_print=self.robot_angle_val
            #    print self.robot_angle_val,
            if angle_to_turn>0:
                self.t.angular.z=-2
                self.t.linear.x=0
                self.twist_pub.publish(self.t)             
            else:
                self.t.angular.z=2
                self.t.linear.x=0
                self.twist_pub.publish(self.t)                

            
        
        
    #####################################################


    #####################################################
    ### Robot Movement Logic
    #####################################################

    def flame_present(self):
        if (self.right_flame_val==True) or (self.left_flame_val==True) or (self.center_flame_val>200):
            return True
        return False
                      
    def close_enough(self):
        for self.dist in self.oct_array:
            if self.dist <.1:
                print ('close enough')
                return True
        return False

    def hungup(self):
	print(self.oct_array)
        for self.dist in self.oct_array:
	    print(self.dist)
            if self.dist <.25:
		print ('hung up')
		print (self.dist)
                return True                
        return False

    def move(self):
        print self.oct_array
        if self.oct_array[4] >.5:
            self.go_straight();
        elif self.oct_array[0] >.5:
            self.go_right();
        else:
            self.go_left();
    #####################################################  



if __name__ == '__main__':
    """ main """
    print "starting"
    iR_flame = IR_flame()
    iR_flame.spin()
            
    
    
