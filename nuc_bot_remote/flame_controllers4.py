#!/usr/bin/env python

import rospy
import roslib
import time
import PyKDL
from random import *

from std_msgs.msg import Int16, Float32MultiArray, Bool
from geometry_msgs.msg import Twist, Point, Quaternion
import tf
from math import radians, copysign, sqrt, pow, pi


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
        self.base_linear=0.05
        self.base_angular=2
        self.angular_tolerance=radians(2.5)
        self.dist=0
        self.t=Twist()
        self.right_flame_val=0
        self.center_flame_val=0
        self.left_flame_val=0
        self.uv_front=False
        self.uv_side=False
        self.robot_angle_val=0
        self.oct_array=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        #self.uv_angle_array=[[-90,0],[-75,0],[-60,0],[-45,0],[-30,0],[-15,0],[0,0],[15,0],[30,0],[45,0],[60,0],[75,0],[90,0]]
        self.uv_angle_array=[[30,0],[45,0],[60,0],[75,0],[90,0]]
        self.position=Point()

        self.circle_array=[]
        self.range_of_angles=[]
        for i in range(0,360):
            self.range_of_angles.append(i)
        self.circle_array=3*self.range_of_angles

        ### subscribers/publishers
        rospy.Subscriber('right_flame_val', Int16, self.r_callback)
        rospy.Subscriber('center_flame_val', Int16, self.c_callback)
        rospy.Subscriber('left_flame_val', Int16, self.l_callback)
        rospy.Subscriber('uv_front', Bool, self.uv_front_callback)
        rospy.Subscriber('uv_side', Bool, self.uv_side_callback)
        rospy.Subscriber('robot_angle_val', Int16, self.a_callback)
        rospy.Subscriber('octant_dist', Float32MultiArray, self.laser_oct_callback)
        self.twist_pub=rospy.Publisher('cmd_vel', Twist, queue_size=1)

        ##TF and listener stuff
        self.tf_listener = tf.TransformListener()
        self.odom_frame = '/odom'
        self.base_frame='/base_link'
        self.tf_listener.waitForTransform(self.odom_frame, self.base_frame, rospy.Time(), rospy.Duration(1.0))
       

    #####################################################
    def spin(self):
    #####################################################
    #    while not rospy.is_shutdown():
          #  raw_input("Press Enter to continue...")
         #   self.spinOnce()
         
        self.tf_go_angle(pi/8)

    #####################################################
    def spinOnce(self):
    #####################################################

      #  self.rotate_to_flame()
      self.tf_go_angle(pi/4)
        
        
    #####################################################                     
    ### Callbacks
    #####################################################
    def r_callback(self, msg):
        self.right_flame_val=msg.data

    def c_callback(self, msg):
        self.center_flame_val=msg.data

    def uv_front_callback(self, msg):
        self.uv_front=msg.data

    def uv_side_callback(self, msg):
        self.uv_side=msg.data

    def l_callback(self, msg):
        self.left_flame_val=msg.data

    def a_callback(self, msg):
        self.robot_angle_val=msg.data

    def laser_oct_callback(self, msg):
        self.oct_array=msg.data
    #####################################################


    #####################################################
    ### Robot Movement Behaviors
    #####################################################

    def random_walk(self):
        if self.uv_side !=True:
            self.right_avg=(self.oct_array[15]+self.oct_array[14])/2        
            self.left_avg=(self.oct_array[8]+self.oct_array[9])/2
            self.midright_avg=(self.oct_array[1]+self.oct_array[0])/2       
            self.midleft_avg=(self.oct_array[6]+self.oct_array[7])/2
    #        if self.right_avg>.6 and self.uv_side==True:
    #            print("Fire to the right")
    #           self.go_angle(45)
            if (self.oct_array[3]>.2 and self.oct_array[4]>.2): #if way ahead is clear and not too close to sides, go straight        
                if (self.oct_array[1]<0.125 or self.oct_array[2]<0.125 or self.oct_array[0] <.125 or self.oct_array[15] <.125):
                        print "One too close Nudge to left"
                        self.nudge_left()
                elif (self.oct_array[6] <.125 or self.oct_array[5] <.125 or self.oct_array[7] <.125 or self.oct_array[8] <.125):
                        print "One too close Nudge to right"
                        self.nudge_right()
                elif self.midright_avg <.2:
                        print "Midright Nudge to left"
                        self.nudge_left()
                elif self.midleft_avg <.2:
                        print "Midright Nudge to right"
                        self.nudge_right()
                elif self.oct_array[4]>3*self.oct_array[3]:
                        print "Forward farter Slight left"
                        self.nudge_left()
                elif self.oct_array[3]>3*self.oct_array[4]:
                        print "Forward farther Slight right"
                        self.nudge_right()           
                else:
                       # print "Going Straight"
                        self.go_straight()
            else:                           #if not
                if random()<.5:
                    if self.midright_avg>.4:
                        print "45 degrees right"
                        self.go_angle(45)
                    elif self.midleft_avg>.4:
                        print "45 degrees left"
                        self.go_angle(-45)
                    elif self.right_avg>.4:                     #if can go, go right preferentially 
                        print "90 degrees right"
                        self.go_angle(90)
                    elif self.left_avg>.4:
                        print "90 degrees left"
                        self.go_angle(-90)
                    else:
                        print "Turn Around"
                        self.go_angle(180)
                else:
                    if self.midleft_avg>.4:
                        print "45 degrees left"
                        self.go_angle(-45)
                    elif self.midright_avg>.4:
                        print "45 degrees right"
                        self.go_angle(45)
                    elif self.left_avg>.4:
                        print "90 degrees left"
                        self.go_angle(-90)
                    elif self.right_avg>.4:                     #if can go, go right preferentially 
                        print "90 degrees right"
                        self.go_angle(90)          
                    else:
                        print "Turn Around"
                        self.go_angle(180)


    def count_front(self):
        counts=0
        print "in counts"
        t_end = time.time() + 10
        while time.time() < t_end:
            if self.uv_side==True:
                counts=counts+1
        print "counts",counts
        return counts
        
    def rotate_to_flame(self):
        for i, ang_info in enumerate(self.uv_angle_array):
            print ang_info
            self.tf_go_angle(radians(ang_info[0]))
            count=self.count_front()
            angle=(ang_info[1])
            self.uv_angle_array[i]=[angle,count]
        print self.uv_angle_array
            
                
    #####################################################
        


    #####################################################
    ### Robot Movement Actions
    #####################################################

    def stop(self):
        self.t.angular.z=0
        self.t.linear.x=0
        self.twist_pub.publish(self.t)
        
    def escape(self):
        self.t.angular.z=self.base_angular
        self.t.linear.x=0
        self.twist_pub.publish(self.t)

    def go_right(self):
        self.t.angular.z=self.base_angular
        self.t.linear.x=.4
        self.twist_pub.publish(self.t)

    def rotate_right(self):
        self.t.angular.z=self.base_angular
        self.t.linear.x=0
        self.twist_pub.publish(self.t)
    
    def go_left(self):
        self.t.angular.z=-1*self.base_angular
        self.t.linear.x=.4
        self.twist_pub.publish(self.t)

    def rotate_left(self):
        self.t.angular.z=-1*self.base_angular
        self.t.linear.x=0
        self.twist_pub.publish(self.t)

    def go_straight(self):
        self.t.angular.z=0
        self.t.linear.x=self.base_linear
        self.twist_pub.publish(self.t)

    def go_forward(self,dist):
        (position, rotoation)=self.get_odom()
        x_start = position.x
        y_start = position.y
        ang_start = rotation
        distance=0
        while distance<dist:
            self.t.angular.z=0
            self.t.linear.x=self.base_linear
            self.twist_pub.publish(self.t)
            (position, rotoation)=self.get_odom()
            distance = sqrt(pow((position.x - x_start), 2) +
                           pow((position.y - y_start), 2))
        self.stop()
        rospy.sleep(1)

    def tf_go_angle(self, goal_angle):
        start_angle=self.get_angle()
        print"start angle: " ,start_angle, "goal turn: ", goal_angle, "end angle: ", self.normalize_angle(start_angle+goal_angle)
        last_angle=start_angle
        turn_angle=0
        while abs(turn_angle+self.angular_tolerance)<abs(goal_angle):
            self.t.angular.z=.25*self.base_angular
            self.t.linear.x=0
            self.twist_pub.publish(self.t)
            new_rotation=self.get_angle()
            delta_angle = self.normalize_angle(new_rotation - last_angle)
            turn_angle += delta_angle
            print"current angle: ",new_rotation, "amount turned: ", delta_angle
            last_angle = new_rotation
        #    raw_input("Press Enter to continue...")
        self.stop()
        rospy.sleep(1)

    def nudge_right(self):
        self.t.angular.z=-.5*self.base_angular
        self.t.linear.x=self.base_linear
        self.twist_pub.publish(self.t)
        time.sleep(.15)
        self.stop()

    def nudge_left(self):
        self.t.angular.z=.5*self.base_angular
        self.t.linear.x=self.base_linear
        self.twist_pub.publish(self.t)
        time.sleep(.15)
        self.stop()

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
        while (1):
            if self.robot_angle_val in end_range:
                self.stop()
                return(0)
            if angle_to_turn>0:
                self.t.angular.z=-1*self.base_angular
                self.t.linear.x=0
                self.twist_pub.publish(self.t)             
            else:
                self.t.angular.z=self.base_angular
                self.t.linear.x=0
                self.twist_pub.publish(self.t)                        
    #####################################################


    #####################################################
    ### Robot Movement Logic
    #####################################################


        

    #####################################################  


    #####################################################
    ### Support Logic
    #####################################################

    def quat_to_angle(self, quat):
        kdl_rot=PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
     #   print kdl_rot.GetRPY() 
        return kdl_rot.GetRPY()[2]   

    def get_odom(self):
        (trans, rot)=self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        #print rot
        tf_quat=Quaternion(*rot)
        angle=self.quat_to_angle(tf_quat)
   #     print angle
        return (Point(*trans), angle)

    def get_angle(self):
        (trans, rot)=self.tf_listener.lookupTransform(self.odom_frame, self.base_frame, rospy.Time(0))
        tf_quat=Quaternion(*rot)
        angle=self.quat_to_angle(tf_quat)
        return (angle)

    def normalize_angle(self, angle):
        res = angle
        while res > pi:
            res -= 2.0 * pi
        while res < -pi:
            res += 2.0 * pi
        return res
    #####################################################  





if __name__ == '__main__':
    """ main """
    print "starting"
    iR_flame = IR_flame()
    iR_flame.spin()
            
    
    
