#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import random
from numpy.typing import NDArray   

import sys
print(sys.path)

from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped, Twist2DStamped
from std_msgs.msg import Bool
from enums import Path
from XsecNavigation import XsecNavigator



try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except ModuleNotFoundError:
    CV_BRIDGE_AVAILABLE = False

from std_msgs.msg import Bool 

class XsecNavigation:
    
    def __init__(self):

        # set update rate
        self.update_rate = 20 #Hz
        self.rate = rospy.Rate(self.update_rate)

        self.received_first_pose_msg = False
        if CV_BRIDGE_AVAILABLE:
            self.cv_bridge = CvBridge()
            
        self.setup_params()
            
        self.x_sec_navigator = XsecNavigator(
            self.move_straight_params, 
            self.move_right_params, 
            self.move_left_params,
            )
        
        self.setup_publishers_and_subscribers()
        

    def print_info(self) -> None:

        print()
        print()
        rospy.loginfo(f"Running Xsec Navigation Node")
        print()
        print()

        rospy.loginfo(
                f"Waiting to receive messages from the topic {self.name_sub_flag_topic}"
            )

        while not self.sub_flag:
            rospy.sleep(0.2)

        rospy.loginfo(f"Received topic {self.name_sub_flag_topic}!")

    
    def setup_params(self) -> None:
        """
        Setup the parameters for the node reading them from the ROS parameter server.
        """

        def get_rosparam(name):
            if rospy.has_param(name):
                param = rospy.get_param(name)
            else:
                txt_error = f"Parameter '{name}' is not set."
                rospy.logerr(txt_error)
                raise KeyError(txt_error)
            return param

        # paths params
        self.vehicle_name = get_rosparam("~vehicle_name")
        #--sub
        self.name_sub_flag_topic = get_rosparam("~topics/sub/flag")
        self.name_sub_tick_l_topic = self.vehicle_name + get_rosparam('~topics/sub/ticks_left')
        self.name_sub_tick_r_topic = self.vehicle_name + get_rosparam('~topics/sub/ticks_right')
        #--pub
        self.name_pub_flag_topic = get_rosparam("~topics/pub/flag")
        self.name_pub_wheel_cmd_topic = self.vehicle_name + get_rosparam("~topics/pub/wheels_cmd")
        self.name_pub_wheel_adj_topic = self.vehicle_name + get_rosparam("~topics/pub/wheels_adj")
        
        #path params
        self.move_straight_params = get_rosparam("~paths/straight")
        self.move_right_params = get_rosparam("~paths/right")
        self.move_left_params = get_rosparam("~paths/left")
        
    def setup_publishers_and_subscribers(self) -> None:
        """
        Setup the ROS publishers and subscribers for the node.
        """
        #left ticks
        self.sub_ticks_l = rospy.Subscriber(
            self.name_sub_tick_l_topic,
            WheelEncoderStamped,
            self.tick_l_callback,
            queue_size=1,
        )
        #right ticks
        self.sub_ticks_r = rospy.Subscriber(
            self.name_sub_tick_r_topic,
            WheelEncoderStamped,
            self.tick_r_callback,
            queue_size=1,
        )
        #xsection navigation start flag
        self.sub_flag = rospy.Subscriber(
            self.name_sub_flag_topic,
            Bool,
            self.xsec_navigation_callback,
            queue_size=1,
        )
        #wheel command topic (setting velocity and angular velocity)
        self.pub_wheel_cmd = rospy.Publisher(
            self.name_pub_wheel_cmd_topic,
            Twist2DStamped,
            queue_size=10,
        )
        #wheel command topic (setting left and right wheel motor speed)
        self.pub_wheel_adj = rospy.Publisher(
            self.name_pub_wheel_adj_topic,
            WheelsCmdStamped,
            queue_size=10,
        )
        #xsection navigation state flag
        self.pub_flag = rospy.Publisher(
            self.name_pub_flag_topic,
            Bool,
            queue_size = 10
        )
    #left ticks callback  
    def tick_l_callback(self, msg):
        self.sub_ticks_l_msg = msg
    #right ticks callback
    def tick_r_callback(self, msg):
        self.sub_ticks_r_msg = msg
        
    def xsec_navigation_callback(self, msg):
        """xsection navigation callback fct
        Publishes the state of the navigation to the state machine. 
        Args:
            msg (Bool): Sets if ghostbot should switch into xsection navigation.
            
        """
        wheel_cmd = Twist2DStamped() 
        wheel_adj = WheelsCmdStamped()       
        
        while not self.sub_ticks_r and not self.sub_ticks_r:
            rospy.loginfo(f"Waiting to receive messages from the topics {self.name_sub_tick_l_topic} and {self.name_sub_tick_r_topic}")
            rospy.sleep(0.2)
        
        # get x_sec_go flag from state_machine
        if msg.data == True:
            #flag: true while navigating
            self.pub_flag.publish(True)
            #get next mission randomly
            path = random.randint(0,2)
            #create mission commands
            if path == Path.STRAIGHT.value: 
                rospy.loginfo("Move straight")
                move = self.x_sec_navigator.Move(commands=self.x_sec_navigator.move_straight.commands, init_ticks=(self.sub_ticks_l_msg.data, self.sub_ticks_r_msg.data), resolution=self.sub_ticks_r_msg.resolution)
            elif path == Path.RIGHT.value: 
                rospy.loginfo("Move right")
                move = self.x_sec_navigator.Move(commands=self.x_sec_navigator.move_right.commands, init_ticks=(self.sub_ticks_l_msg.data, self.sub_ticks_r_msg.data), resolution=self.sub_ticks_r_msg.resolution)
            elif path == Path.LEFT.value:
                rospy.loginfo("Move left")
                move = self.x_sec_navigator.Move(commands=self.x_sec_navigator.move_left.commands, init_ticks=(self.sub_ticks_l_msg.data, self.sub_ticks_r_msg.data), resolution=self.sub_ticks_r_msg.resolution)
            else:
                # Print an error message if conversion fails
                rospy.loginfo("Error: Invalid mission. Please enter a valid number.", file=sys.stderr)
                sys.exit(1)  # Exit with a non-zero code indicating an error
                
            while not move.all_commands_excecuted:
                # wheel vel control with encoder ticks feedback
                wheel_cmd, wheel_adj, fine_adjust = move.get_wheel_cmd_ticks((self.sub_ticks_l_msg.data, self.sub_ticks_r_msg.data))
                    
                if fine_adjust:
                    self.pub_wheel_adj.publish(wheel_adj)
                else:
                    self.pub_wheel_cmd.publish(wheel_cmd)
                
                self.rate.sleep()
                
            # stop the bot's navigation commands
            #switch to lane following
            self.pub_flag.publish(False)
            wheel_cmd.v = 0
            wheel_cmd.omega = 0
            wheel_adj.vel_left = 0
            wheel_adj.vel_right = 0
            self.pub_wheel_cmd.publish(wheel_cmd)
            self.pub_wheel_adj.publish(wheel_adj)
            #wait before entering back to x-section detection
            rospy.sleep(2)
            
        else: 
            rospy.sleep(0.5)
              
if __name__ == "__main__":
    rospy.init_node("x_sec_detection", anonymous=True)
    #build xsec navigation librabry
    xsec_navigation = XsecNavigation()
    xsec_navigation.print_info()

    rospy.spin()
