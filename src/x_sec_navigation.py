#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import random
from numpy.typing import NDArray   

import sys
print(sys.path)

from duckietown_msgs.msg import WheelsCmdStamped, WheelEncoderStamped
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion 
from std_msgs.msg import Bool
from enums import Path
from XsecNavigation import XsecNavigator



try:
    from cv_bridge import CvBridge
    CV_BRIDGE_AVAILABLE = True
except ModuleNotFoundError:
    CV_BRIDGE_AVAILABLE = False

from std_msgs.msg import Bool 

WITH_FEEDBACK = 0
WITH_TICKS = 1
WITH_TRAJ = 0

class XsecNavigation:
    
    def __init__(self):

        # set update rate
        self.update_rate = 50 #Hz
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
        
        #path params
        self.move_straight_params = get_rosparam("~paths/straight")
        self.move_right_params = get_rosparam("~paths/right")
        self.move_left_params = get_rosparam("~paths/left")
        
    def setup_publishers_and_subscribers(self) -> None:
        """
        Setup the ROS publishers and subscribers for the node.
        """

        self.sub_ticks_l = rospy.Subscriber(
            self.name_sub_tick_l_topic,
            WheelEncoderStamped,
            self.tick_l_callback,
            queue_size=10,
        )
        
        self.sub_ticks_r = rospy.Subscriber(
            self.name_sub_tick_r_topic,
            WheelEncoderStamped,
            self.tick_r_callback,
            queue_size=10,
        )
        
        self.sub_flag = rospy.Subscriber(
            self.name_sub_flag_topic,
            Bool,
            self.xsec_navigation_callback,
            queue_size=1,
        )
        
        self.pub_wheel_cmd = rospy.Publisher(
            self.name_pub_wheel_cmd_topic,
            WheelsCmdStamped,
            queue_size=10,
        )
        
        self.pub_flag = rospy.Publisher(
            self.name_pub_flag_topic,
            Bool,
            queue_size = 10
        )
        
    def tick_l_callback(self, msg):
        self.sub_ticks_l_msg = msg
        
    def tick_r_callback(self, msg):
        self.sub_ticks_r_msg = msg
        
    def xsec_navigation_callback(self, msg):
        wheel_cmd = WheelsCmdStamped()
        
        self.setup_params()
        
        rospy.loginfo(f"Waiting to receive messages from the topics {self.name_sub_tick_l_topic} and {self.name_sub_tick_r_topic} ")
        while not self.sub_ticks_r and not self.sub_ticks_r:
            rospy.sleep(0.2)
        rospy.loginfo(f"Received topics {self.name_sub_tick_l_topic} and {self.name_sub_tick_r_topic}!")
        
        if msg.data == True:
            #flag: true while navigating
            self.pub_flag.publish(True)
            #get next mission
            path = random.randint(0,2)
            
            path = 0
            print("init ticks: ", self.sub_ticks_l_msg.data, self.sub_ticks_r_msg.data)
            if path == Path.STRAIGHT.value: 
                print("Move straight")
                move = self.x_sec_navigator.Move(commands=self.x_sec_navigator.move_straight.commands, update_rate=self.update_rate, init_ticks=(self.sub_ticks_l_msg.data, self.sub_ticks_r_msg.data), resolution=self.sub_ticks_r_msg.resolution)
            elif path == Path.RIGHT.value: 
                print("Move right")
                move = self.x_sec_navigator.Move(commands=self.x_sec_navigator.move_right.commands, update_rate=self.update_rate, init_ticks=(self.sub_ticks_l_msg.data, self.sub_ticks_r_msg.data), resolution=self.sub_ticks_r_msg.resolution)
            elif path == Path.LEFT.value:
                print("Move left")
                move = self.x_sec_navigator.Move(commands=self.x_sec_navigator.move_left.commands, update_rate=self.update_rate, init_ticks=(self.sub_ticks_l_msg.data, self.sub_ticks_r_msg.data), resolution=self.sub_ticks_r_msg.resolution)
            else:
                # Print an error message if conversion fails
                print("Error: Invalid mission. Please enter a valid number.", file=sys.stderr)
                sys.exit(1)  # Exit with a non-zero code indicating an error
                
            
            if WITH_TRAJ:
                self.PD_controller(move, self.x_sec_navigator.trajectories[path])
            
            else:
                while not move.all_commands_excecuted:
                    
                    if WITH_FEEDBACK: #TODO
                        wheel_cmd = move.get_wheel_cmd_pose((self.sub_ticks_l_msg.data, self.sub_ticks_r_msg.data))
                    elif WITH_TICKS:
                        print(self.sub_ticks_l_msg.data, self.sub_ticks_r_msg.data)
                        wheel_cmd = move.get_wheel_cmd_ticks((self.sub_ticks_l_msg.data, self.sub_ticks_r_msg.data))
                    else:
                        # calculate wheel cmd
                        wheel_cmd = move.get_wheel_cmd()
                    self.pub_wheel_cmd.publish(wheel_cmd)
                    self.rate.sleep()
                    print("Driving ", wheel_cmd.vel_left, wheel_cmd.vel_right)
            
            print("Finished Path")
            #switch to lane following
            self.pub_flag.publish(False)
            wheel_cmd.vel_left = 0
            wheel_cmd.vel_right = 0
            self.pub_wheel_cmd.publish(wheel_cmd)
            
            #wait for state machine to set the flags
            self.name_sub_flag_topic == True
            rospy.sleep(2)
            
        else: 
            print("... wait for xsec GO-command ...")
            
            rospy.sleep(0.5)
            # self.cnt += 1
            
            
    def PD_controller(self, move, trajectory): 
        
        pos_tol = 0.01
        current_coord = np.array([0,0])
        
        for idx, coord in enumerate(trajectory):
            
            end_coord = np.array(coord)
            while np.linalg.norm(current_coord, end_coord) >= pos_tol: 
                
                wheel_cmd, current_coord = move.get_wheel_cmd_traj((self.sub_ticks_l_msg.data, self.sub_ticks_r_msg.data), current_coord, end_coord) 
                self.pub_wheel_cmd.publish(wheel_cmd)
                self.rate.sleep()
            
                
            
if __name__ == "__main__":
    rospy.init_node("x_sec_detection", anonymous=True)

    xsec_navigation = XsecNavigation()
    xsec_navigation.print_info()

    rospy.spin()
