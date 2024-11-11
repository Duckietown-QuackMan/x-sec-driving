import rospy
import cv2
import numpy as np
import random
from numpy.typing import NDArray   

from duckietown_msgs.msg import WheelsCmdStamped, Pose2DStamped
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion 
from std_msgs.msg import Bool
from include.enums import Path
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
        self.update_rate = 60
        self.rate = rospy.Rate(self.update_rate)

        self.received_first_pose_msg = False
        if CV_BRIDGE_AVAILABLE:
            self.cv_bridge = CvBridge()
            
        self.setup_params()
        self.x_sec_navigator = XsecNavigator(self.move_straight_params, self.move_right_params, self.move_left_params)
        self.setup_publishers_and_subscribers()
        

    def print_info(self) -> None:

        print()
        print()
        rospy.loginfo(f"Running Xsec Navigation Node")
        print()
        print()

        if not self.received_first_pose_msg:
            rospy.loginfo(
                f"Waiting to receive messages from the topic {self.name_sub_pose_topic}"
            )

        while not self.received_first_pose_msg:
            rospy.sleep(0.2)

        rospy.loginfo(f"Received topic {self.name_sub_pose_topic}!")

    
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
        self.move_straight_params = get_rosparam("~paths/straight")
        self.move_right_params = get_rosparam("~topics/right")
        self.move_left_params = get_rosparam("~topics/left")
        
    def setup_publishers_and_subscribers(self) -> None:
        """
        Setup the ROS publishers and subscribers for the node.
        """

        self.sub_pose = rospy.Subscriber(
            self.name_sub_pose_topic,
            Pose2DStamped,
            queue_size=10,
        )
        
        self.sub_flag = rospy.Subscriber(
            self.name_sub_flag_topic,
            Bool,
            self.xsec_navigation_callback,
            queue_size=10,
        )
        
        self.pub_wheel_cmd = rospy.Publisher(
            self.name_pub_wheel_cmd_topic,
            WheelsCmdStamped,
            queue_size=1,
        )
        
        self.pub_flag = rospy.Publisher(
            self.name_pub_flag_topic,
            Bool,
            queue_size = 10
        )
        
    def xsec_navigation_callback(self):
        
        if self.name_sub_flag_topic == True:
            #get next mission
            path = random.randint(0,2)
             
            if path == Path.STRAIGHT: 
                print("Move straight")
                move = self.x_sec_navigator.Move(self.name_sub_pose_topic, self.x_sec_navigator.move_straight)
            elif Path.RIGHT: 
                print("Move right")
                move = self.x_sec_navigator.Move(self.name_sub_pose_topic, self.x_sec_navigator.move_right)
            elif Path.LEFT:
                print("Move left")
                move = self.x_sec_navigator.Move(self.name_sub_pose_topic, self.x_sec_navigator.move_left)
                
            self.x_sec_navigator.run_path()
            
            while not move.all_commands_excecuted:
                # calculate wheel cmd
                wheel_cmd = self.move.get_wheel_cmd(self.name_sub_pose_topic)
                self.pub_wheel_cmd.publish(wheel_cmd)
                self.rate.sleep()
            
            print("Finished Path")
            self.pub_flag.publish(True)
            
            #wait for state machine to set the flags
            rospy.sleep(2)
            
        else: 
            print("... wait for xsec GO-command ...")
            
        
if __name__ == "__main__":
    rospy.init_node("x_sec_detection", anonymous=True)

    xsec_navigation = XsecNavigation()
    xsec_navigation.print_info()

    rospy.spin()
