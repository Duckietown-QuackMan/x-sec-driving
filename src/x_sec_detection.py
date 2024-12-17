#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from numpy.typing import NDArray

try:
    from cv_bridge import CvBridge

    CV_BRIDGE_AVAILABLE = True
except ModuleNotFoundError:
    CV_BRIDGE_AVAILABLE = False

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Bool

from XsecDetector import (
    redline_detection,
    )

class XsecDetection:

    def __init__(self):
        self.rate = 0.1
        self.evaluate = False
        self.det_start_time = None 
        self.received_first_input_msg_image = False
        
        if CV_BRIDGE_AVAILABLE:
            self.cv_bridge = CvBridge()
        self.setup_params()
        self.setup_publishers_and_subscribers()

    def print_info(self) -> None:

        print()
        print()
        rospy.loginfo(f"Running Xsec Detection Node")
        print()
        print()

        if not self.received_first_input_msg_image:
            rospy.loginfo(
                f"Waiting to receive messages from the topic {self.name_sub_image_input_topic}"
            )

        while not self.received_first_input_msg_image:
            rospy.sleep(0.2)

        rospy.loginfo(f"Received topic {self.name_sub_image_input_topic}!")

    def setup_params(self) -> None:
        """
        Setup the parameters for the node reading them from the ROS parameter server.
        - self.name_sub_image_input_topic: name of the 3-channels input image topic
        - self.name_pub_image_red_line_topic: name of the output img of the red line
        """

        def get_rosparam(name):
            if rospy.has_param(name):
                param = rospy.get_param(name)
            else:
                txt_error = f"Parameter '{name}' is not set."
                rospy.logerr(txt_error)
                raise KeyError(txt_error)
            return param

        # variable params
        self.vehicle_name = rospy.get_param("~vehicle_name")

        # topics params
        self.name_sub_image_input_topic = self.vehicle_name + get_rosparam("~topics/sub/camera")
        self.name_pub_image_red_line_topic = self.vehicle_name + get_rosparam("~topics/pub/red_line")
        self.name_pub_bool_xsec_flag = get_rosparam("~topics/pub/xsec_flag")

    def setup_publishers_and_subscribers(self) -> None:
        """
        Setup the ROS publishers and subscribers for the node.
        """

        self.sub_image = rospy.Subscriber(
            self.name_sub_image_input_topic,
            CompressedImage,
            self.xsec_detection_callback,
            queue_size=1,
        )

        self.pub_image_red_line = rospy.Publisher(
            self.name_pub_image_red_line_topic,
            CompressedImage,
            queue_size=1,
        )
        
        self.pub_bool_xsec_flag = rospy.Publisher(
            self.name_pub_bool_xsec_flag,
            Bool,
            queue_size=1,
        )

    def publish_red_line(
        self,
        image: NDArray[np.uint8],
        input_msg_stamp: rospy.Time,
    ) -> None:
        """
        Publish the red line masks as compressed images,
        with the same timestamp as the input image to be able to match the
        masks with the input image for the evaluation.

        Args:
            red_line: 1-channel compressed image the red line
            input_msg_stamp (rospy.Time): ROS timestamp of the input image
        """

        if CV_BRIDGE_AVAILABLE:
            red_line_msg = self.cv_bridge.cv2_to_compressed_imgmsg(
                255 * image.astype(np.uint8), dst_format="png"
            )
        else:
            red_line_msg = CompressedImage()
            red_line_msg.format = "png"
            ext_format = '.' + red_line_msg.format
            red_line_msg.data = np.array(
                cv2.imencode(ext_format, 255 * image.astype(np.uint8))[1]
            ).tobytes()
        red_line_msg.header.stamp = input_msg_stamp

        self.pub_image_red_line.publish(red_line_msg)
        
        
    def publish_xsec_flag(
        self,
        flag: np.bool
    ) -> None:
        self.pub_bool_xsec_flag.publish(flag)
        
        

    def xsec_detection_callback(self, msg: CompressedImage) -> None:
        """Callback for the input image subscriber.
        Detects red lines in the input image,
        and publish a detection flag.

        Args:
            msg (CompressedImage): input image message
        """
        
        #Limit callback to rate frequency
        if self.det_start_time is None:
            self.det_start_time = rospy.Time.now()
        else:
            if rospy.Time.now() - self.det_start_time < rospy.Duration.from_sec(self.rate):
                return
            else:
                self.det_start_time = rospy.Time.now()
            
            #Detect redlines in town through pixel count
            img = self.cv_bridge.compressed_imgmsg_to_cv2(msg)
            x_sec_flag, img = redline_detection(img)
            self.publish_xsec_flag(x_sec_flag)
            
            # Debug 
            if self.evaluate:
                rospy.loginfo(f"image timestampe: {input_msg_stamp.to_sec()}")
                #Publish eval image
                input_msg_stamp = msg.header.stamp
                self.publish_red_line(img, input_msg_stamp)

if __name__ == "__main__":
    rospy.init_node("x_sec_detection", anonymous=True)
    #build xsection detection library
    xsec_detection = XsecDetection()
    xsec_detection.print_info()

    rospy.spin()
