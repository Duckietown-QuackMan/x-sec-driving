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
    line_detection,
    plot_lines,
    XSecTile,
    evaluate
    )


class XsecDetection:

    def __init__(self):

        
        self.callback_rate = 20
        self.cnt = 0
        self.evaluate = True
        
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
        - self.name_pub_image_red_lines_mask_topic: name of the 1-channel output mask for red lines
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
        self.name_pub_image_red_lines_mask_topic = self.vehicle_name + get_rosparam("~topics/pub/red_lines_mask")
        self.name_pub_image_xsec_eval_topic = self.vehicle_name + get_rosparam("~topics/pub/xsec_eval")
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

        self.pub_image_red_lines_mask = rospy.Publisher(
            self.name_pub_image_red_lines_mask_topic,
            CompressedImage,
            queue_size=1,
        )
        
        self.pub_image_red_xsec_eval = rospy.Publisher(
            self.name_pub_image_xsec_eval_topic,
            CompressedImage,
            queue_size=1,
        )
        
        self.pub_bool_xsec_flag = rospy.Publisher(
            self.name_pub_bool_xsec_flag,
            Bool,
            queue_size=1,
        )

    def validate_mask_shapes(
        self,
        input_image: NDArray[np.uint8],
        red_lines_mask: NDArray[np.bool_],
    ) -> None:
        """
        Validate the shapes of the red line masks.

        Args:
            input_image (NDArray[np.uint8]): 3-channels input image
            red_lines_mask (NDArray[np.bool_]): 1-channel binary mask of the red lines
        """
        assert (
            len(red_lines_mask.shape) == 2
            and red_lines_mask.shape == input_image.shape[:2]
            and red_lines_mask.dtype == bool
        ), (
            "The red lines mask should be a 2D numpy binary array"
            f" with the same height and width {input_image.shape[:2]}"
            f" as the input image, got shape={red_lines_mask.shape}"
            f" and dtype={red_lines_mask.dtype} instead."
        )

    def publish_line_masks(
        self,
        image: NDArray[np.uint8], #np.red_lines_mask: NDArray[np.bool_],
        input_msg_stamp: rospy.Time,
    ) -> None:
        """
        Publish the red line masks as compressed images,
        with the same timestamp as the input image to be able to match the
        masks with the input image for the evaluation.

        Args:
            red_lines_mask (NDArray[np.bool_]): 1-channel binary mask of the red lines
            input_msg_stamp (rospy.Time): ROS timestamp of the input image
        """

        if CV_BRIDGE_AVAILABLE:
            red_lines_mask_msg = self.cv_bridge.cv2_to_compressed_imgmsg(
                255 * image.astype(np.uint8), dst_format="png"
            )
        else:
            red_lines_mask_msg = CompressedImage()
            red_lines_mask_msg.format = "png"
            ext_format = '.' + red_lines_mask_msg.format
            red_lines_mask_msg.data = np.array(
                cv2.imencode(ext_format, 255 * image.astype(np.uint8))[1]
            ).tobytes()
        red_lines_mask_msg.header.stamp = input_msg_stamp

        self.pub_image_red_lines_mask.publish(red_lines_mask_msg)
        
    def publish_xsec_eval(
        self,
        image: NDArray[np.uint8], #np.red_lines_mask: NDArray[np.bool_],
        input_msg_stamp: rospy.Time,
    ) -> None:
        """

        Args:
            evalutation (NDArray[np.bool_])
        """

        if CV_BRIDGE_AVAILABLE:
            xsec_eval_msg = self.cv_bridge.cv2_to_compressed_imgmsg(
                255 * image.astype(np.uint8), dst_format="png"
            )
        else:
            xsec_eval_msg = CompressedImage()
            xsec_eval_msg.format = "png"
            ext_format = '.' + xsec_eval_msg.format
            xsec_eval_msg.data = np.array(
                cv2.imencode(ext_format, 255 * image.astype(np.uint8))[1]
            ).tobytes()
        xsec_eval_msg.header.stamp = input_msg_stamp

        self.pub_image_red_xsec_eval.publish(xsec_eval_msg)
        
    def publish_xsec_flag(
        self,
        flag: np.bool
    ) -> None:
        self.pub_bool_xsec_flag.publish(flag)

    def xsec_detection_callback(self, msg: CompressedImage) -> None:
        """
        Callback for the input image subscriber.
        Detect the red lines in the input image,
        and publish the masks of the lines.

        Args:
            msg (CompressedImage): input image message
        """

        self.cnt += 1
        self.received_first_input_msg_image = True

        #sync callback
        if self.cnt % self.callback_rate == 0:
            self.cnt = 0
            
            input_msg_stamp = msg.header.stamp
            input_bgr_image = cv2.imdecode(
                np.ndarray(shape=(1, len(msg.data)), dtype=np.uint8, buffer=msg.data),
                cv2.IMREAD_UNCHANGED,
            )

            lines_mask, lines, projected_lines, cropped_image = line_detection(input_bgr_image)
            #self.validate_mask_shapes(input_bgr_image, lines_mask.astype(np.bool_))
            four_way_x_sec = XSecTile()
            score, eval, x_sec_flag = evaluate(four_way_x_sec, projected_lines, self.evaluate)
            self.publish_xsec_flag(x_sec_flag)
            if x_sec_flag:
                rospy.loginfo(f"image timestampe: {input_msg_stamp.to_sec()}")
                rospy.loginfo(f"score: {score}")

            
            if x_sec_flag:
                self.cnt = 0
            
            if self.evaluate: 
                #plot lines on image
                cropped_image_w_lines = plot_lines(cropped_image, lines, (0,0,255))
                #publish
                self.publish_line_masks(cropped_image_w_lines, input_msg_stamp)
                self.publish_xsec_eval(eval, input_msg_stamp)
        
        
            


if __name__ == "__main__":
    rospy.init_node("x_sec_detection", anonymous=True)

    xsec_detection = XsecDetection()
    xsec_detection.print_info()

    rospy.spin()
