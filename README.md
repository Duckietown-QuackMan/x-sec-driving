# x-sec-driving
ROS Package of crosssection driving


# ROS Architecture
```mermaid
C4Context
title Cross Section Driving Package

Enterprise_Boundary(b1, "workspace", "includes all packages") {

  System_Boundary(b2, "x-sec-driving_pkg", "Package") {
    System(xsec_det,"x_sex_detection", "Node")
    System(xsec_nav, "x_sec_nav", "Node")
  }

  System_Boundary(b3, "state_controller_pkg", "Package") {
    System(sm, "state_machine", "Node")
  }

  System_Boundary(b4, "duckie_pkg", "Package") {
    System(camera, "camera_node", "Node")
    System(motor_driver, "wheels_driver_node", "Node")
    System(veltopose, "velocity_to_pose_node", "Node")
  }

  System_Boundary(b5, "Debugging")
}

Rel(camera, xsec_det, "/gimpy/camera_node/image/compressed" ,"sensor_msg/CompressedImage")
Rel(xsec_det, sm, "/gimpy/xsec_detection/xsec_flag/bool", "std_msgs/Bool")


Rel(xsec_nav, sm, "/quackman/x_sec_navigating", "std_msgs/Bool")
Rel(xsec_nav, motor_driver, "/gimpy/wheels_driver_node/wheels_cmd", "duckietown.msg/WheelsCmdStamped")
Rel(veltopose, xsec_nav, "/gimpy/velocity_to_pose_node/pose", "duckietown.msg/Pose2DStamped")
Rel(sm, xsec_nav, "/quackman/x_sec_go", "std_msgs/Bool")

Rel(xsec_det, b5, "/gimpy/xsec_detection/mask_red_lines/compressed","sensor_msg/CompressedImage")
Rel(xsec_det, b5, "/gimpy/xsec_detection/xsec_eval/compressed","sensor_msg/CompressedImage")

```

# Cross section tile
<img src="media/4way_4_done.png" width="400" />
Cross section tile

# Cross section detection 
<img src="media/x_sec_detection.png" width="400" />
Detected Line Segments
<img src="media/x_sec_detection_eval.png" width="400" />
Evaluation of Xsec Detection

# Test Setup
<img src="media/img_1656_720.jpg" width="400" />
<img src="media/img_1658_720.jpg" width="400" />

