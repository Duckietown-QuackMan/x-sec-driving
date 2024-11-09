# x-sec-driving
ROS Package of crosssection driving


# ROS Architecture
```mermaid
C4Context
title Cross Section Driving Package

Enterprise_Boundary(b1, "workspace", "includes all packages") {

  System_Boundary(b2, "x-sec-driving_pkg", "Package") {
    System(SystemA,"x_sex_detection", "Node")
    System(SystemB, "x_sec_nav", "Node")
  }

  System_Boundary(b3, "state_controller_pkg", "Package") {
    System(SystemC, "state_machine", "Node")
  }

  System_Boundary(b4, "duckie_pkg", "Package") {
    System(SystemD, "camera_node", "Node")
  }
}

Rel(SystemD, SystemA, "/gimpy/camera_node/image/compressed" ,"sensor_msg/CompressedImage")
Rel(SystemA, SystemC, "/gimpy/xsec_detection/xsec_flag/bool", "std_msgs/Bool")
```

# Cross section tile
<img src="media/4way_4_done.png]" width="400" />
Cross section tile

# Cross section detection 
<img src="media/x_sec_detection.png" width="400" />
Detected Line Segments
<img src="media/x_sec_detection_eval.png" width="400" />
Evaluation of Xsec Detection


