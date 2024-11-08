# x-sec-driving
ROS Package of crosssection driving

```mermaid
C4Context
title Cross Section Driving Package

Enterprise_Boundary(b1, "workspace", "includes all packages") {

  System_Boundary(b2, "x-sec-driving_pkg", "Package") {
    System(SystemA,"x_sex_detection", "Node")
    System(SystemB, "x_sec_navigation", "Node")
  }

  System_Boundary(b3, "duckie_pkg", "Package") {
    System(SystemC, "camera_node", "Node")
  }
}

Rel(SystemC, SystemA, "topic/../image/compressed" ,"sensor_msgs.msg/CompressedImage")
Rel(SystemA, SystemB, "topic/../mask_red_lines/compressed", "sensor_msgs.msg/CompressedImage")
```
