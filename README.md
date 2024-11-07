# x-sec-driving
ROS Package of crosssection driving

```mermaid
C4Context
title Cross Section Driving Package

Enterprise_Boundary(b1, "workspace", "includes all packages") {

  Package_Boundary(b2, "x-sec-driving_pkg", "Package") {
    Node(SystemA, "x_sex_detection", "Node")
    System(SystemB, "x_sec_navigation", "Node")
  }

  System_Boundary(b3, "duckiebot_pkg", "boundary") {
    System(SystemC, "camera/compressed_img", "std_msgs.msg/compressed")
  }
}

Rel(SystemC, SystemA, "compressed img")
Rel(SystemA, SystemB, "red lane information")
```
