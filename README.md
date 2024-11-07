# x-sec-driving
ROS Package of crosssection driving

```mermaid
C4Context
title Cross Section Driving Package

Enterprise_Boundary(b1, "workspace") {

  System_Boundary(b2, "x-sec-driving_pkg") {
    System(SystemA, "x_sex_detection")
    System(SystemB, "x_sec_navigation")
  }

  System_Boundary(b3, "duckiebot_pkg", "boundary") {
    System(SystemC, "camera/compressed_img", "std_msgs.msg/compressed")
  }
}

Rel(SystemC, SystemA, "compressed img")
Rel(SystemA, SystemB, "red lane information")
````

```%% C4 Level 1: System Context Diagram
flowchart TB
  %% Users and external systems
  user(User) -->|Interacts| sys(System)
  ext(External System) -->|Uses| sys

  %% Main system
  sys(System) -->|Sends data to| db[(Database)]

  %% Labels and style
  classDef userClass fill:#f96,stroke:#333,stroke-width:2px;
  class user,ext userClass;

  classDef systemClass fill:#9cf,stroke:#333,stroke-width:2px;
  class sys systemClass;
```
```%% C4 Level 2: Container Diagram
flowchart TB
  user(User) --> web[Web Application]
  user --> mob[Mobile App]
  web -->|Reads/Writes| db[(Database)]
  mob --> api[API Server]
  api --> db

  %% Styles
  classDef app fill:#bbf,stroke:#333,stroke-width:2px;
  classDef dbStyle fill:#f99,stroke:#333,stroke-width:2px;
  class user userClass;
  class web,mob,api app;
  class db dbStyle;
```
