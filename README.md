## System requirement:
- Ubuntu 24.04
- ROS2 Jazzy
- Gazebo Harmonic

Refer to https://www.youtube.com/watch?v=OWeLUSzxMsw&amp;list=PLunhqkrRNRhYAffV8JDiFOatQXuU-NnxT

## Troubleshoot
ERROR
```
[gz-2] [Err] [SystemLoader.cc:92] Failed to load system plugin [gz_ros2_control-system] : Could not find shared library.
```
FIX
```
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:/opt/ros/${ROS_DISTRO}/lib
```
