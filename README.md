### Install Dependencies:
Install https://www.sick.com/de/en/sick-vision-suite-28linux29/p/p660738
*Tested with ROS2 humble on Ubuntu 22.04

### Run node
```
ros2 run sick_vision_ros2 vision_node
```
### Run node with optional params
```
ros2 run sick_vision_ros2 vision_node --ros-args -p fps:=30.0 -p serial_no:="_002117XX"
```