# utari-ros2-robot-follow-repo

## Project folders 

### robot_follow_pkg  
Main ROS2 package including code from publishing and prefixing TFs, and mulitple methods of robot following.  

### clearpath_config_folders 
These are the Clearpath setup folder used by the ROS2 Gazebo Clearpath simulator as described here:  
https://docs.clearpathrobotics.com/docs/ros/installation/offboard_pc/  
More information will be provided in the Initial setup section below.  

## Project Initial setup 
Before starting the robot_follow_pkg the Clearpath simuator must be configured. These steps are descriped here:  
https://docs.clearpathrobotics.com/docs/ros/installation/offboard_pc/  
https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/overview  
More specifically the above instructions include: 
- The setup folder(s) must be configured
- The different robot.yaml files must use a differenet namespace for a multiple robot simulation
  - For mutli robot simulation 2 robot.yaml files are used in 2 different directories
  - These directories are included in this repository
  - As per the instructions above, these are commonly copied to the home folder
- The robot.yaml file(s) must have the ip address and hostname updated.
  - Unstable and unpredictable performance results from leaving these blank or incorrect 
- The setup.bash(s) must be generated via the clearpath command in the offboard_pc instructions
- 




## ROS Params


## Project Startup Commands 


## ROS Topics 


## 




