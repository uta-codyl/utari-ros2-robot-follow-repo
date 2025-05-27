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




## ROS Params
This ROS2 repository does not include any parameter files that would predefine parameter values, but the nodes do make use of checking for parameters and using default value if those parameters do not already exist. The launch files include do modify parameters used by depended packages such as the Clearpath simulator.  
Listed below are all parameters modified from dependent packages and parameters that can be modified for this repository.  
- File robot_follow_pkg/launch/nav_first_husky.launch.yaml
  - launch clearpath_gz robot_spawn.launch.py
    - setup_path
      - This is critial to the multi robot simulation
      - Each spawned Clearpath robot must be defined in a setup config folder
      - Each robot must have a specific unique setup folder that defines a unique name and serial number
      - 2 setup config folders are included with this repository
-  File robot_follow_pkg/launch/nav_second_husky.launch.yaml
  - launch clearpath_gz robot_spawn.launch.py
    - x
      - This is an offset from the map origin to spawn the robot. 
      - This is required to avoid the second robot spawning directly on top ofthe first. 
    - setup_path
      - Note that this must use a different setup config folder than the first robot
      - 

## Project Startup Commands 


## ROS Topics 


## 




