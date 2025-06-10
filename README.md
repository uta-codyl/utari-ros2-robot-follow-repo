# utari-ros2-robot-follow-repo
This repository contains a ROS2 packages developed by the University of Texas at Arlington Research Institute (UTARI) designed to enable robot following using multiple methods including a proportional controller and waypoint navigaition.   
This project has been tested in simulation using:  
- Ubuntu 22.04
- ROS2 Humble


## Project Goal and Methodlogy 
This project enables two following modes for one robot to follow another: A propotional controller, and a waypoint generator for the NAV2 system.    
The core method to enable these following modes is have both robots using seperate transform frame trees in seperate names spaces such as:  
/robot1/tf  
/robot1/tf_static   
/robot2/tf  
/robot2/tf_static   

A node (republish_tf.py) in this project will take selected frames in the namespace tf topics and republish them in the global topic space. The bare minimium number of frames are republished in the global space. The following node will run in the global space and listen to only the global tf topic to enable velociy or waypoint signals to be generated.   
 
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

## Project Startup Commands 

```bash
[Start the Gazebo simulation, RVIZ, and first robot]
$ ros2 launch robot_follow_pkg nav_first_husky.launch.yaml
[new terminal]
[Spawn the second robot with navigation and RVIZ]
$ ros2 launch robot_follow_pkg nav_two_husky.launch.yaml
[new terminal]
[This will launch 4 nodes to republish and prefix tf links to the global space]
$ ros2 launch robot_follow_pkg husky_follow_sim.launch.yaml
[new terminal]
[This will start nav2 for the first robot so it can move autonomously]
$ ros2 launch robot_follow_pkg nav_husky.launch
[new terminal]
[This will start the robot following code]
ros2 run robot_follow_pkg follow.py 
[You can now move the first robot using RVIZ nav goals and watch the second robot follow]
```

## ROS Nodes
### republish_tf.py
This file listens to the tf topic inside a given namespace for key parent and child link pairs. When the specified parent and child combinations are found, they are republished to the global /tf topic. A prefix can be added to the links before republishing them in the global /tf topic.   
This node allows each robot to published limited important coordiates and transforms into the global space to share with other robotic systems. Each instance of this node will listen for one specific parent-child link pair in a specific namespace and republish the link pair into another specified topic. The output topic can be inside of another namespace, but the primary goal of the node is to republish in the global namespace.  
The input topic/namespace, parent frame, child frame, and output topic are all specified via ROS parameters. More detail is given in a below section. Here is a quick list of the parameters for this node.  
- republish_tf_child_frame_id (str)
- republish_tf_frame_id (str)
- republish_tf_child_prefix (str)
- republish_tf_prefix (str)
- republish_tf_input_topic (str)
- republish_tf_output_topic (str)

### follow.py
This node is responsible for commanding a robot to follow another specified robot. Two methods are available: a proportional controller and waypoint navigaition using the Nav2 stack. This node will listen to the tf topic in the same namespace as the node and location either the specified robot or a point just behind that robot depending on parameter configuration. 
The use of these two modes is controlled via the parameter "use_nav_goal". If it is set to true (default) the node will send Nav2 waypoint navigation goals to a point two meters behind the lead robot and facing the same direction as the lead robot. If the parameter is set to false, a simple proportional controller will be used to move the robot directly toward the lead robot until they are within 2 meters of eachother and then a zero velocity signal will be sent. 
The input and output topics, modes, and starting behavior of this node is configured via ROS2 parameters. These are described in detail later in this docuement. Here is a quick list of the parameters for this node.  
- prefix0 (str)
- prefix1 (str)
- output_vel_topic (str)
- output_goal_topic (bool)
- use_nav_goal (bool)
- start_enabled (bool)

## ROS Params
This ROS2 repository does include 1 parameter files that predefines parameter values, and the other nodes make use of checking for parameters and using default value if those parameters do not already exist. The launch files included do modify parameters used by depended packages such as the Clearpath simulator.  
Listed below are all parameters modified from dependent packages and parameters that can be modified for this repository.  
- File: robot_follow_pkg/launch/nav_first_husky.launch.yaml
  - launch: clearpath_gz robot_spawn.launch.py
    - setup_path
      - This is critial to the multi robot simulation
      - Each spawned Clearpath robot must be defined in a setup config folder
      - Each robot must have a specific unique setup folder that defines a unique name and serial number
      - 2 setup config folders are included with this repository
-  File: robot_follow_pkg/launch/nav_second_husky.launch.yaml
  - launch: clearpath_gz robot_spawn.launch.py
    - x
      - This is an offset from the map origin to spawn the robot. 
      - This is required to avoid the second robot spawning directly on top ofthe first. 
    - setup_path
      - Note that this must use a different setup config folder than the first robot
- File: robot_follow_pkg/launch/husky_follow_sim.launch.yaml
  - Node: republish_tf.py
    - republish_tf_child_frame_id
      - This is the name of a child frame that needs to be republished into the global tf topic
    - republish_tf_frame_id
      - This is the name of the parent frame that needs to be republished into the global tf topic
    - republish_tf_child_prefix
      - This is a string prefix to append to the child frame when it is republished into the global tf topic
    - republish_tf_prefix
      - This is a string prefix to append to the parent frame when it is republished into the global tf topic
    - republish_tf_input_topic
      - This is the topic name to listen for the tf links in before republishing them 
- File: robot_follow_pkg/robot_follow_pkg/follow.py
  - prefix0 
    - This parameter specifies the prefix for the lead robot
    - This is required for the node to find the location of the lead robot
  - prefix1 
    - This parameter specifies the prefix for the following robot
    - This is required for the node to find the location of the following robot
  - output_vel_topic 
    - This specifies the topic to send velocity commands to the following robot
  - output_goal_topic 
    - This specifies the topic to send navigation goals to the following robot
  - use_nav_goal 
    - This chooses the following mode of the node 
    - True: The node will send nav2 navigational goals to a point 2 meters behind the lead robot with the same heading
    - False: The node will use a proportional controller to send yaw and x velocity commands to the following robot to move directly toward the lead robot and stop when within 2 meters. 
  - start_enabled 
    - This topic activates and deactivates the following mode
    - if deactivated the node will idle until activated 


## ROS Topics 
Most of the topics in this package have reconfigurable names via ROS parameters. This section will list them by their default names 
### Node: republish_tf.py



### Node: follow.py

Default name: /a200_0001/cmd_vel  
Type: geometry_msgs Twist  
Controlled by parameter: output_vel_topic  
This topic must be configured so that cmd velocity signals can be sent to the following robot. If this is not done the proportional controller mode will not function.  
  
Default name: follow/enable  
Type: std_msgs Bool  
Controlled by parameter: None  
This topic activates and deactivates the following node. If a false value is sent the node will ideal until reactivated. If a true value is sent the following mode will activate and move the following robot.  

## 




