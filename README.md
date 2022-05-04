# OMTP

This repository is part of the Object Manipulation and Task Planning course, from the 8th semester, Msc. in Robotics 2022.
The repository contains the solutions to the assignments given in the course. Each lecture has their own section in the README and each exercise has a tag like lec<number>_submission. In each lecture section the solution to the assignment in the specific lecture is detailed further. Installation of necessary packages, usage of the code and error handling is elaborated on in the lecture sections.
##Prerequisites:
### ROS and Gazebo installation
The project has been tested using the following versions.
* Ubuntu 20.04 and [ROS Noetic](https://wiki.ros.org/noetic/Installation) & [Gazebo](http://gazebosim.org/tutorials/?tut=ros_wrapper_versions) 11 

## Lecture 1- Building a Robot Simulation Environment in ROS
####Before getting started, the following tutorials about URDF en Xacro files can be used to learn the basics:

* [Building a Visual Robot Model with URDF from Scratch](http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch) 
* [Building a Movable Robot Model with URDF](http://wiki.ros.org/urdf/Tutorials/Building%20a%20Movable%20Robot%20Model%20with%20URDF)
* [Using Xacro to Clean Up a URDF File](http://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File)

###To get started:
1) Create a catkin workspace.(http://wiki.ros.org/catkin/Tutorials/create_a_workspace)
2) Clone the repository:`git clone https://github.com/ROB8-very-N0ICE/OMTP.git` in `$ cd ~/catkin_ws/src`
3) `$ catkin_make` in `$ cd ~/catkin_ws`
4)  The following packages will be needed: 
  - http://wiki.ros.org/ros_controllers
  - http://wiki.ros.org/gazebo_ros_pkgs
  - http://wiki.ros.org/gazebo_plugins 
  - http://wiki.ros.org/joint_state_publisher
  - http://wiki.ros.org/joint_state_publisher_gui
  - http://wiki.ros.org/robot_state_publisher 
5) `$ source devel/setup.bash`
6) Visualize factory in Rviz:`$ roslaunch omtp_support visualize_omtp_factory.launch`

###Inspecting the factory
1) We transform the xacro factory file into a urdf file: `$ xacro omtp_factory.xacro > omtp.urdf`
2) Check the syntax of the urdf file: `$ check_urdf omtp.urdf`
3) Check the link graph by generating a pdf using: `$ urdf_to_graphiz`


###Rebuild the OMTP factory world
It should include:
* Two Franka robots, one bin per robot can be found here: [file](/franka_description)
* AAU smart lab Festo modules can be found here: [file](/aau_lab_ros_models)
* Additional models/objects( Conveyors, boxes, pallets, shelves etc.) models can be found in te following links:
  * https://app.ignitionrobotics.org/fuel/models
  * https://github.com/osrf/gazebo_models
  * http://data.nvision2.eecs.yorku.ca/3DGEMS/
### The OMTP factory xacro file which is modified is:
  `omtp_support/urdf/omtp_factory.xacro`

###1) Adding the robots to the factory
Robot descriptions (URDF/XACRO) can be found on [ROS-industrial](http://wiki.ros.org/Industrial/supported_hardware) 

The two robot arms added were the panda manipulators, which can be found from line 41 to 519 for the first panda arm, and the second panda arm being from line 520 to 992, where the eight links are described with a cylinder length, radius, and origin for the two end points. Along with the links, the joints are also described. They contain information about the individual joints rotation position and which links they are connected to, as well as their velocity limit and damping. The links and joints for the end effectors were also added for the two panda arms. Two for each arm.

###3) Adding AAU Smart Lab modules
Seven modules were  added and connected to one another in line 1057 to 1225, describing the specific model, their origin point, and which module they are connected to. The modules consists of five modules with a conveyor belt going straight forward, one in a T shape, and the last one containing a closed off workstation.

###4) Adding additional objects

 Additionally, a wooden pellet and a nurse were added in fron of the factory at line 1236 to line 1282, containing an origin, model and inertia. A screenshot titled lecture1exercise can be found in the folder named "pictures".

![Image](/images/factory.png)

## Lecture 2
Task list for lecture 2:
1. Create a MoveIt configuration package of your OMTP environment
2. Test moveit_config package with MoveIt Commander command line tool
3. Create a custom OMTP Gazebo launch file and .world
4. Create a pick and place pipeline in Python

In lecture 2 we created a ROS package using MoveIt setup assistant. By running:
```
rosrun moveit_setup_assistant moveit_setup_assistant 
```
<p class="aligncenter">
    <img src="Images/moveit.png" width=80% height=80% align=center alt="MoveIt Setup Assistant">
</p>


## Lecture 3
## Lecture 4
## Lecture 5
## Lecture 7
## Lecture 8
