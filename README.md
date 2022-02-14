# OMTP

This repository is part of the Object Manipulation and Task Planning course, from the 8th semester, Msc. in Robotics 2022. The repository contains the solution to the assignments given in the course. Each lecture has their own section in the README. In each lecture section the solution to the assignment in the specific lecture is detailed further. Installation of necessary packages, usage of the code and error handling is elaborated on in the lecture sections.

## Lecture 1

This Lecture took place 03/02-22 and was the lecture 1/12. The assignment entailed:

1.	Do two URDF tutorials
2.	Do one XACRO tutorial
3.	Inspect the OMTP factory world
4.	Rebuild the OMTP factory world
5.	Write documentation

### Installation

A zip file is provided in the lecture which contains all the necessary packages and additional installation is not required. The zip file should be unzipped in src in catkin workspace.

### Usage

The tutorials can be found at:

1.	http://wiki.ros.org/urdf/Tutorials/Building%20a%20Visual%20Robot%20Model%20with%20URDF%20from%20Scratch
2.	http://wiki.ros.org/urdf/Tutorials/Building%20a%20Movable%20Robot%20Model%20with%20URDF
3.	http://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File

The OMTP factory world can be inspected in the ``` .../omtp course files lecture 1 2022/ ``` folder in this repository which contains the meshes, XACRO files and more. 
Rebuilding the OMTP factory is accomplished by editing the XACRO file of the factory. The XACRO file is found at ``` .../omtp course files lecture 1 2022/omtp_support/urdf/omtp_factory.xacro. ``` The second robot arm is added similar to first one, the bins and ballet are added as modules in the XACRO file. To view the complete factory in RVIZ download this repository and unzip it into src in the catkin workspace and then type the following commands into terminal:
```
cd catkin_ws/
source devel/setup.bash
roscd omtp_support/urdf/
roslaunch omtp_support visualize_omtp_factory.launch
```

Afterwards the complete factory should be displayed in RVIZ.
