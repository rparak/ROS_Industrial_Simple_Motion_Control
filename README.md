# ROS Industrial - Simple Motion Control

## Requirements:

**Software:**

```bash
Robot Operating System (ROS) - Tested on Melodic distribution, Ubuntu 18.04
```

ROS: https://www.ros.org

Melodic Install: http://wiki.ros.org/melodic/Installation/Ubuntu

Ubuntu 18.04: https://releases.ubuntu.com/18.04/

## Project Description:

Simple demonstration of robot motion control and trajectory planning via the ROS system using several simulation tools (RVIZ, gazebo, etc.)

Controlling the movement of multiple industrial robots (ABB, Fanuc, Universal Robots, etc.) using the Python programming language (catkin, rospy, launch file, etc.)

Main challenges of project implementation:
- text

The project was created to improve the [VRM (Programming for Robots and Manipulators)](https://github.com/rparak/Programming-for-robots-and-manipulators-VRM) university course.

The project was realized at Institute of Automation and Computer Science, Brno University of Technology, Faculty of Mechanical Engineering (NETME Centre - Cybernetics and Robotics Division)

<p align="center">
  <img src="https://github.com/rparak/ROS_Industrial_Simple_Motion_Control/blob/main/images/1_c.png" width="250" height="250">
  <img src="https://github.com/rparak/ROS_Industrial_Simple_Motion_Control/blob/main/images/2_c.png" width="250" height="250">
  <img src="https://github.com/rparak/ROS_Industrial_Simple_Motion_Control/blob/main/images/3_c.png" width="250" height="250">
</p>

## Project Hierarchy:

**Repositary [/ABB-RobotStudo-Tutorial-SortingMachine/]:**

```bash
[ Main Program (.rspag)                ] /Final/
[ Project Template (without a robot)   ] /Template/ 
[ Example of the resulting application ] /Exe_file/
[ Rapid codes (.mod) - Right/Left Arm  ] /Rapid/
[ Scene parts, gripper, etc.           ] /Project_Materials/
```

## Application:

**ABB IRB 1200:**

```bash

$ roslaunch abb_irb1200_5_90_moveit_config demo.launch
$ roslaunch robot_ctrl demo.launch
$ rosrun robot_ctrl test.py

```

<p align="center">
  <img src="https://github.com/rparak/ROS_Industrial_Simple_Motion_Control/blob/main/images/1.png" width="800" height="450">
</p>

**Universal Robots UR5:**

```bash

$ roslaunch ur5_moveit_config demo.launch
$ roslaunch robot_ctrl demo.launch
$ rosrun robot_ctrl test.py

```

<p align="center">
  <img src="https://github.com/rparak/ROS_Industrial_Simple_Motion_Control/blob/main/images/3.png" width="800" height="450">
</p>

**Fanuc CR-7iA:**

```bash

$ roslaunch fanuc_cr7ia_moveit_config demo.launch
$ roslaunch robot_ctrl demo.launch
$ rosrun robot_ctrl test.py

```

<p align="center">
  <img src="https://github.com/rparak/ROS_Industrial_Simple_Motion_Control/blob/main/images/2.png" width="800" height="450">
</p>

## Useful commands:

**Setup a Catkin Workspace:**

```bash
Create Catkin Workspace

$ source /opt/ros/melodic/setup.bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
$ source devel/setup.bash
$ echo $ROS_PACKAGE_PATH

Reset Catkin Workspace

$ rm -r build devel
$ rosclean purge

Reload

$ catkin_make

Create Package

$ catkin_create_pkg robot_ctrl rospy
$ cd src/robot_ctrl

Create Launch File

$ mkdir launch
$ cd launch
$ touch robot.launch
$ roslaunch robot_ctrl robot.launch

```

ROS Tutorials: http://wiki.ros.org/ROS/Tutorials

## Contact Info:
Roman.Parak@outlook.com

## License
[MIT](https://choosealicense.com/licenses/mit/)
