# ROS Industrial (Introduction) - Simple Motion Control

## Requirements:

**Software:**

```bash
Robot Operating System (ROS) - Tested on Melodic distribution, Ubuntu 18.04
```

| Software/Package      | Link                                                                                  |
| --------------------- | ------------------------------------------------------------------------------------- |
| ROS                   | https://www.ros.org                                                                   |
| Melodic Install       | http://wiki.ros.org/melodic/Installation/Ubuntu                                       |
| Ubuntu 18.04          | https://releases.ubuntu.com/18.04/                                                    |

## Project Description:

Simple demonstration of robot motion control and trajectory planning via the ROS system using simulation tool RVIZ (3D visualization tool for ROS). The main idea of the project is to control the motion and other dependencies of multiple industrial robots (ABB, Fanuc, Universal Robots, etc.) using the Python programming language and also the way to work with catkin workspace, ros dependencies (publisher, subscriber, parameters, ...), launch files, etc.

Main challenges of project implementation:
- RVIZ simulation tool
- robot motion control (joint, cartesian - different types of control)
- rosparam, rostopic, rosservice, etc.
- collision avoidance

The project was created to improve the [VRM (Programming for Robots and Manipulators)](https://github.com/rparak/Programming-for-robots-and-manipulators-VRM) university course.

The project was realized at the Institute of Automation and Computer Science, Brno University of Technology, Faculty of Mechanical Engineering (NETME Centre - Cybernetics and Robotics Division).

<p align="center">
  <img src="https://github.com/rparak/ROS_Industrial_Simple_Motion_Control/blob/main/images/1_c.png" width="250" height="250">
  <img src="https://github.com/rparak/ROS_Industrial_Simple_Motion_Control/blob/main/images/2_c.png" width="250" height="250">
  <img src="https://github.com/rparak/ROS_Industrial_Simple_Motion_Control/blob/main/images/3_c.png" width="250" height="250">
</p>

## Project Hierarchy:

**Repositary [/ROS_Industrial_Simple_Motion_Control/robot_ctrl/]:**

```bash
[ Configuration file of the robot parameters      ] /config/
[ Main Launch File (demo)                         ] /launch/ 
[ Read joint/cartesian data                       ] /src/data_collection/
[ Service (reset parameters, add object to env.)  ] /src/service/
[ Main control script                             ] /src/main/
```

## Application:

**ABB IRB 1200:**

**Install:**

```bash

Note 1: replace '$ROS_DISTRO' with the ROS version (e.g. melodic, see 'branches/tags' in the github repository)
Note 2: replace 'workspace_name' with used workspace (e.g. catkin_ws)

$ cd $HOME/workspace_name/src
$ git clone -b $ROS_DISTRO-devel https://github.com/ros-industrial/abb_experimental.git
$ cd ..
$ rosdep update
$ rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src

```

Link: https://github.com/ros-industrial/abb_experimental

**Robot Control:**

```bash

Terminal 1:
$ roslaunch abb_irb1200_5_90_moveit_config demo.launch

Terminal 2:
$ roslaunch robot_ctrl demo.launch

Terminal 3:
$ rosrun robot_ctrl test.py

```

<p align="center">
  <img src="https://github.com/rparak/ROS_Industrial_Simple_Motion_Control/blob/main/images/1.png" width="800" height="450">
</p>

**Universal Robots UR5:**

**Install:**

```bash

Note 1: replace '$ROS_DISTRO' with the ROS version (e.g. melodic, see 'branches/tags' in the github repository)
Note 2: replace 'workspace_name' with used workspace (e.g. catkin_ws)

$ cd $HOME/workspace_name/src
$ git clone -b $ROS_DISTRO-devel https://github.com/ros-industrial/universal_robot.git
$ cd ..
$ rosdep update
$ rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src

```

Link: https://github.com/ros-industrial/universal_robot

**Robot Control:**

```bash

Terminal 1:
$ roslaunch ur5_moveit_config demo.launch

Terminal 2:
$ roslaunch robot_ctrl demo.launch

Terminal 3:
$ rosrun robot_ctrl test.py

```

<p align="center">
  <img src="https://github.com/rparak/ROS_Industrial_Simple_Motion_Control/blob/main/images/3.png" width="800" height="450">
</p>

**Fanuc CR-7iA:**

```bash

Note 1: replace '$ROS_DISTRO' with the ROS version (e.g. melodic, see 'branches/tags' in the github repository)
Note 2: replace 'workspace_name' with used workspace (e.g. catkin_ws)

$ cd $HOME/workspace_name/src
$ git clone -b $ROS_DISTRO-devel https://github.com/ros-industrial/fanuc.git
$ cd ..
$ rosdep update
$ rosdep install --rosdistro $ROS_DISTRO --ignore-src --from-paths src

```

Link: https://github.com/ros-industrial/fanuc

**Robot Control:**

```bash

Terminal 1:
$ roslaunch fanuc_cr7ia_moveit_config demo.launch

Terminal 2:
$ roslaunch robot_ctrl demo.launch

Terminal 3:
$ rosrun robot_ctrl test.py

```

<p align="center">
  <img src="https://github.com/rparak/ROS_Industrial_Simple_Motion_Control/blob/main/images/2.png" width="800" height="450">
</p>

## Useful Commands:

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
