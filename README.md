# Maze Runner: Turtlebot Maze Solver

## Overview
The Maze Runner is an autonomous, wheeled robotic system based on Turtlebot3 that aims to navigate a simply connected maze without a prior map given a start and end point. The Maze Runner uses inputs from wheel odometers and a 2D LiDAR for localization and mapping, and uses a wall following algorithm that is adapted from this [repository](https://github.com/celisun/ROS_Turtlebot_Maze_Solving_task). This repository provides the software for the robot itself as well as a simulation file that offers a user interface for both deterministic and stochastic modes of operation to evaluate certain metrics of interest to support the enhancement of system performance and decision making. 


## Personnel
Hao Da (Kevin) Dong

Krithika Govindaraj

Praveen Kumar Menaka Sekar    

Sarjana Oradiambalam Sachidanandam


## Install Dependencies
This package requires Ubuntu 18.04 with Python ROS Melodic and Turtlebot3. Install Turtlebot3 dependencies using:
```
sudo apt-get install ros-melodic-joy ros-melodic-teleop-twist-joy ros-melodic-teleop-twist-keyboard ros-melodic-laser-proc ros-melodic-rgbd-launch ros-melodic-depthimage-to-laserscan ros-melodic-rosserial-arduino ros-melodic-rosserial-python ros-melodic-rosserial-server ros-melodic-rosserial-client ros-melodic-rosserial-msgs ros-melodic-amcl ros-melodic-map-server ros-melodic-move-base ros-melodic-urdf ros-melodic-xacro ros-melodic-compressed-image-transport ros-melodic-rqt-image-view ros-melodic-gmapping ros-melodic-navigation ros-melodic-interactive-markers
```

Next, install Turtlebot3 with:
```
sudo apt-get install ros-melodic-turtlebot3-*
```


## Build Instructions
This repository is a ROS package. Thus, ensure that your current directory is the src folder inside your catkin workspace, then execute the following (assuming [Catkin Command Line Tools](https://catkin-tools.readthedocs.io/en/latest/) is used):
```
git clone https://github.com/kdglider/mazerunner.git
catkin build
```


## Run Demonstration
To examine the example maze world files, navigate to the world folder and use the gazebo command. For example:
```
gazebo maze1.world
```

To launch the simulation, open two terminal windows. In the first window, run:
```
roscore
```
In the second window, run:
```
rosrun mazerunner launchSim.py
```
The start/end locations of each maze for the deterministic mode and the number of runs per maze for the stochastic mode can be changed in the launchSim.py file. After the file is run, enter either "d" or "s" at the prompt to run the simulation in either deterministic or stochastic mode respectively. Multiple Gazebo windows will open and close as the difference test cases are run. Metrics of interest will be printed to the terminal as they become available.