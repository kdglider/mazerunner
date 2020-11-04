# Maze Runner: Turtlebot Maze Solver
This repository is not yet complete

## Overview
This code is adapted from this [repository](https://github.com/celisun/ROS_Turtlebot_Maze_Solving_task).

Exemplary code **[Autonomous Robotics Lab](http://campusrover.org.s3-website-us-west-2.amazonaws.com)** 

@ Celi Sun  @ Nov, 2017  @ Brandeis University

To nagivate turtlebot solving a maze, we have: 

- main
- scan twist center control
- policy
- helper controller

The scan twist center control is receiving messages, processing message raw data, and publishing reaction messages once activated. It is also registered with a policy and an optional helper controller to help it make decisions of what action to take in response to the incoming message it receives, and therefore, eventually, navigates a turtlebot out of maze.

The repo here gives the exemplary code of solution and the default policy we provide is LeftOrRightHandRule, but you are more than welcomed to add your own policies (extends our Policy class). To add, put your policy script in the policy folder, and make the center control "know" it, and get it registered in the constructor to make it work.
 
Oct/2017

<img src="https://raw.githubusercontent.com/celisun/ROS_Turtlebot_Maze_Solving_task/master/pics/tb3-LABEL[1].png" width="180"> 


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
This repository is a ROS package. Ensure that your current directory is the src folder inside your catkin workspace, then execute the following (assuming [Catkin Command Line Tools](https://catkin-tools.readthedocs.io/en/latest/) is used):
```
git clone https://github.com/kdglider/mazerunner.git
catkin build
```


## Run Demonstration

