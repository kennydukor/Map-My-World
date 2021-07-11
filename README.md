# Where AM I
Udacity Robotics Software Engineer Nanodegree Program [Project 3]

![Overview](screenshots/where-am-i.gif)  

## Overview  
In this project you'll utilize ROS AMCL package to accurately localize a mobile robot inside a map in the Gazebo simulation environments. Here are the steps to learn several aspects of robotic software engineering with a focus on ROS:  
* Create a ROS package that launches a custom robot model in a custom Gazebo world  
* Utilize the ROS AMCL package and the Tele-Operation / Navigation Stack to localize the robot  
* Explore, add, and tune specific parameters corresponding to each package to achieve the best possible localization results  
## Prerequisites/Dependencies  
* Gazebo >= 7.0  
* ROS Kinetic  
* ROS navigation package  
```
sudo apt-get install ros-kinetic-navigation
```
* ROS map_server package  
```
sudo apt-get install ros-kinetic-map-server
```
* ROS move_base package  
```
sudo apt-get install ros-kinetic-move-base
```
* ROS amcl package  
```
sudo apt-get install ros-kinetic-amcl
```
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
## Setup Instructions (abbreviated)  
1. Meet the `Prerequisites/Dependencies`  
2. Open Ubuntu Bash and clone the project repository  
3. On the command line execute  
```bash
sudo apt-get update && sudo apt-get upgrade -y
```
4. Build and run your code.  

## Run the project  
* Clone this repository into your catkin_ws/src dir
```
git clone https://github.com/kennydukor/Where-Am-I.git
```
* Open the repository and make  
```
cd <PATH>/catkin_ws/
catkin_make
```
* Launch my_robot in Gazebo to load both the world and plugins  
```
roslaunch kene_robot world.launch
```  
* Launch amcl node  
```
roslaunch kene_robot amcl.launch
```  
* Testing  
You have two options to control your robot while it localize itself here:  
  * Send navigation goal via RViz  
  * Send move command via teleop package.  
Navigate your robot, observe its performance and tune your parameters for AMCL.  

**Option 1: Send `2D Navigation Goal`**  
Your first option would be sending a `2D Nav Goal` from RViz. The `move_base` will try to navigate your robot based on the localization. Based on the new observation and the odometry, the robot to further perform the localization.  
Click the `2D Nav Goal` button in the toolbar, then click and drag on the map to send the goal to the robot. It will start moving and localize itself in the process. If you would like to give `amcl` node a nudge, you could give the robot an initial position estimate on the map using `2D Pose Estimate`.  
**Option 2: Use `teleop` Node**  
You could also use teleop node to control your robot and observe it localize itself in the environment.  
Open another terminal and launch the `teleop` script:  
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
You could control your robot by keyboard commands now.  
