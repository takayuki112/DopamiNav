
## Hello, Welcome to DopamiNav!

In this repository you will find 3 ROS packages:
1. turtlebot3 - This folder contains all the packages that come with the turtlebot3 robot. It is used for the simulation of the robot, and also a mappping package (gmapping or hector slam)

2. simulation - This folder contains the simulation of the turtlebot3 robot in a custom environment. This environment is a maze that was created with the intention of training and testing an autonomous robot. 
It contains the following launch files:
    1. dopamaze_move_base.launch - This file launches the robot with Gazebo and Rviz, with a navigation stack that uses the move_base package, and a slam.
    2. dopamaze_onlyglobal.launch - This file launches the robot with Gazebo and Rviz, with a navigation stack that doesn't have a local motion planner, only a global one. The purpose of this file is to train and test an RL based local motion planner (still being worked upon)

3. rl_nav - This folder contains an OpenAI gym wrapper for a gazebo based environment to train an RL based local motion planner. This segment of the project is still being worked upon and is not yet complete.

## Setup Instructions
1. Put all the above folders in the src folder of a catkin workspace.
2. Run 'catkin_make' or `catkin build' in the root of the catkin workspace.
3. Source the setup.bash file in the devel folder of the catkin workspace.
4. Run the dopamaze_move_base.launch file in the simulation package to launch the robot in the maze environment.
5. Set a 2d nav goal in Rviz to see the robot navigate to the goal.
