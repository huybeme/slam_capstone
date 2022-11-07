#Graduate Capstone Project

This capstone project implements autonmous SLAM using a Turtlebot 3 model Burger, or TB3. The TB3 is a ground mobile robot developed by ROBOTIS and comes with several sensors standard. For this proejct, we will only be interested in the lidar sensor and will install the TPS-10 temperature sensor. This project programs the TB3 to use ROBOTIS' turtlebot3_cartographer and turtlebot3_navigation2 packages to use SLAM techniques while tracking the temperature surrounding the robot. Once a completed map is generated, the program will provide coordinates to the location with the highest temperature detected and navigate towards it.

##Requirements
laptop and raspberry pi installed with the following
1. Ubuntu 20.04
2. ROS2 Foxy - https://docs.ros.org/en/foxy/Installation.html
3. Turtlebot 3 standard packages - https://github.com/ROBOTIS-GIT/turtlebot3/tree/foxy-devel

note: turtlebot3_bringup package was modified to read and publish the tmp-10 temperature sensor

##Installation and run
1. Once ROS2 Foxy and Turtlebot3 standard packages are installed, clone repository.
2. In terminal, build package by: colcon build --symlink-install
3. Launch project by: ros2 launch capstone_bringup capstone_slam.launch.py

Bugs
1. turtlebot3_bringup package spawns odometry frame at a seeminly random location. This random location can be large enough that the turtlebot3_cartographer package will crash.
2. robot index in the robot_world node sometimes gets stuck at one location