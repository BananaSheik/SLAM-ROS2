SLAM Robot using ROS 2 and Nav2 Stack
Overview
This project implements a SLAM (Simultaneous Localization and Mapping) robot using ROS 2 and the Nav2 stack. The robot is designed to autonomously navigate and build a map of its environment while accurately estimating its position.

Contents
src/: Contains the main source code for the SLAM robot, including launch files and node implementations.

slam_robot/: Main package for the SLAM robot functionalities.
launch/: Launch files to start the robot and its navigation stack.
params/: Configuration files for various parameters (e.g., robot model, navigation settings).
config/: Configuration files for the robot's parameters, including:

robot_description.yaml: Robot model and joint configurations.
nav2_params.yaml: Parameters for the Nav2 stack, including costmaps, planners, and controllers.
map.yaml: Mapping parameters to assist in the navigation process.
maps/: Contains pre-generated maps of the environment for testing and demonstration purposes.

scripts/: Utility scripts for various tasks, such as data collection, visualization, and simulation.

launch/: Contains launch files for starting various components of the SLAM robot, including:

README.md: This README file, providing an overview of the project and its contents.

LICENSE: License information for the project.
