# Final Project - Autonomous Maze Navigation with ROS2

## Overview
Two-phase autonomous maze exploration and navigation using TurtleBot3 Burger in Gazebo.
- Phase 1: Autonomous wall-following exploration with SLAM and ArUco marker detection
- Phase 2: Goal-directed navigation to specified ArUco markers using Nav2

## Dependencies
```bash
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
sudo apt install ros-humble-slam-toolbox
sudo apt install ros-humble-nav2-smac-planner
sudo apt install ros-humble-nav2-regulated-pure-pursuit-controller
sudo apt install ros-humble-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

## Build
```bash
cd ~/final-project-Ihtesham-star-main
colcon build --packages-select final_project
source install/setup.bash
```

## Phase 1 - Mapping and Exploration
```bash
ros2 launch final_project maze_exploration.launch.xml
```
Runs until 92%+ coverage. Saves `explored_maze.pgm`, `explored_maze.yaml`, `aruco_markers.yaml` to `maps/`.

## Phase 2 - Navigate to Marker
```bash
ros2 launch final_project aruco_navigation.launch.xml marker_id:=4
```
Replace `4` with any detected marker ID from Phase 1.

## Nodes
- `maze_explorer_node` - Wall-following exploration + ArUco detection
- `map_saver_node` - Auto-saves map at coverage threshold
- `aruco_goal_node` - Loads markers and navigates to specified ID

## Notes
- Robot starts at (0.25, 0.5) facing positive x-axis
- AMCL localization with 5000 particles for robust pose tracking
- SMAC Hybrid-A* global planner for maze navigation# final_project

## Project Video
https://drive.google.com/file/d/1J7b024wxlBJ-5QMMCALGvoARaM6FbOz9/view?usp=sharing