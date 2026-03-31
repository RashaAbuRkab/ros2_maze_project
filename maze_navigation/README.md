# Maze Navigation Starter Package

This package provides the starting skeleton for your ROS 2 Final Project on Potential Field Navigation. It is designed to work with ROS 2 Jazzy and Gazebo Sim.

## Contents

- `worlds/simple_maze.world`: A 10x10m maze environment in SDF 1.8 format. Note: You may need to add missing system plugins for sensors to work!
- `launch/maze_sim.launch.py`: A skeleton launch file. You must complete this file to start Gazebo Sim, spawn your chosen robot, and bridge the necessary topics (`/scan`, `/odom`, `/cmd_vel`).
- `maze_navigation/potential_field_planner.py`: A skeleton ROS 2 node. You must initialize the subscribers and publishers, handle the sensor data, and implement the Potential Field algorithm inside the `control_loop` method.

## Topics Required

Your completed planner node should use the following ROS 2 topics:
- `/odom` (`nav_msgs/msg/Odometry`): Current robot position and orientation.
- `/scan` (`sensor_msgs/msg/LaserScan`): LiDAR data for obstacle detection.
- `/cmd_vel`: Velocity commands sent to the robot. (Hint: check if your robot expects `Twist` or `TwistStamped` in Jazzy).

## Parameters

You can tune the planner behavior without recompiling by passing parameters at runtime:
- `goal_x`, `goal_y`: The target coordinates.
- `k_att`: Attractive gain (pull towards goal).
- `k_rep`: Repulsive gain (push away from walls).
- `d_obs`: Distance of influence for obstacles.

Example:
```bash
ros2 run maze_navigation potential_field_planner --ros-args -p goal_x:=9.0 -p goal_y:=9.0 -p k_rep:=250.0
```

Good luck with your implementation!
