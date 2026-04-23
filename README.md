## Project: Autonomous Navigation using Potential Field Method
ROS 2 Maze Navigation using the Potential Field Method.  featuring autonomous robot navigation in Gazebo Sim with obstacle avoidance and local minima recovery.


---

### 3.1 Task 1: Robot Selection and Integration

#### 1. Selected Mobile Robot
For this project, our group has selected the **TurtleBot3 (Burger)**.
*   **Robot Type:** Differential Drive Mobile Robot.
*   **Source:** [ROS Ground Robots Directory](https://robots.ros.org).
*   **Rationale:** We chose this platform due to its robust support for **ROS 2 Jazzy** and its integrated **LiDAR** sensor, which is essential for processing environmental data and implementing the **Potential Field** obstacle avoidance algorithm.

---

### 2. Workspace Initialization
First, we established a dedicated ROS 2 workspace to manage our project packages and dependencies.

```bash
# Create the workspace and source directory
mkdir -p ~/ros2_project_ws/src
cd ~/ros2_project_ws/src
```

#### 3. Package Integration (Jazzy Distribution)
We cloned the official repositories, specifically targeting the jazzy branches to ensure compatibility with Gazebo Sim (Harmonic).

```bash
# Clone the main TurtleBot3 packages
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git

# Clone the simulation-specific packages
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

#### 4. Dependency Management & Compilation
To ensure all system-level dependencies are met, we utilized rosdep before compiling the workspace using colcon.
```bash
# Navigate to workspace root
cd ~/ros2_project_ws

# Update and install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build

# Source the overlay
source install/setup.bash
```
#### 5. Verification: Spawning in Gazebo Sim
To verify the integration, we defined the robot model environment variable and launched the simulation in an empty world.
```bash
# Set the environment variable for the robot model
export TURTLEBOT3_MODEL=burger

# Launch the Gazebo Sim environment
ros2 launch turtlebot3_gazebo empty_world.launch.py
```
<img width="1911" height="1012" alt="image" src="https://github.com/user-attachments/assets/8f0952d9-d132-4c08-b287-516eb2e23519" />

### 3.2 Task 2: Environment Setup

#### 1. World File Configuration (simple_maze.world)
To ensure compatibility with ROS 2 Jazzy and Gazebo Harmonic, we updated the provided world file to include essential system plugins and corrected the library namespaces.

* Namespace Migration: Updated all plugins from the ignition::gazebo namespace to the modern gz::sim namespace (e.g., gz::sim::systems::Physics).
* Sensor Simulation: Integrated the gz-sim-sensors-system plugin. This is critical for simulating the LiDAR sensor, allowing the environment to generate laser scan data.
* Physics Tuning: Configured the physics engine with a 0.001 step size to ensure stable interaction between the robot and the maze walls.

#### 2. Launch System Implementation (maze_sim.launch.py)
We developed a comprehensive launch file to automate the simulation startup. The script performs the following operations:
* Automatic Resource Discovery: Utilizes AppendEnvironmentVariable to dynamically add the TurtleBot3 model paths to GZ_SIM_RESOURCE_PATH, eliminating "Model not found" errors.
* Gazebo Initialization: Launches Gazebo Sim with the custom simple_maze.world in a ready-to-run state.
* Precise Spawning: Spawns the TurtleBot3 (Burger) at the required starting coordinates: x=0.5, y=0.5.
* Communication Bridge: Initializes the parameter_bridge to connect Gazebo's transport system with ROS 2 topics.

#### 3. Topic Mapping & Bridge Configuration
A robust communication bridge was established to ensure seamless data flow between the simulator and the navigation stack:

| Topic | Type | Direction | Purpose |
| :--- | :--- | :--- | :--- |
| `/scan` | `sensor_msgs/msg/LaserScan` | GZ ➔ ROS | Environment perception for obstacle avoidance. |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS ➔ GZ | Motor control commands for navigation. |
| `/odom` | `nav_msgs/msg/Odometry` | GZ ➔ ROS | Real-time robot localization and pose tracking. |
| `/clock` | `rosgraph_msgs/msg/Clock` | GZ ➔ ROS | Time synchronization between ROS and Gazebo. |
| `/world/maze_world/dynamic_pose/info` | `tf2_msgs/TFMessage` |  GZ ➔ ROS | The robot pose in the Gazebo sim

#### 4. Build and Execution Instructions
Follow these steps to compile the package and launch the environment:
```
# 1. Build the specific package
cd ~/ros2_project_ws
colcon build --packages-select maze_navigation

# 2. Source the workspace
source install/setup.bash

# 3. Launch the simulation
ros2 launch maze_navigation maze_sim.launch.py
```
#### 5. Verification: Integration Testing
To verify the setup, we performed a manual control test to confirm that all systems are operational:
* Movement: Used teleop_twist_keyboard to command the robot; the robot responded correctly to velocity commands in the Gazebo environment.
* Sensors: Verified LiDAR data stream using ros2 topic echo /scan, confirming that the robot can "see" the maze walls.
bash
#### Run teleop to verify movement
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

<img width="1913" height="1011" alt="image" src="https://github.com/user-attachments/assets/1a887e54-3069-4720-8e63-3c4b3150aa03" />

### 3.3 Task 3, 4: Potential Field Navigation Implementation 
Autonomous maze navigation for a **TurtleBot3 Burger** in **Gazebo Sim** under **ROS 2 Jazzy**.  
The robot navigates from a spawn point at `(0.5, 0.5)` to a goal at `(9.0, 9.0)` inside a ~10 × 10 m maze using an Artificial Potential Field planner with an automatic wall-following escape for local minima.
 
---
 
## How It Works
 
The planner runs as a single ROS 2 node (`potential_field_planner`) at **10 Hz** and operates as a two-state machine:
 
```
┌─────────────┐   stuck detected    ┌──────────────────┐
│   STATE_APF │ ──────────────────► │ STATE_WALL_FOLLOW │
│  (default)  │ ◄────────────────── │   (escape mode)  │
└─────────────┘   goal path clear   └──────────────────┘
```
 
---

## State 1 — Artificial Potential Field (APF)
 
The robot computes a virtual force from two components and follows the resulting gradient.
 
### Attractive Force
Pulls the robot toward the goal `(goal_x, goal_y)`:
 
$$\vec{F}_{att} = -k_{att} \cdot \vec{d}_{goal}$$
 
### Repulsive Force
Pushes the robot away from any obstacle within influence radius `d_obs`.  
LiDAR returns are segmented into contiguous clusters; the closest point of each cluster contributes:
 
$$\vec{F}_{rep} = k_{rep} \left(\frac{1}{d_{obs}} - \frac{1}{d}\right) \frac{1}{d^3} \cdot \vec{d}_{obs} \quad \text{for } d \leq d_{obs}$$
 
### Heading & Speed Control
- The desired heading is `atan2(Fy, Fx)` of the combined force vector.
- For large heading errors (`> π/6 rad`) the robot rotates in place at minimum speed.
- Forward speed scales with the distance to the nearest obstacle (`√(closest / d_obs)`) and with proximity to the goal, so the robot slows down naturally near walls and near the target.
### Ground-Truth Pose
Pose is sourced from Gazebo's dynamic pose bridge (`/world/maze_world/dynamic_pose/info`, type `tf2_msgs/TFMessage`) rather than wheel odometry, eliminating drift. The robot's `base_link` transform is identified by its z-height (~0.01 m).
 
---

## State 2 — Wall-Following Escape
 
### Why It Is Needed
A local minimum occurs when repulsive forces from two or more obstacle surfaces exactly cancel the attractive force, trapping the robot. This is common in concave geometries such as U-shaped corridors or corners where the goal lies behind a wall.
 
### Stuck Detection
A sliding position window tracks `(timestamp, x, y)` over the last **4 seconds**.  
Net displacement between the oldest and newest entry is computed each tick:
 
$$d_{net} = \sqrt{(x - x_0)^2 + (y - y_0)^2}$$
 
If `d_net < 0.20 m` the stuck timer increments; if not, it resets. After **3 continuous seconds** below the threshold the escape state is triggered. This catches both a fully stationary robot and a slowly circling one.
 
### Right-Hand Rule
On entry the robot commits to keeping the **right wall** on its right side for the duration of the escape. This guarantees complete traversal of any simply-connected obstacle region without revisiting corridors.
 
### Two-Sensor Parallel Controller
A single 90° range reading encodes only wall *distance*, not wall *alignment*, causing a distance-only P-controller to oscillate and circle. Three LiDAR sectors are used instead:
 
```
Robot frame  (+x forward)
 
       d_fs (−45°)  ╱
                   ╱
  ════════════════● ──────► +x
  RIGHT WALL       ╲
       d_rs (−135°)  ╲
       wall_dist (−90°) ↓
```
 
Two independent errors are computed:
 
| Error | Formula | Meaning |
|---|---|---|
| Distance | `e_dist = wall_dist − 0.30` | Too close or too far from wall |
| Alignment | `e_align = d_fs − d_rs` | Nose angling into or away from wall |
 
Combined angular command:
 
$$\omega = -(k_{wall} \cdot e_{dist} + k_{align} \cdot e_{align})$$
 
When both errors are zero the robot drives dead straight parallel to the wall. The alignment term damps heading error before it grows into a distance error, eliminating oscillation.

### Return to APF
The robot exits wall-following and resumes APF when:
- The LiDAR sector in the direction of the goal is clear (`> 1.1 × d_obs`), **and**
- At least **3 seconds** have elapsed in the escape state (ensuring the robot has cleared the local minimum geometry).
A hard timeout of **30 seconds** forces a return to APF as a safety net.
 
---
 
## Parameters
 
| Parameter | Default | Description |
|---|---|---|
| `goal_x` | `9.0` | Goal x-coordinate [m] |
| `goal_y` | `9.0` | Goal y-coordinate [m] |
| `k_att` | `1.0` | Attractive force gain |
| `k_rep` | `3.0` | Repulsive force gain |
| `d_obs` | `1.2` | Obstacle influence radius [m] |
| `max_linear_vel` | `0.4` | Maximum forward speed [m/s] |
| `max_angular_vel` | `1.0` | Maximum turn rate [rad/s] |
 
---

## Running
 
```bash
# Build
cd ~/ros2_project_ws
colcon build --symlink-install --packages-select maze_navigation
source install/setup.bash
 
# Launch Gazebo with the maze world (adjust to your launch file) and the potential_field_planner node
ros2 launch maze_navigation maze.launch.py
 



