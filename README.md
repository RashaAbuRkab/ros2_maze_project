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



