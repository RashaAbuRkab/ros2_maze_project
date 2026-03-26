# Project: Autonomous Navigation using Potential Field Method
ROS 2 Maze Navigation using the Potential Field Method.  featuring autonomous robot navigation in Gazebo Sim with obstacle avoidance and local minima recovery.

**Status:** Task 3.1 Completed  

---

## 3.1 Task 1: Robot Selection and Integration

### 1. Selected Mobile Robot
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

### 3. Package Integration (Jazzy Distribution)
We cloned the official repositories, specifically targeting the jazzy branches to ensure compatibility with Gazebo Sim (Harmonic).

```bash
# Clone the main TurtleBot3 packages
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3.git

# Clone the simulation-specific packages
git clone -b jazzy https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
```

### 4. Dependency Management & Compilation
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
### 5. Verification: Spawning in Gazebo Sim
To verify the integration, we defined the robot model environment variable and launched the simulation in an empty world.
```bash
# Set the environment variable for the robot model
export TURTLEBOT3_MODEL=burger

# Launch the Gazebo Sim environment
ros2 launch turtlebot3_gazebo empty_world.launch.py
```


