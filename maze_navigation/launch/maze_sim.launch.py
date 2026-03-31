import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_maze_nav = get_package_share_directory('maze_navigation')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Path to our custom maze world
    world_file = os.path.join(pkg_maze_nav, 'worlds', 'simple_maze.world')
    
    # TODO: Identify the packages required to launch Gazebo Sim and your chosen robot
    
    # TODO: 1. Start Gazebo Sim with the custom world file
    gz_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # TODO: 2. Spawn your chosen robot at x=0.5, y=0.5
    start_robot_spawner_cmd =Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot3',
            '-x', '0.5',
            '-y', '0.5',
            '-z', '0.1',
            '-file', os.path.join(get_package_share_directory('turtlebot3_gazebo'), 
                                  'models', 'turtlebot3_burger', 'model.sdf')
        ],
        output='screen',
    )

    # TODO: 3. Start the parameter bridge to connect Gazebo topics to ROS 2 topics
    # Ensure you bridge at least /scan, /odom, and /cmd_vel
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_sim_cmd,
        start_robot_spawner_cmd,
        start_gazebo_ros_bridge_cmd
    ])
