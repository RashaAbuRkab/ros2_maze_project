import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_maze_nav = get_package_share_directory('maze_navigation')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Path to our custom maze world
    world_file = os.path.join(pkg_maze_nav, 'worlds', 'complex_maze.world')
    
    # Identify the packages required to launch Gazebo Sim and your chosen robot
    
    # 1. Start Gazebo Sim with the custom world file
    gz_sim_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}',
                          'use_sim_time': "true"}.items(),
    )

    # 2. Spawn your chosen robot at x=0.6, y=0.6
    start_robot_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'turtlebot3',
            '-x', '0.6',
            '-y', '0.6',
            '-z', '0.1',
            '-file', os.path.join(get_package_share_directory('turtlebot3_gazebo'), 
                                  'models', 'turtlebot3_burger', 'model.sdf')
        ],
        output='screen',
    )

    # 3. Start the parameter bridge to connect Gazebo topics to ROS 2 topics
    # Ensure you bridge at least /scan, /odom, and /cmd_vel
    start_gazebo_ros_bridge_cmd = TimerAction(period=5.0, actions=[Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/turtlebot3/pose@geometry_msgs/msg/Pose[gz.msgs.Pose',
            '/world/complex_maze_world/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        parameters=[{
        'qos_overrides./tf_static.publisher.durability': 'transient_local',
        'qos_overrides./tf_static.publisher.reliability': 'reliable',
        'qos_overrides./tf.publisher.reliability': 'reliable',
        'qos_overrides./tf.publisher.durability': 'volatile',
        'qos_overrides./clock.publisher.reliability':     'best_effort',
        'qos_overrides./clock.publisher.durability':      'volatile',
         }],
        output='screen'
    )
    ])

    # Load TurtleBot3 URDF (for TF only)
    urdf_file = PathJoinSubstitution([
        FindPackageShare("turtlebot3_description"),
        "urdf",
        "turtlebot3_burger.urdf"
    ])

    robot_description = ParameterValue(
        Command(["xacro ", urdf_file]),
        value_type=str
    )

    robot_state_publisher_node = TimerAction(period=12.0, actions=[
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True,
                'publish_frequency': 50.0,
                'frame_prefix': '',
            }],
            output='screen'
        )
    ])

    # joint_state_publisher_node = TimerAction(period=9.0, actions=[
    #     Node(
    #         package='joint_state_publisher',
    #         executable='joint_state_publisher',
    #         parameters=[{'use_sim_time': True}],
    #         output='screen'
    #     )
    # ])

    # # 5. SLAM Toolbox (async online) — map->odom TF
    # slam_toolbox_cmd = TimerAction(period=15.0, actions=[
    #     Node(
    #         package='slam_toolbox',
    #         executable='async_slam_toolbox_node',
    #         name='slam_toolbox',
    #         parameters=[{
    #         'use_sim_time': True,
    #         'odom_frame': 'odom',
    #         'map_frame': 'map',
    #         'base_frame': 'base_footprint',
    #         'scan_topic': '/scan',
    #         'mode': 'mapping',
    #         'map_update_interval': 0.1,          # was 0.2 — faster map updates
    #         'minimum_travel_distance': 0.001,    # was 0.0099 — update on tiny movements
    #         'minimum_travel_heading': 0.001,     # was 0.0099 — update on tiny rotations
    #         'transform_publish_period': 0.02,    # ADD — publish map→odom at 50Hz
    #         'tf_buffer_duration': 30.0,          # ADD — longer TF history
    #         'max_laser_range': 10.0,
    #         'resolution': 0.05,
    #         'do_loop_closing': True,
    #     }],
    #         output='screen'
    #     )
    # ])

    planner_cmd = TimerAction(period=15.0, actions=[
        Node(
            package='maze_navigation',
            executable='potential_field_planner2',
            name='potential_field_planner2',
            parameters=[{
                'use_sim_time': True,
            }],
            output='screen'
        )
    ])

    return LaunchDescription([
        gz_sim_cmd,
        start_robot_spawner_cmd,
        start_gazebo_ros_bridge_cmd,
        robot_state_publisher_node,
        #joint_state_publisher_node,
        #slam_toolbox_cmd,
        planner_cmd,
        #republish_static_tf,
    ])