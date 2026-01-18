import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Configuration
    # Replace 'my_robot_pkg' with your actual package name
    pkg_name = 'pac_mouse_pkg' 
    pkg_share = get_package_share_directory(pkg_name)
    
    # rviz config file
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'mouse_view.rviz')
    
    # WORLD FILE
    world_file = os.path.join(pkg_share, 'worlds', 'maze.sdf')
    
    # Path to your URDF file
    xacro_file = os.path.join(pkg_share, 'urdf', 'mouse.urdf.xacro')
    
    # 2. Process the URDF
    # We use 'xacro' to convert the .xacro file to a raw URDF string
    robot_description_raw = Command(['xacro ', xacro_file])
    
    # 3. Nodes
    
    # A. Robot State Publisher
    # Publishes the TF tree so Rviz knows where the robot parts are
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
                     'use_sim_time': True}] # Important for syncing with Gazebo
    )

    # B. Gazebo Simulation (gz-sim)
    # Starts an empty world. You can change 'empty.sdf' to your own maze world later.
    include_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r ' + world_file}.items(),
    )

    # C. Spawn the Robot
    # Takes the robot description and spawns it into Gazebo at (0,0,0.1)
    node_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'squeak_mouse',
            '-z', '0.1'
        ],
        output='screen'
    )

    # D. ROS-Gazebo Bridge
    # This maps Gazebo topics to ROS 2 topics.
    # Syntax: <topic_name>@<ros_type>[<gazebo_type>
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # 1. Clock (Sync time)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # 2. Cmd_vel (ROS -> Gazebo) for moving
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # 3. Odometry (Gazebo -> ROS)
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # 4. TF (Gazebo -> ROS) for map->odom->base_link
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            # 5. Lidar Scan (Gazebo -> ROS)
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # 6. Camera Image (Gazebo -> ROS)
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            # 7. IMU (Gazebo -> ROS)
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        output='screen'
    )

    # E. Rviz2 (Visualization)
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file], # Load the config!
        parameters=[{'use_sim_time': True}] # Force sim time
    )
    
    # F. Fix Frame Mismatch
    # Connects 'squeak_mouse/base_link/lidar' (Gazebo name) to 'lidar_link' (URDF name)
    node_tf_fix = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'squeak_mouse/base_link/lidar'],
        output='screen'
    )

    return LaunchDescription([
        node_robot_state_publisher,
        include_gazebo,
        node_spawn_entity,
        node_ros_gz_bridge,
        node_rviz,
        node_tf_fix
    ])