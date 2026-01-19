import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('pac_mouse_pkg')

    # 1. RESOURCES
    xacro_file = os.path.join(pkg_share, 'urdf', 'mouse.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_desc_xml = doc.toxml()
    
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'mouse_view.rviz')
    world_file = os.path.join(pkg_share, 'worlds', 'maze.sdf') 
    ekf_config_file = os.path.join(pkg_share, 'config', 'ekf.yaml')
    
    gazebo_ros_pkg = get_package_share_directory('ros_gz_sim')

    # 2. GAZEBO SIMULATION
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    node_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description', 
            '-name', 'squeak_mouse', 
            '-x', '-3.0',  # Move back 3 meters
            '-y', '-3.0',  # Move left 3 meters (Safe corner)
            '-z', '0.1',   # Lift up slightly
            '-Y', '0.0'    # Face East (0.0 radians)
        ],
        output='screen'
    )

    # 3. BRIDGE (Jazzy / Harmonic Compatible)
    # The syntax [gz.msgs... is standard and should work fine in Jazzy
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry', 
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',         
        ],
        output='screen'
    )

    # 4. ROBOT STATE PUBLISHER
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc_xml,
            'use_sim_time': True 
        }]
    )

    # 5. JOINT STATE PUBLISHER
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # 6. TF FIXES (Glue Gazebo frames to Robot frames)
    
    # Fix 1: Connect Lidar (Matches your echo output)
    node_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_lidar_fix',
        # Connect internal 'lidar_link' to Gazebo's 'squeak_mouse/base_link/lidar'
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'squeak_mouse/base_link/lidar'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Fix 2: Connect Base Link (Fixes the "Map not existing" / EKF issue)
    node_tf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_fix',
        # Connect internal 'base_link' to Gazebo's 'squeak_mouse/base_link'
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'squeak_mouse/base_link'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 7. EKF (SENSOR FUSION)
    node_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': True}]
    )

    # 8. SLAM TOOLBOX (Jazzy Compatible)
    slam_config_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    node_slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_file, {'use_sim_time': True}]
    )

    # 9. RVIZ
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    # DELAY GROUP
    delayed_nodes = TimerAction(
        period=5.0, 
        actions=[
            node_robot_state_publisher, 
            node_joint_state_publisher, 
            node_tf_lidar,  # <--- Added
            node_tf_base,   # <--- Added
            node_ekf, 
            node_slam, 
            node_rviz
        ]
    )

    return LaunchDescription([
        gazebo,
        node_spawn,
        node_ros_gz_bridge,
        delayed_nodes
    ])