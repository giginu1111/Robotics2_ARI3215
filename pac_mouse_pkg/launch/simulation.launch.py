import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('pac_mouse_pkg')

    # 1. PROCESS URDF
    xacro_file = os.path.join(pkg_share, 'urdf', 'mouse.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_desc_xml = doc.toxml()

    # 2. CONFIG
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'mouse_view.rviz')
    world_file = os.path.join(pkg_share, 'worlds', 'maze.sdf') 
    gazebo_ros_pkg = get_package_share_directory('ros_gz_sim')

    # ========================================================================
    # A. GAZEBO & BRIDGE (The Time Source)
    # ========================================================================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    node_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', '/robot_description', '-name', 'squeak_mouse', '-z', '0.1'],
        output='screen'
    )

    # BRIDGE: NOTICE I REMOVED '/tf' TO PREVENT CONFLICTS
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model', # Fixed mapping
        ],
        remappings=[
            ('/joint_states', '/joint_states') # Ensure explicit mapping
        ],
        output='screen'
    )

    # ========================================================================
    # B. ROBOT NODES (Forced Simulation Time)
    # ========================================================================
    
    # 1. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc_xml,
            'use_sim_time': True 
        }]
    )

    # 2. TF Fix for Lidar (Updated syntax)
    node_tf_fix = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'squeak_mouse/base_link/lidar'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 3. RViz
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    # DELAY GROUP: Wait 5 seconds for Bridge to establish Clock
    delayed_nodes = TimerAction(
        period=5.0, 
        actions=[node_robot_state_publisher, node_tf_fix, node_rviz]
    )

    return LaunchDescription([
        gazebo,
        node_spawn,
        node_ros_gz_bridge,
        delayed_nodes
    ])