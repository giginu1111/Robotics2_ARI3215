#!/usr/bin/env python3
"""
=============================================================================
COMPLETE PAC-MOUSE SIMULATION LAUNCH FILE
=============================================================================
Course: Robotics 2 (ARI3215)
Project: Autonomous Cat and Mouse Chase Simulation

This launch file orchestrates the complete simulation environment including:
- Gazebo physics simulation with custom maze world
- Two differential drive robots (cat and mouse)
- SLAM for mapping and localization
- Nav2 stack for autonomous navigation
- ROS-Gazebo bridge for sensor/actuator communication
- Custom AI controllers for both agents
- Game master for scoring and state management
- RViz for visualization

Author: Damian Cutajar
Date: January 2026
=============================================================================
"""

import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, 
    TimerAction, 
    AppendEnvironmentVariable, 
    DeclareLaunchArgument,
    ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the complete launch description for the pac-mouse simulation.
    
    Returns:
        LaunchDescription: Complete launch configuration
    """
    
    # ========================================================================
    # SECTION 1: PACKAGE PATHS AND CONFIGURATION FILES
    # ========================================================================
    pkg_share = get_package_share_directory('pac_mouse_pkg')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')
    
    # Configuration file paths
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    bridge_config = os.path.join(pkg_share, 'config', 'bridge_params.yaml')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    slam_config = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'mouse_view.rviz')
    world_file = os.path.join(pkg_share, 'worlds', 'maze_v3_scaled_1.5.sdf')
    
    # Ensure Gazebo can find custom models
    set_model_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
        value=os.path.join(pkg_share, 'models')
    )

    # ========================================================================
    # SECTION 2: LAUNCH ARGUMENTS FOR CONFIGURABILITY
    # ========================================================================
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_mouse_brain = DeclareLaunchArgument(
        'mouse_controller',
        default_value='hybrid_explorer_mouse',
        description='Mouse AI controller: hybrid_explorer_mouse, smart_mouse, or teleop'
    )
    
    declare_cat_brain = DeclareLaunchArgument(
        'cat_controller',
        default_value='cat_brain',
        description='Cat AI controller executable name'
    )

    # ========================================================================
    # SECTION 3: ROBOT DESCRIPTION PROCESSING (URDF/XACRO)
    # ========================================================================
    # Process Mouse URDF with xacro
    mouse_urdf_file = os.path.join(pkg_share, 'urdf', 'mouse.urdf.xacro')
    mouse_doc = xacro.process_file(mouse_urdf_file, mappings={'robot_name': 'mouse'})
    mouse_robot_description = mouse_doc.toxml()

    # Process Cat URDF with xacro
    cat_urdf_file = os.path.join(pkg_share, 'urdf', 'cat.urdf.xacro')
    cat_doc = xacro.process_file(cat_urdf_file, mappings={'robot_name': 'cat'})
    cat_robot_description = cat_doc.toxml()

    # ========================================================================
    # SECTION 4: GAZEBO SIMULATION ENVIRONMENT
    # ========================================================================
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 
                        'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': f'-r {world_file}',  # -r flag starts simulation paused
            'on_exit_shutdown': 'true'
        }.items(),
    )

    # ========================================================================
    # SECTION 5: ROBOT SPAWNING
    # ========================================================================
    # Spawn Mouse at bottom-left corner
    spawn_mouse = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', mouse_robot_description,
            '-name', 'mouse',
            '-x', '-3.0',
            '-y', '-3.0',
            '-z', '0.1',
            '-Y', '0.0'  # Yaw orientation
        ],
        output='screen'
    )

    # Spawn Cat at top-right corner
    spawn_cat = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', cat_robot_description,
            '-name', 'cat',
            '-x', '4.0',
            '-y', '4.0',
            '-z', '0.5',
            '-Y', '3.14159'  # Facing opposite direction
        ],
        output='screen'
    )

    # ========================================================================
    # SECTION 6: ROBOT STATE PUBLISHERS (TF TREE)
    # ========================================================================
     # Mouse State Publisher
    mouse_rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace='mouse', 
        parameters=[{'use_sim_time': True, 'robot_description': mouse_xml}],
        output='screen'
    )

    # Cat State Publisher
    cat_rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        namespace='cat',
        parameters=[{'use_sim_time': True, 'robot_description': cat_xml}],
        output='screen'
    )

    # ========================================================================
    # SECTION 7: STATIC TF TRANSFORMS (SENSOR FRAME FIXES)
    # ========================================================================
    # Fix LiDAR frame alignment for mouse
    mouse_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='mouse_lidar_fix',
        arguments=['0.05', '0', '0.04', '0', '0', '0', 
                  'mouse/base_link', 'mouse/mouse/base_link/lidar'],
        output='screen'
    )

    # Fix LiDAR frame alignment for cat
    cat_lidar_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cat_lidar_fix',
        arguments=['0.05', '0', '0.10', '0', '0', '0', 
                  'cat/base_link', 'cat/cat/base_link/lidar'],
        output='screen'
    )

    # ========================================================================
    # SECTION 8: ROS-GAZEBO BRIDGE
    # ========================================================================
    # Bridges Gazebo topics to ROS 2 topics
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': bridge_config}],
        remappings=[
            ('/mouse/tf', '/tf'),
            ('/cat/tf', '/tf')
        ],
        output='screen'
    )

    # ========================================================================
    # SECTION 9: LOCALIZATION - EXTENDED KALMAN FILTER
    # ========================================================================
    # Fuses odometry and IMU data for better pose estimation
    mouse_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace='mouse',
        parameters=[
            ekf_config,
            {
                'use_sim_time': True,
                'map_frame': 'map',
                'odom_frame': 'mouse/odom',
                'base_link_frame': 'mouse/base_link',
                'world_frame': 'mouse/odom',
                'odom0': '/mouse/odom',
                'imu0': '/mouse/imu',
                'publish_tf': True
            }
        ],
        output='screen'
    )

    # ========================================================================
    # SECTION 10: SLAM - SIMULTANEOUS LOCALIZATION AND MAPPING
    # ========================================================================
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_config,
            'use_sim_time': 'true'
        }.items()
    )

    # ========================================================================
    # SECTION 11: NAVIGATION STACK (NAV2)
    # ========================================================================
    # Controller Server: Executes path plans
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}],
        remappings=[('cmd_vel', '/mouse/cmd_vel')]
    )

    # Planner Server: Generates optimal paths
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}]
    )

    # Behavior Server: Handles recovery behaviors
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}],
        remappings=[('cmd_vel', '/mouse/cmd_vel')]
    )

    # BT Navigator: Coordinates navigation using behavior trees
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file, {'use_sim_time': True}]
    )

    # Lifecycle Manager: Manages navigation node lifecycle
    nav_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator'
            ]
        }]
    )

    # ========================================================================
    # SECTION 12: CUSTOM AI CONTROLLERS
    # ========================================================================
    # Mouse Brain: Autonomous exploration and cheese collection
    mouse_brain = Node(
        package='pac_mouse_pkg',
        executable='mouse_brain',  # Changed from hybrid_explorer_mouse
        name='mouse_brain',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Cat Brain: Pursuit and interception behavior
    cat_brain = Node(
        package='pac_mouse_pkg',
        executable='cat_brain',  # Changed from cat_brain
        name='cat_brain',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # Game Master: Scoring, game state, and referee
    game_master = Node(
        package='pac_mouse_pkg',
        executable='game_master',  # Changed from game_master
        name='game_master',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # ========================================================================
    # SECTION 13: VISUALIZATION (RVIZ2)
    # ========================================================================
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ========================================================================
    # SECTION 14: TIMED LAUNCH SEQUENCE
    # ========================================================================
    # Phase 1: Start Gazebo and wait for /clock topic (5s)
    phase_1 = TimerAction(
        period=5.0,
        actions=[
            spawn_mouse,
            spawn_cat,
            mouse_rsp,
            cat_rsp,
            ros_gz_bridge
        ]
    )
    
    # Phase 2: Start localization and visualization (10s)
    phase_2 = TimerAction(
        period=10.0,
        actions=[
            mouse_ekf,
            mouse_lidar_tf,
            cat_lidar_tf,
            slam_toolbox,
            rviz
        ]
    )
    
    # Phase 3: Start navigation stack (15s)
    phase_3 = TimerAction(
        period=15.0,
        actions=[
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            nav_lifecycle_manager
        ]
    )
    
    # Phase 4: Start AI controllers (20s)
    phase_4 = TimerAction(
        period=20.0,
        actions=[
            game_master,
            mouse_brain,
            cat_brain
        ]
    )

    # ========================================================================
    # SECTION 15: LAUNCH DESCRIPTION
    # ========================================================================
    return LaunchDescription([
        # Arguments
        declare_use_sim_time,
        declare_mouse_brain,
        declare_cat_brain,
        
        # Environment setup
        set_model_path,
        
        # Gazebo
        gazebo_server,
        
        # Timed sequences
        phase_1,
        phase_2,
        phase_3,
        phase_4
    ])
