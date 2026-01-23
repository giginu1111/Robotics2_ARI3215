"""
=============================================================================
COMPLETE PAC-MOUSE SIMULATION LAUNCH FILE
=============================================================================
This launch file orchestrates the complete simulation environment including:
- Gazebo physics simulation with custom maze world
- Two differential drive robots (cat and mouse)
- SLAM for mapping and localization
- Nav2 stack for autonomous navigation
- ROS-Gazebo bridge for sensor/actuator communication
- Custom AI controllers for both agents
- Game master for scoring and state management
- RViz for visualization
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
    # ========================================================================
    # 1. SETUP & PATHS
    # ========================================================================
    pkg_share = get_package_share_directory('pac_mouse_pkg')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')
    
    # Ensure Gazebo can find models/meshes
    set_model_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
        value=os.path.join(pkg_share, 'models')
    )

    # Config Files
    bridge_config    = os.path.join(pkg_share, 'config', 'bridge_params.yaml')
    ekf_config       = os.path.join(pkg_share, 'config', 'ekf.yaml')
    slam_config      = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    rviz_config      = os.path.join(pkg_share, 'rviz', 'mouse_view.rviz')
    world_file       = os.path.join(pkg_share, 'worlds', 'maze_v3_scaled_1.5.sdf')

    # ========================================================================
    # 2. PROCESS URDFs (XACRO)
    # ========================================================================
    # We pass 'robot_name' to create namespaced frames (e.g., mouse/base_link)
    
    # Process Mouse
    mouse_file = os.path.join(pkg_share, 'urdf', 'mouse.urdf.xacro')
    mouse_doc = xacro.process_file(mouse_file, mappings={'robot_name': 'mouse'})
    mouse_xml = mouse_doc.toxml()

    # Process Cat
    cat_file = os.path.join(pkg_share, 'urdf', 'cat.urdf.xacro')
    cat_doc = xacro.process_file(cat_file, mappings={'robot_name': 'cat'})
    cat_xml = cat_doc.toxml()

    # ========================================================================
    # 3. GAZEBO SIMULATION
    # ========================================================================
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # ========================================================================
    # 4. SPAWN ROBOTS
    # ========================================================================
    spawn_mouse = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-string', mouse_xml, '-name', 'mouse', '-x', '-3.0', '-y', '-3.0', '-z', '0.1'],
        output='screen'
    )

    spawn_cat = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-string', cat_xml, '-name', 'cat', '-x', '4.0', '-y', '4.0', '-z', '0.5'],
        output='screen'
    )

    # ========================================================================
    # 5. STATE PUBLISHERS & TF
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

    # Static TF Fixes (Gazebo Sensor Frame Alignment)
    # Fixes nested sensor frames that sometimes fail to link in Gazebo
    fix_mouse_lidar = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0.05', '0', '0.04', '0', '0', '0', 'mouse/base_link', 'mouse/mouse/base_link/lidar'],
        output='screen'
    )

    fix_cat_lidar = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['0.05', '0', '0.10', '0', '0', '0', 'cat/base_link', 'cat/cat/base_link/lidar'],
        output='screen'
    )

    # ========================================================================
    # 6. ROS-GAZEBO BRIDGE
    # ========================================================================
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        remappings=[
            ('/mouse/tf', '/tf'),
            ('/cat/tf', '/tf')
        ],
        output='screen'
    )

    # ========================================================================
    # 7. LOCALIZATION (EKF & SLAM)
    # ========================================================================
    
    # EKF for Mouse (Sensor Fusion)
    mouse_ekf = Node(
        package='robot_localization', executable='ekf_node',
        name='ekf_filter_node',
        namespace='mouse',
        output='screen',
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
        ]
    )

    # SLAM Toolbox (Mapping)
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_config,
            'use_sim_time': 'true'
        }.items()
    )
    # 8. NAVIGATION (Manual Launch - Bypasses Collision Monitor Error)
    # We launch only the 4 essential nodes needed to drive.
    nav_nodes = []
    
    # A. Controller (The Driver)
    nav_nodes.append(Node(
        package='nav2_controller', executable='controller_server',
        output='screen', parameters=[nav2_params_file],
        remappings=[('cmd_vel', '/mouse/cmd_vel')] # WIRE DIRECTLY TO MOUSE
    ))

    # B. Planner (The Map Reader)
    nav_nodes.append(Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen', parameters=[nav2_params_file]
    ))

    # C. Behaviors (Recovery actions like backing up)
    nav_nodes.append(Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen', parameters=[nav2_params_file],
        remappings=[('cmd_vel', '/mouse/cmd_vel')]
    ))

    # D. BT Navigator (The Brain that coordinates them)
    nav_nodes.append(Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen', parameters=[nav2_params_file]
    ))

    # E. Lifecycle Manager (Turns them all on)
    nav_nodes.append(Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen',
        parameters=[{'use_sim_time': True, 
                     'autostart': True, 
                     'node_names': ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']}]
    ))

    # ========================================================================
    # 8. VISUALIZATION (RVIZ)
    # ========================================================================
    rviz = Node(
        package='rviz2', executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    # GAME MASTER (Referee)
    game_master = Node(
        package='pac_mouse_pkg',
        executable='game_master',
        name='game_master',
        output='screen'
    )

    mouse_brain = Node(
        package='pac_mouse_pkg',
        executable='hybrid_explorer_mouse',
        name='mouse_brain',
        output='screen'
    )

    cat_brain = Node(
        package='pac_mouse_pkg',
        executable='cat_brain',
        name='cat_brain',
        output='screen'
    )
    # 1. Game Master in its own window
    game_master_cmd = ExecuteProcess(
        cmd=['terminator', '-T', 'Game Master', '-x', 'ros2', 'run', 'pac_mouse_pkg', 'game_master'],
        output='screen'
    )

    # 2. Mouse Brain in its own window
    mouse_brain_cmd = ExecuteProcess(
        cmd=['terminator', '-T', 'Mouse Brain', '-x', 'ros2', 'run', 'pac_mouse_pkg', 'hybrid_explorer_mouse'],
        output='screen'
    )

    # 3. Cat Brain in its own window
    cat_brain_cmd = ExecuteProcess(
        cmd=['terminator', '-T', 'Cat Brain', '-x', 'ros2', 'run', 'pac_mouse_pkg', 'cat_brain'],
        output='screen'
    )

    # ========================================================================
    # 9. LAUNCH SEQUENCE
    # ========================================================================
    # Delay sensitive nodes to allow Gazebo to start publishing /clock
    delay_first_nodes = TimerAction(
        period=10.0,
        actions=[
            spawn_mouse,
            spawn_cat,
            mouse_rsp,
            cat_rsp,
            bridge
        ]
    )
    
    delayed_nodes = TimerAction(
        period=15.0, 
        actions=[
            mouse_ekf,
            rviz,
            fix_mouse_lidar,
            fix_cat_lidar,
            slam_toolbox
        ]
    )
    extra_delayed_nodes = TimerAction(
        period=20.0,
        actions=[
            *nav_nodes
        ]
    )

    extra_extra_delayed_nodes = TimerAction(
        period=30.0,
        actions=[
            game_master,
            mouse_brain,
            cat_brain
            #,
            #game_master_cmd,
            #mouse_brain_cmd,
            #cat_brain_cmd
        ]
    )

    return LaunchDescription([
        set_model_path,
        gazebo,
        delay_first_nodes,
        delayed_nodes,
        extra_delayed_nodes,
        extra_extra_delayed_nodes
        ]
    )