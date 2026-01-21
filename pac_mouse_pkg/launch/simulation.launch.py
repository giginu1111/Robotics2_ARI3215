import os
import xacro
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # ========================================================================
    # 1. SETUP & PATHS
    # ========================================================================
    pkg_share = get_package_share_directory('pac_mouse_pkg')
    
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
    world_file       = os.path.join(pkg_share, 'worlds', 'maze_v3.sdf')

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
        arguments=['-string', cat_xml, '-name', 'cat', '-x', '3.0', '-y', '3.0', '-z', '0.1'],
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
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_config,
            'use_sim_time': 'true'
        }.items()
    )
    # 10. NAVIGATION 2 ( The "Smart" Driver )
    # We use the default launch but tell it NOT to run AMCL (localization) or a Map Server
    # because SLAM Toolbox is already doing that.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': os.path.join(pkg_share, 'config', 'nav2_params.yaml'), # We will create this next
        }.items()
    )

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

    # ========================================================================
    # 9. LAUNCH SEQUENCE
    # ========================================================================
    # Delay sensitive nodes to allow Gazebo to start publishing /clock
    delayed_nodes = TimerAction(
        period=5.0, 
        actions=[
            mouse_ekf,
            rviz,
            fix_mouse_lidar,
            fix_cat_lidar,
            slam_toolbox
        ]
    )
    extra_delayed_nodes = TimerAction(
        period=7.0,
        actions=[
            game_master,
            nav2_launch
        ]
    )

    return LaunchDescription([
        set_model_path,
        gazebo,
        spawn_mouse,
        spawn_cat,
        mouse_rsp,
        cat_rsp,
        bridge,
        delayed_nodes,
        extra_delayed_nodes
    ])