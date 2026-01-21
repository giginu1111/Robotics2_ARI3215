import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, AppendEnvironmentVariable # <--- UPDATED
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('pac_mouse_pkg')

    # --- NEW: TELL GAZEBO WHERE THE MODELS ARE ---
    # This automatically adds the install/share/pac_mouse_pkg/models folder
    # to the GZ_SIM_RESOURCE_PATH variable.
    set_model_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
        value=os.path.join(pkg_share, 'models')
    )
    # ---------------------------------------------

    # 1. RESOURCES
    mouse_file_path = os.path.join(pkg_share, 'urdf', 'mouse.urdf.xacro')
    mouse_doc = xacro.process_file(mouse_file_path)
    mouse_desc_xml = mouse_doc.toxml()

    cat_file_path = os.path.join(pkg_share, 'urdf', 'cat.urdf.xacro') 
    cat_doc = xacro.process_file(cat_file_path)
    cat_desc_xml = cat_doc.toxml()
    
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'mouse_view.rviz')
    world_file = os.path.join(pkg_share, 'worlds', 'maze.sdf') 
    
    gazebo_ros_pkg = get_package_share_directory('ros_gz_sim')

    # 2. GAZEBO SIMULATION
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    node_spawn_mouse = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', mouse_desc_xml,
            '-name', 'squeak_mouse', 
            '-x', '-3.0',
            '-y', '-3.0',
            '-z', '0.1',
            '-Y', '0.0'
        ],
        output='screen'
    )

    node_spawn_cat = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', cat_desc_xml,   # Pass the actual XML string
            '-name', 'doraemon',       # The name in Gazebo
            '-x', '0.0',               # Spawning at (0,0) - Center of maze?
            '-y', '0.0',
            '-z', '0.3',               # Slightly higher so it doesn't get stuck in floor
        ],
        output='screen'
    )

    # 3. BRIDGE
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # --- GLOBAL ---
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            # --- MOUSE TOPICS ---
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',

            # --- CAT TOPICS (NEW) ---
            # Command Velocity (ROS -> Gazebo)
            '/cat/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            
            # Sensors (Gazebo -> ROS)
            '/cat/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/cat/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/cat/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/cat/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
        ],
        output='screen'
    )

    # --- MOUSE PUBLISHERS (Namespace: 'mouse') ---
    node_mouse_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='mouse_state_publisher',
        namespace='mouse', # <--- Critical
        output='screen',
        parameters=[{'robot_description': mouse_desc_xml, 'use_sim_time': True}]
    )

    node_mouse_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='mouse_joint_state_publisher',
        namespace='mouse', # <--- Critical: Must match RSP
        parameters=[{'robot_description': mouse_desc_xml, 'use_sim_time': True}]
    )

    # --- CAT PUBLISHERS (Namespace: 'cat') ---
    node_cat_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='cat_state_publisher',
        namespace='cat', # <--- Critical
        output='screen',
        parameters=[{
            'robot_description': cat_desc_xml, 
            'use_sim_time': True,
            'frame_prefix': 'doraemon/' # <--- Ensures unique TF frames
        }]
    )

    node_cat_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='cat_joint_state_publisher',
        namespace='cat', # <--- Critical: Must match RSP
        parameters=[{
            'robot_description': cat_desc_xml, # <--- Must have XML to know about wheels
            'use_sim_time': True
        }]
    )

    # 6. TF FIXES
    node_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_lidar_fix',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'squeak_mouse/base_link/lidar'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    node_tf_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_base_fix',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'squeak_mouse/base_link'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 6.5 CAT INFRASTRUCTURE (Joints & TFs)
    # C. TF: Connect the Camera Data (Optional but good)
    node_cat_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cat_tf_camera_fix',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'doraemon/camera_link', 'camera_link_optical'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    # D. TF: Glue the Cat's Odom to the World Map
    # This says: "The Cat's starting point (cat/odom) is at (0,0) on the map."
    node_tf_map_to_cat = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_map_to_cat',
        # Arguments: x y z yaw pitch roll parent child
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'cat/odom'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 7. EKF
    ekf_config_file = os.path.join(pkg_share, 'config', 'ekf.yaml')
    
    node_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file, {'use_sim_time': True}]
    )

    # 8. SLAM TOOLBOX
    slam_config_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    toolbox_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
    ),
    launch_arguments={
        'slam_params_file': slam_config_file,
        'use_sim_time': 'true'
    }.items()
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
            # --- MOUSE NODES ---
            node_mouse_state_publisher,
            node_mouse_joint_state_publisher,
            node_tf_base, 
            node_ekf,

            # --- CAT NODES ---
            node_cat_state_publisher,
            node_cat_joint_state_publisher,
            node_cat_tf_camera,
            node_tf_map_to_cat,

            # --- RVIZ ---
            node_rviz
        ]
    )

    return LaunchDescription([
        set_model_path,
        gazebo,
        node_spawn_mouse,
        node_spawn_cat,
        node_ros_gz_bridge,
        delayed_nodes,
        toolbox_launch
    ])