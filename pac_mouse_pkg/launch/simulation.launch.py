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
            '-topic', '/robot_description', 
            '-name', 'squeak_mouse', 
            '-x', '-3.0',  # Move back 3 meters
            '-y', '-3.0',  # Move left 3 meters (Safe corner)
            '-z', '0.1',   # Lift up slightly
            '-Y', '0.0'    # Face East (0.0 radians)
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

    # 4. ROBOT STATE PUBLISHER (MOUSE)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='mouse_state_publisher', # Rename for clarity
        output='screen',
        parameters=[{
            'robot_description': mouse_desc_xml,
            'use_sim_time': True 
        }],
        # Remap to a namespace so RViz distinguishes them
        namespace='mouse' 
    )

    # 4.5 ROBOT STATE PUBLISHER (CAT)
    node_cat_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='cat_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': cat_desc_xml,
            'use_sim_time': True,
            'frame_prefix': 'doraemon/' # Optional: prefixes TF frames if needed
        }],
        namespace='cat'
    )

    # 5. JOINT STATE PUBLISHER
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
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
    
    # A. Joint State Publisher (Keeps the wheels attached to the body)
    node_cat_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='cat_joint_state_publisher',
        namespace='cat', # Must match the namespace of the state publisher
        parameters=[{'use_sim_time': True}]
    )

    # B. TF: Connect the Lidar Data to the Robot
    # The Robot State Publisher creates "doraemon/lidar_link"
    # The Gazebo Plugin outputs data in "lidar_link"
    # This node connects them.
    node_cat_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cat_tf_lidar_fix',
        # Arguments: x y z qx qy qz qw parent_frame child_frame
        arguments=['0', '0', '0', '0', '0', '0', '1', 'doraemon/lidar_link', 'lidar_link'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # C. TF: Connect the Camera Data (Optional but good)
    node_cat_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='cat_tf_camera_fix',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'doraemon/camera_link', 'camera_link_optical'],
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
            node_robot_state_publisher, 
            node_joint_state_publisher, 
            node_tf_lidar, 
            node_tf_base, 
            node_ekf,

            # --- CAT NODES ---
            node_cat_state_publisher,
            node_cat_joint_state_publisher,
            node_cat_tf_lidar,
            node_cat_tf_camera,

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