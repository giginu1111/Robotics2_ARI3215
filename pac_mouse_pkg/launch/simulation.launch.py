import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('pac_mouse_pkg')

    # 1. SETUP MODEL PATH FOR GAZEBO
    # This ensures Gazebo can find your mesh files if you have any
    set_model_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
        value=os.path.join(pkg_share, 'models')
    )

    # 2. PROCESS XACRO FILES
    # We pass the 'robot_name' argument to the Xacro file. 
    # This creates the "mouse/base_link" and "cat/base_link" frames.
    
    # Process Mouse
    mouse_file = os.path.join(pkg_share, 'urdf', 'mouse.urdf.xacro')
    mouse_doc = xacro.process_file(mouse_file, mappings={'robot_name': 'mouse'})
    mouse_xml = mouse_doc.toxml()

    # Process Cat
    cat_file = os.path.join(pkg_share, 'urdf', 'cat.urdf.xacro')
    cat_doc = xacro.process_file(cat_file, mappings={'robot_name': 'cat'})
    cat_xml = cat_doc.toxml()

    # 3. LAUNCH GAZEBO WORLD
    world_file = os.path.join(pkg_share, 'worlds', 'maze.sdf') 
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 4. SPAWN ROBOTS
    # We use simple names 'mouse' and 'cat' to match the topics in the URDF 
    # (e.g. /model/mouse/cmd_vel)
    
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

    # 5. ROBOT STATE PUBLISHERS
    # These publish the static transforms (like wheel -> base_link).
    # Since our URDFs already have namespaced links (mouse/base_link), we don't need 'frame_prefix'.
    # We run them in their own namespaces to keep topics clean.
    
    mouse_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='mouse', 
        parameters=[{'use_sim_time': True, 'robot_description': mouse_xml}],
        output='screen'
    )

    cat_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='cat',
        parameters=[{'use_sim_time': True, 'robot_description': cat_xml}],
        output='screen'
    )

    # 6. ROS GZ BRIDGE
    # This is the most critical part. It maps Gazebo topics to ROS topics.
    # We bridge the TF topics separately to merge them into the global /tf.
    
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # --- GLOBAL ---
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            # --- MOUSE BRIDGE ---
            '/model/mouse/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/mouse/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/mouse/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/mouse/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/mouse/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/model/mouse/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/mouse/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',

            # --- CAT BRIDGE ---
            '/model/cat/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/cat/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/cat/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/cat/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/model/cat/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/cat/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/model/cat/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            # MOUSE REMAPS
            ('/model/mouse/cmd_vel', '/mouse/cmd_vel'),
            ('/model/mouse/odometry', '/mouse/odom'),
            ('/model/mouse/scan', '/mouse/scan'),
            ('/model/mouse/tf', '/tf'), # Merge into global TF tree
            ('/model/mouse/imu', '/mouse/imu'),
            ('/model/mouse/camera/image_raw', '/mouse/camera/image_raw'),
            ('/model/mouse/joint_states', '/mouse/joint_states'),

            # CAT REMAPS
            ('/model/cat/cmd_vel', '/cat/cmd_vel'),
            ('/model/cat/odometry', '/cat/odom'),
            ('/model/cat/scan', '/cat/scan'),
            ('/model/cat/tf', '/tf'),   # Merge into global TF tree
            ('/model/cat/imu', '/cat/imu'),
            ('/model/cat/camera/image_raw', '/cat/camera/image_raw'),
            ('/model/cat/joint_states', '/cat/joint_states'),
        ],
        output='screen'
    )

    # 7. EKF NODE (For the Mouse)
    # We override the frames here so we don't need to edit the ekf.yaml file manually
    ekf_config_file = os.path.join(pkg_share, 'config', 'ekf.yaml')
    
    mouse_ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        namespace='mouse',
        output='screen',
        parameters=[
            ekf_config_file, 
            {
                'use_sim_time': True,
                'map_frame': 'map',
                'odom_frame': 'mouse/odom',
                'base_link_frame': 'mouse/base_link',
                'world_frame': 'mouse/odom',
                'odom0': '/mouse/odom',
                'imu0': '/mouse/imu',
                'publish_tf': False 
            }
        ]
    )

    # 8. RVIZ
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'mouse_view.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}]
    )

    # 9. DELAYED ACTIONS
    # We delay the EKF and RVIZ slightly to ensure Gazebo is up and publishing /clock
    delayed_nodes = TimerAction(
        period=5.0, 
        actions=[
            mouse_ekf,
            rviz
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
        delayed_nodes
    ])