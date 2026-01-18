import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('pac_mouse_pkg')

    # 1. PROCESS THE URDF
    xacro_file = os.path.join(pkg_share, 'urdf', 'mouse.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_desc_xml = doc.toxml()

    # 2. CONFIG FILES
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'mouse_view.rviz')
    world_file = os.path.join(pkg_share, 'worlds', 'maze.sdf') 
    gazebo_ros_pkg = get_package_share_directory('ros_gz_sim')

    # ========================================================================
    # GROUP 1: INFRASTRUCTURE (Start these immediately)
    # ========================================================================
    
    # A. Gazebo Simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # B. Spawn the Robot in Gazebo
    node_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'squeak_mouse',
            '-z', '0.1'
        ],
        output='screen'
    )

    # C. The Bridge (CRITICAL: Needs to be running for Clock)
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Bridge Joint States for the wheels
            '/world/maze/model/squeak_mouse/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model'
        ],
        remappings=[
            ('/world/maze/model/squeak_mouse/joint_state', '/joint_states')
        ],
        output='screen'
    )

    # ========================================================================
    # GROUP 2: ROBOT BRAIN & VISUALS (Delay these by 5 seconds)
    # ========================================================================

    # D. Robot State Publisher (Publishes the "Body" of the robot)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc_xml,
            'use_sim_time': True  # Essential for syncing
        }]
    )

    # E. RViz (The Visualizer)
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}] # Essential for syncing
    )

    # F. TF Fix (Connects Lidar)
    node_tf_fix = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'squeak_mouse/base_link/lidar'],
        parameters=[{'use_sim_time': True}], # Essential for syncing
        output='screen'
    )

    # Create a Timer to launch Group 2 after 5 seconds
    delayed_nodes = TimerAction(
        period=5.0, 
        actions=[node_robot_state_publisher, node_rviz, node_tf_fix]
    )

    return LaunchDescription([
        gazebo,
        node_spawn,
        node_ros_gz_bridge,
        delayed_nodes # Everything else launches later
    ])