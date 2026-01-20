import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('pac_mouse_pkg')

    # Resources
    mouse_xacro = os.path.join(pkg_share, 'urdf', 'mouse.urdf.xacro')
    mouse_doc = xacro.process_file(mouse_xacro)
    mouse_desc = mouse_doc.toxml()

    cat_xacro = os.path.join(pkg_share, 'urdf', 'cat.urdf.xacro')
    cat_doc = xacro.process_file(cat_xacro)
    cat_desc = cat_doc.toxml()

    world_file = os.path.join(pkg_share, 'worlds', 'pac_mouse_maze.sdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'pac_mouse_dual.rviz')
    gazebo_ros_pkg = get_package_share_directory('ros_gz_sim')

    # 1. GAZEBO
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # 2. SPAWN MOUSE (using -string argument to pass description directly)
    spawn_mouse = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_mouse',
        arguments=[
            '-string', mouse_desc,
            '-name', 'mouse_robot',
            '-x', '-4.5',
            '-y', '-4.5',
            '-z', '0.1',
            '-Y', '0.785'
        ],
        output='screen'
    )

    # 3. SPAWN CAT (using -string argument to pass description directly)
    spawn_cat = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_cat',
        arguments=[
            '-string', cat_desc,
            '-name', 'cat_robot',
            '-x', '4.5',
            '-y', '4.5',
            '-z', '0.1',
            '-Y', '-2.356'
        ],
        output='screen'
    )

    # 4. BRIDGE FOR MOUSE
    bridge_mouse = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_mouse',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/mouse/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/mouse/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/mouse/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/mouse/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/mouse/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen'
    )

    # 5. BRIDGE FOR CAT
    bridge_cat = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_cat',
        arguments=[
            '/cat/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/cat/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/cat/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/cat/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/cat/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen'
    )

    # 6. ROBOT STATE PUBLISHERS
    mouse_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='mouse_robot_state_publisher',
        namespace='mouse',
        parameters=[{
            'robot_description': mouse_desc,
            'use_sim_time': True
        }]
    )

    cat_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='cat_robot_state_publisher',
        namespace='cat',
        parameters=[{
            'robot_description': cat_desc,
            'use_sim_time': True
        }]
    )

    # 7. CHEESE MANAGER
    cheese_manager = Node(
        package='pac_mouse_pkg',
        executable='cheese_manager',
        name='cheese_manager',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 8. SMART MOUSE CONTROLLER
    smart_mouse = Node(
        package='pac_mouse_pkg',
        executable='smart_mouse_final',
        name='smart_mouse',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 9. SMART CAT CONTROLLER
    smart_cat = Node(
        package='pac_mouse_pkg',
        executable='smart_cat',
        name='smart_cat',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 10. RVIZ
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    # DELAYED NODES (spawn robots immediately, others after delay)
    delayed_nodes = TimerAction(
        period=3.0,
        actions=[
            mouse_state_pub,
            cat_state_pub,
            cheese_manager,
            smart_mouse,
            smart_cat,
            rviz
        ]
    )

    return LaunchDescription([
        gazebo,
        spawn_mouse,
        spawn_cat,
        bridge_mouse,
        bridge_cat,
        delayed_nodes
    ])