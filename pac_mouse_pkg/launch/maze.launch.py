import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('pac_mouse_pkg')

    # 1. SETUP RESOURCES (So Gazebo finds your models if you need them)
    set_model_path = AppendEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
        value=os.path.join(pkg_share, 'models')
    )

    world_file = os.path.join(pkg_share, 'worlds', 'maze.sdf') 
    start_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 3. BRIDGE (Optional: Keeps ROS happy if you leave it running)
    # We don't really need it just for building, but it prevents errors.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        set_model_path,
        start_gazebo,
        bridge
    ])