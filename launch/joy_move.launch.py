from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    world_file_path = PathJoinSubstitution([
        get_package_share_directory('inrof_sim'),
        'worlds',
        'inrof_field.sdf'
    ])
    sim = IncludeLaunchDescription(    
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[
            ('gz_args', world_file_path)]
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        namespace='daisha'
    )

    joy_vel_converter = LifecycleNode(
        package='control_pkg',
        name='joy_vel_converter',
        executable='joy_vel_converter',
        namespace='daisha',
    )

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/daisha/cmd_vel_robot@geometry_msgs/msg/Twist@ignition.msgs.Twist']
    )

    ld.add_action(sim)
    ld.add_action(joy_node)
    ld.add_action(joy_vel_converter)
    ld.add_action(gz_bridge_node)

    return ld