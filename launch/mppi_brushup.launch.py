from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    world_file_path = PathJoinSubstitution([
        get_package_share_directory('inrof_sim'),
        'worlds',
        'mppi_brush.sdf'
    ])
    sim = IncludeLaunchDescription(    
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
        launch_arguments=[
            ('gz_args', world_file_path)]
    )

    gz_bridge_node_wheel0 = Node(
        package='ros_gz_bridge',
        name='bridge_vel',
        executable='parameter_bridge',
        arguments=['/swerve/wheel_vel0@std_msgs/msg/Float64@ignition.msgs.Double']
    )

    gz_bridge_node_wheel1 = Node(
        package='ros_gz_bridge',
        name='bridge_vel',
        executable='parameter_bridge',
        arguments=['/swerve/wheel_vel1@std_msgs/msg/Float64@ignition.msgs.Double']
    )

    gz_bridge_node_wheel2 = Node(
        package='ros_gz_bridge',
        name='bridge_vel',
        executable='parameter_bridge',
        arguments=['/swerve/wheel_vel2@std_msgs/msg/Float64@ignition.msgs.Double']
    )

    gz_bridge_node_stare0 = Node(
        package='ros_gz_bridge',
        name='bridge_vel',
        executable='parameter_bridge',
        arguments=['/swerve/swerve_pos0@std_msgs/msg/Float64@ignition.msgs.Double']
    )

    gz_bridge_node_stare1 = Node(
        package='ros_gz_bridge',
        name='bridge_vel',
        executable='parameter_bridge',
        arguments=['/swerve/swerve_pos1@std_msgs/msg/Float64@ignition.msgs.Double']
    )

    gz_bridge_node_stare2 = Node(
        package='ros_gz_bridge',
        name='bridge_vel',
        executable='parameter_bridge',
        arguments=['/swerve/swerve_pos2@std_msgs/msg/Float64@ignition.msgs.Double']
    )

    gz_bridge_node_pose = Node(
        package='ros_gz_bridge',
        name='bridge_pose',
        executable='parameter_bridge',
        arguments=['/world/swerve_sim/pose/info@geometry_msgs/msg/PoseArray[ignition.msgs.Pose_V']
    )

    ld.add_action(sim)
    ld.add_action(gz_bridge_node_wheel0)
    ld.add_action(gz_bridge_node_wheel1)
    ld.add_action(gz_bridge_node_wheel2)
    ld.add_action(gz_bridge_node_stare0)
    ld.add_action(gz_bridge_node_stare1)
    ld.add_action(gz_bridge_node_stare2)
    ld.add_action(gz_bridge_node_pose)

    return ld