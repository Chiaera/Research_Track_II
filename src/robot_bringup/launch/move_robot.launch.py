from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()

    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('bme_gazebo_sensors'), 'launch', 'spawn_robot_ex.launch.py')))

    container = ComposableNodeContainer(
        name="my_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions= [
            ComposableNode(
                package="pkg_cpp",
                plugin="robot_namespace::MoveRobotServerNode",
                name="move_robot_server"
            )
        ]
    )

    move_robot_client = Node(
        package="pkg_cpp",
        executable="move_robot_client",
        name="move_robot_client"
    )

    ld.add_action(container)
    ld.add_action(move_robot_client)
    ld.add_action(gazebo)

    return ld