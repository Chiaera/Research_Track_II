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

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('bme_gazebo_sensors'), 'launch', 'spawn_robot_ex.launch.py')),
        launch_arguments={'rviz': 'false'}.items()
    )

    rviz_config_file = os.path.join(get_package_share_directory('robot_bringup'), 'config', 'start_configuration.rviz')    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_custom',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

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
            ),
            ComposableNode(
                package="pkg_cpp",
                plugin="robot_namespace::MoveRobotClientNode",
                name="move_robot_client"
            )
        ]
    )

    interface_node = Node(
        package="pkg_py",
        executable="interface",
        name="interface",
        output="screen",
        prefix="xterm -fa 'Monospace' -fs 14 -e"
    )

    ld.add_action(container)
    ld.add_action(gazebo)
    ld.add_action(interface_node)
    ld.add_action(rviz_node)

    return ld