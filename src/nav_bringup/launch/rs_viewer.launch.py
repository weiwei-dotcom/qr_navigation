from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os


def generate_launch_description():
    rs_pub_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('nav_bringup'), 'launch', 'rs_scan_pub.launch.py')
        ])
    )
    rviz_config=get_package_share_directory('rslidar_sdk')+'/rviz/rviz_qr.rviz'
    rviz_node = Node(package='rviz2', executable='rviz2', arguments=['-d',rviz_config])
    ld = LaunchDescription()
    ld.add_action(rs_pub_node)
    ld.add_action(rviz_node)
    return ld