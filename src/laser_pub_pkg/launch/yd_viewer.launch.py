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
    rviz_config_pkg_dir = get_package_share_directory('laser_pub_pkg')
    rviz_config_file_name = 'rviz_laser.rviz'
    rviz_node = Node(
        package='rviz2', 
        executable='rviz2', 
        arguments=[
            '-d', 
            os.path.join(
                rviz_config_pkg_dir, 
                'rviz', 
                rviz_config_file_name
            )
        ]
    )
    return LaunchDescription([
        rviz_node
    ])