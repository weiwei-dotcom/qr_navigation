from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import PushRosNamespace
from launch.conditions import IfCondition
import os


def generate_launch_description():

    slam = LaunchConfiguration('slam')

    return LaunchDescription([
        DeclareLaunchArgument(
            'slam',
            default_value='True',
            description='Whether run a SLAM'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("nav_bringup"),
                "launch",
                "turtlebot3_world.launch.py"
            )),
            launch_arguments={'use_sim_time': 'True'}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("nav_bringup"),
                "launch",
                "navigation2.launch.py"
            )),
            launch_arguments={'slam': slam}.items()
        )
    ])