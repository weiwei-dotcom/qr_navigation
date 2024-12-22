from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import PushRosNamespace
from launch.conditions import IfCondition
import os


def generate_launch_description():

    # rs_scan_pkg_dir = get_package_share_directory('cart_localization')
    rs_scan_pkg_dir = get_package_share_directory('laser_pub_pkg')
    rs_scan_file_name = 'rs_scan_pub.launch.py'
    cart_localization_pkg_dir = get_package_share_directory('cart_localization')
    cart_localization_file_name = 'cartographer_localization.launch.py'

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                rs_scan_pkg_dir,
                "launch",
                rs_scan_file_name
            ))
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                cart_localization_pkg_dir,
                "launch",
                cart_localization_file_name
            ))
        ),
    ])