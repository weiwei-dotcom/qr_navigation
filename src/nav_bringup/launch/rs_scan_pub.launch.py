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

    config_file = ''

    lidar_node = Node(package='rslidar_sdk', 
                      executable='rslidar_sdk_node', 
                      output='screen', parameters=[{'config_path': config_file}])

    pcl2scan_node = Node(
        package='pointcloud_to_laserscan', node_executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', '/rslidar_points'),
                    ('scan', '/scan')],
        parameters=[{
            'target_frame': 'base_scan',
            'transform_tolerance': 0.01,
            'min_height': -0.2,
            'max_height': 0.2,
            'angle_min': -3.1416,  # -M_PI/2
            'angle_max': 3.1416,  # M_PI/2
            'angle_increment': 0.0087,  # M_PI/360.0
            'scan_time': 0.02,
            'range_min': 0.2,
            'range_max': 30.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        node_name='pointcloud_to_laserscan'
    )
    return LaunchDescription([
        lidar_node,
        pcl2scan_node
    ])