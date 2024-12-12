from launch import LaunchDescription
from launch.action import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    DeclareLaunchArgument(
        name='scanner', default_value='scanner',
        description='Namespace for sample topics'
    )

    rviz_config=get_package_share_directory('rslidar_sdk')+'/rviz/rviz2.rviz'
    config_file = '' # your config file path

    lidar_node = Node(namespace='rslidar_sdk', package='rslidar_sdk', executable='rslidar_sdk_node', output='screen', parameters=[{'config_path': config_file}])

    rviz_node = Node(namespace='rviz2', package='rviz2', executable='rviz2', arguments=['-d',rviz_config])

    pcl2scan_node = Node(
        package='pointcloud_to_laserscan', node_executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', [LaunchConfiguration(variable_name='scanner'), '/cloud']),
                    ('scan', [LaunchConfiguration(variable_name='scanner'), '/scan'])],
        parameters=[{
            'target_frame': 'cloud',
            'transform_tolerance': 0.01,
            'min_height': 0.0,
            'max_height': 1.0,
            'angle_min': -1.5708,  # -M_PI/2
            'angle_max': 1.5708,  # M_PI/2
            'angle_increment': 0.0087,  # M_PI/360.0
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 4.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        node_name='pointcloud_to_laserscan'
    )
    return LaunchDescription([
        lidar_node,
        rviz_node,
        pcl2scan_node
    ])