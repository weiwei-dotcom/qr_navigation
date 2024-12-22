import os
 
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
 
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    param_pkg_dir = get_package_share_directory('cart_localization')
    param_file_name = 'burger.yaml'
    rviz_pkg_dir = get_package_share_directory('cart_localization')
    rviz_file_name = 'nav2_default_view.rviz'
    bringup_launch_pkg_dir = get_package_share_directory('cart_localization')
    bringup_launch_file_name = 'bringup_launch_cartographer.launch.py'
    state_publisher_launch_pkg_dir = get_package_share_directory('cart_localization')
    state_publisher_launch_file_name = 'robot_state_publisher.launch.py'
    urdf_pkg_dir = get_package_share_directory('cart_localization')
    urdf_file_name = 'mybot.urdf'
    
    urdf = LaunchConfiguration(
        'urdf', 
        default=os.path.join(
            urdf_pkg_dir,
            'urdf',
            urdf_file_name
        )
    )

    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            param_pkg_dir,
            'param',
            param_file_name))
 
    rviz_config_dir = os.path.join(
        rviz_pkg_dir,
        'rviz',
        rviz_file_name)
 
    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            os.path.join(
                state_publisher_launch_pkg_dir,
                'launch',
                state_publisher_launch_file_name
                )
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'urdf': urdf}.items(),
        ),
 
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
            os.path.join(
                bringup_launch_pkg_dir,
                'launch',
                bringup_launch_file_name
                )
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        ),
 
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'),
    ])
 