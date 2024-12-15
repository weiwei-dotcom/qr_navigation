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

    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_nav = os.path.join(bringup_dir, 'launch')
    launch_tf = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')

    declared_args = []

    declared_args.append(DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'))
    
    declared_args.append(DeclareLaunchArgument(
        'slam',
        default_value='True', #todo:
        description='Whether run a SLAM'))
    declared_args.append(DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'))

    declared_args.append(DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'))

    declared_args.append(DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack'))

    declared_args.append(DeclareLaunchArgument(
        'default_bt_xml_filename',
        default_value=os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use'))

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')

    bringup_cmd_group = GroupAction([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_tf, '/robot_state_publisher.launch.py']),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_nav, 'slam_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file}.items()),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_nav, 'navigation_launch.py')),
            launch_arguments={'namespace': namespace,
                              'use_sim_time': use_sim_time,
                              'autostart': autostart,
                              'params_file': params_file,
                              'default_bt_xml_filename': default_bt_xml_filename,
                              'use_lifecycle_mgr': 'false',
                              'map_subscribe_transient_local': 'true'}.items()),
    ])

    declared_args.append(DeclareLaunchArgument(
        name='scan', 
        default_value='scan',
        description='Namespace for sample topics'))

    config_file = ''

    lidar_node = Node(package='rslidar_sdk', 
                      executable='rslidar_sdk_node', 
                      output='screen', parameters=[{'config_path': config_file}])
    
    rviz_config=get_package_share_directory('rslidar_sdk')+'/rviz/rviz_qr.rviz'

    rviz_node = Node(package='rviz2', executable='rviz2', arguments=['-d',rviz_config])

    pcl2scan_node = Node(
        package='pointcloud_to_laserscan', node_executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', '/rslidar_points'),
                    ('scan', '/scan')],
        parameters=[{
            'target_frame': 'base_scan',
            'transform_tolerance': 0.01,
            'min_height': -0.3,
            'max_height': 1.5,
            'angle_min': -3.14,  # -M_PI/2
            'angle_max': 3.14,  # M_PI/2
            'angle_increment': 0.0087,  # M_PI/360.0
            'scan_time': 0.3333,
            'range_min': 0.2,
            'range_max': 80.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        node_name='pointcloud_to_laserscan'
    )
    return LaunchDescription(declared_args + [
        bringup_cmd_group,
        lidar_node,
        # tf_node,
        pcl2scan_node,
        rviz_node
    ])