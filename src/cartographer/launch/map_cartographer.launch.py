import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 定位到功能包的地址
    lua_config_pkg_dir = get_package_share_directory('cartographer')

    rviz_pkg_dir = get_package_share_directory('cartographer')
    rviz_file_name = 'nav2_default_view.rviz'

    rs_scan_pkg_dir = get_package_share_directory('laser_pub_pkg')
    rs_scan_file_name = 'rs_scan_pub.launch.py'

    #=====================运行节点需要的配置=======================================================================
    # 是否使用仿真时间，我们用gazebo，这里设置成true
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # 地图的分辨率
    resolution = LaunchConfiguration('resolution', default='0.05')
    # 地图的发布周期
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='0.3')
    # 配置文件夹路径
    configuration_directory = LaunchConfiguration('configuration_directory',default= os.path.join(lua_config_pkg_dir, 'config') )
    # 配置文件
    configuration_basename = LaunchConfiguration('configuration_basename', default='map.lua')
    
    #=====================声明三个节点，cartographer/occupancy_grid_node/rviz_node=================================
    rs_pub_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                rs_scan_pkg_dir,
                "launch",
                rs_scan_file_name
            )
        )
    )
    
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-configuration_directory', configuration_directory,
                   '-configuration_basename', configuration_basename])

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution', resolution, 
            '-publish_period_sec', publish_period_sec
            ]
        )
    
    rviz_config_dir = os.path.join(
        rviz_pkg_dir,
        'rviz',
        rviz_file_name)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    #===============================================定义启动文件========================================================
    ld = LaunchDescription()
    ld.add_action(rs_pub_launch)
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    ld.add_action(rviz)

    return ld