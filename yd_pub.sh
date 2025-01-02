sudo chmod 777 /dev/ttyUSB0
gnome-terminal -- bash -c "echo 'start single line lidar!'; source /opt/ros/noetic/setup.bash; cd /home/wl/Documents/ydlidar_ros_driver; source devel/setup.bash; roslaunch ydlidar_ros_driver X2.launch; exec bash"
sleep 3
gnome-terminal -- bash -c "echo 'start ros1_bridge!'; source /opt/ros/foxy/setup.bash; source /opt/ros/noetic/setup.bash; ros2 run ros1_bridge dynamic_bridge; exec bash"
sleep 2
source /opt/ros/foxy/setup.bash
source ./install/setup.bash
ros2 launch laser_pub_pkg yd_viewer.launch.py