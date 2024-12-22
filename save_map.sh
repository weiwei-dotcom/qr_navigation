source /opt/ros/foxy/setup.bash
source ./install/setup.bash
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/wl/Documents/qr_navigation/src/cart_localization/map/map.pbstream'}"
