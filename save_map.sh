source /opt/ros/foxy/setup.bash
source ./install/setup.bash
if [ $# == 1 ]; then
    ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/wl/Documents/qr_navigation/src/cartographer/map/map_$1.pbstream'}"
else
    echo "please input the map serial number!!"
fi