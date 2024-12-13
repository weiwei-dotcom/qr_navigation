source /opt/ros/foxy/setup.bash
source ./install/setup.bash

killall gzserver
killall gzclient

export TURTLEBOT3_MODEL=waffle
ros2 launch nav_bringup ros_default_nav.launch.py slam:=False