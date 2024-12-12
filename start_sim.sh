source /opt/ros/foxy/setup.bash

export TURTLEBOT3_MODEL=waffle
ros2 launch nav_bringup ros_default_nav.launch.py $1:=$2