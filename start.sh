source /opt/ros/foxy/setup.bash
source ./install/setup.bash

export TURTLEBOT3_MODEL=waffle

if [ "$1" == "cart" ]; then
    if [ "$2" == "map" ]; then
        ros2 launch cartographer map_cartographer.launch.py
    elif [ "$2" == 'nav' ]; then
        ros2 launch cartographer system.launch.py
    fi

elif [ "$1" == "amcl" ]; then
    echo "AMCL navigation method is under developing ! ! !"

elif [ "$1" == "ros" ]; then
    killall gzserver
    killall gzclient
    ros2 launch nav_bringup ros_nav.launch.py

else
    echo "please input correct bootargs ! ! !"

fi