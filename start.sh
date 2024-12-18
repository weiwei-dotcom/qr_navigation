source /opt/ros/foxy/setup.bash
source ./install/setup.bash

killall gzserver
killall gzclient

export TURTLEBOT3_MODEL=waffle


if [ "$1" == 'yd' ]; then
    if [ "$2" == 'cart' ]; then
        ./yd_pub.sh
        ros2 launch nav_bringup cartographer.launch.py
    elif [ "$2" == 'amcl' ]; then
        ./yd_pub.sh
        ros2 launch nav_bringup qr_nav_yd.launch.py
    elif [ "$2" == 'pub' ]; then
        ./yd_pub.sh
        ros2 launch nav_bringup yd_viewer.launch.py
    fi
elif [ "$1" == "rs" ]; then
    if [ "$2" == "cart" ]; then
        ros2 launch nav_bringup cartographer_rs.launch.py
    elif [ "$2" == "amcl" ]; then
        ros2 launch nav_bringup qr_nav_rs.launch.py
    elif [ "$2" == 'pub' ]; then
        ros2 launch nav_bringup rs_viewer.launch.py
    elif [ "$2" == 'gen_odom' ]; then
        ros2 launch nav_bringup rs_cart_gen_odom.launch.py
    fi
elif [ "$1" == "ros" ]; then
    ros2 launch nav_bringup ros_nav.launch.py
else
    echo "please input correct bootargs ! ! !"
fi