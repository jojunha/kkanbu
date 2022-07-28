## /bin/bash

source ~/git/Kkanbu/devel/setup.bash

roslaunch realsense2_camera rs_camera.launch &

sleep 2 &

roslaunch darknet_ros darknet_ros.launch &

sleep 2 &

roslaunch rplidar_ros rplidar_a3.launch &

sleep 2 &

roslaunch obstacle_detector nodelets.launch &

sleep 2 &

roslaunch kkanbu_sensor kkanbu_sensor.launch 

sleep 2 &

roslaunch ntrip_ros ntrip_ros.launch 

sleep 2 &

roslaunch ublox_gps ublox_device.launch

sleep 2 &

rosrun myahrs_driver myahrs_driver _port:=/dev/ttyACM1
