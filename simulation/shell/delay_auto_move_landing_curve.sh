#!/bin/bash

source ~/.bashrc

gnome-terminal --window -e 'bash -c "roslaunch simulation auto_move_landing_curve.launch; exec bash"' \
--tab -e 'bash -c "sleep 15; roslaunch aruco_ros single.launch; exec bash"' \
--tab -e 'bash -c "sleep 15; rosrun image_view image_view image:=/aruco_single/result; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch detector forward_delay_node.launch; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch detector feedback_delay_node.launch; exec bash"' \
--tab -e 'bash -c "sleep 10; rosrun uav_control uav_auto_takeoff_with_delay; exec bash"' \
--tab -e 'bash -c "sleep 5; roslaunch detector simulation_detector_move_curve_landing.launch;  exec bash"' 

