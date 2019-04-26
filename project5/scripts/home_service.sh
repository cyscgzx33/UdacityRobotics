#!/bin/sh
xterm -e " roslaunch turtlebot_gazebo yusen_world.launch " &
sleep 5

xterm -e " roslaunch turtlebot_gazebo amcl_demo_yusen.launch " &
sleep 5

xterm -e " roslaunch turtlebot_rviz_launchers view_navigation_yusen.launch " &
sleep 5

xterm -e " roslaunch pick_objects double_objects_picking.launch " &
sleep 5

xterm -e " rosrun add_markers add_markers_odom"
