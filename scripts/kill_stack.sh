#!/bin/bash
# Kill every AUV stack process, verify the graph is empty.
pkill -9 -f "install/auv_"
pkill -9 -f joy_node
pkill -9 -f mavros
pkill -9 -f rosbag2
sleep 3
ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1
echo "--- remaining nodes (must be empty): ---"
ros2 node list
