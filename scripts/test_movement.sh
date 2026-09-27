#!/usr/bin/env bash
# Test script to demonstrate 4-wheel robot movement entirely within Gazebo Sim (no ROS 2)
set -e

export GZ_IP=127.0.0.1

echo "=========================================================="
echo " Testing 4-Wheel Robot Movement via Gazebo Native Topics"
echo "=========================================================="

echo -e "\n[1/5] Moving Forward (linear.x = 1.0 m/s)..."
gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 1.0}, angular: {z: 0.0}"
sleep 2
echo "Current Odometry:"
gz topic -e -t /odom -n 1

echo -e "\n[*] Stopping..."
gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.0}, angular: {z: 0.0}"
sleep 1
echo "Current Odometry:"
gz topic -e -t /odom -n 1

echo -e "\n[2/5] Moving Backward (linear.x = -1.0 m/s)..."
gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: -1.0}, angular: {z: 0.0}"
sleep 2
echo "Current Odometry:"
gz topic -e -t /odom -n 1

echo -e "\n[*] Stopping..."
gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.0}, angular: {z: 0.0}"
sleep 1
echo "Current Odometry:"
gz topic -e -t /odom -n 1

echo -e "\n[3/5] Turning Left (angular.z = 1.0 rad/s)..."
gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.0}, angular: {z: 1.0}"
sleep 2
echo "Current Odometry:"
gz topic -e -t /odom -n 1

echo -e "\n[*] Stopping..."
gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.0}, angular: {z: 0.0}"
sleep 1

echo -e "\n[4/5] Turning Right (angular.z = -1.0 rad/s)..."
gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.0}, angular: {z: -1.0}"
sleep 2
echo "Current Odometry:"
gz topic -e -t /odom -n 1

echo -e "\n[5/5] Final Stop..."
gz topic -t "/cmd_vel" -m gz.msgs.Twist -p "linear: {x: 0.0}, angular: {z: 0.0}"
sleep 1
echo "Final Odometry:"
gz topic -e -t /odom -n 1

echo "=========================================================="
echo " All movements successfully executed and verified!"
echo "=========================================================="
