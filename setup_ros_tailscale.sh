#!/bin/bash
export CYCLONEDDS_URI=file://$PIXI_PROJECT_ROOT/cyclonedds.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_NETWORK_INTERFACE=tailscale0
export ROS_IP=$(tailscale ip -4)
export ROS_DOMAIN_ID=0
ros2 daemon stop
ros2 daemon start
echo "ROS2 environment configured for Tailscale communication"
