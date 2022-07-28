#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --
source "$ROS_WS/install/setup.bash" --
ros2 service call /execute_job demo_interfaces/srv/ExecuteJob "{rc_path: '/root/robot_config.yaml', pc_path: '/root/protocol_config.yaml'}"
exec "$@"
