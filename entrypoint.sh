#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source /root/ros2_ws/install/setup.bash
source /opt/replay_ws/install/setup.bash
exec python3 -B /app/run.py "$@"
