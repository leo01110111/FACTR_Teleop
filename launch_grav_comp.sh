#!/usr/bin/env bash
# Launch the FACTR gravity-comp demo under the venv python (where dynamixel_sdk/zmq live)
# with ROS Jazzy + workspace overlay. Run via: sg dialout -c '.../launch_grav_comp.sh [config.yaml]'
set -e
cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
exec python install/factr_teleop/lib/factr_teleop/factr_teleop_grav_comp_demo \
  --ros-args -p config_file:="${1:-ur7e_leader_right.yaml}"
