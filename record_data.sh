#!/usr/bin/env bash
# Record teleop observation data as a ROS node (bc/data_record.py).
#
# Also launches the RealSense camera node so its color stream is recorded
# alongside the state topics. Subscribes to the observation topics listed in
# src/bc/bc/topics.md and saves episodes to raw_data/<dataset_name>/ as
# ep_00000.pkl, ep_00001.pkl, ...
#
# Controls (keyboard requires THIS terminal focused; foot pedals work anywhere):
#   SPACE / left  pedal - start / stop an episode (the episode is saved when you stop)
#   DELETE / right pedal - delete the most recently saved episode
#
# Foot pedal (PCsensor FootSwitch): the first run asks you to press the LEFT
# then RIGHT pedal to calibrate, saving the mapping to raw_data/.foot_pedal.json.
# Requires read access to the pedal: add this user to the 'input' group once with
#   sudo usermod -aG input "$USER"   (then log out and back in).
#
# Usage: ./record_data.sh [dataset_name]      (default: test)
set -e

DATASET="${1:-test}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO"

# conda's base env breaks ROS 2 rclpy; leave it before sourcing the overlay.
if command -v conda >/dev/null 2>&1 && [ -n "${CONDA_DEFAULT_ENV:-}" ]; then
    conda deactivate || true
fi

source ./factr_env

# Start the RealSense camera node in the background (config: src/cameras/configs/realsense.yaml).
# It publishes the color stream on /realsense/realsense/im, which data_record subscribes to below.
ros2 run cameras realsense --ros-args -p config_file:=realsense.yaml &
CAMERA_PID=$!

# Stop the camera node whenever this script exits (normal exit, Ctrl-C, or error).
cleanup() {
    kill "${CAMERA_PID}" 2>/dev/null || true
    wait "${CAMERA_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Observation topics from src/bc/bc/topics.md.
#   state_topics -> JointState observations
#   image_topics -> camera streams (sensor_msgs/Image)
ros2 run bc data_record --ros-args \
    -p state_topics:="['/ur/left/obs_ur_state', '/ur/left/obs_gripper', '/ur/left/obs_ur_wrench']" \
    -p image_topics:="['/realsense/realsense/im']" \
    -p dataset_name:="${DATASET}"
