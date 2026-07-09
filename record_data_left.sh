#!/usr/bin/env bash
# Record teleop observation data as a ROS node (bc/data_record.py).
#
# Also launches the RealSense camera nodes (left = D455, top = D435) so their
# color streams are recorded alongside the state topics. Subscribes to the
# observation topics listed in src/bc/bc/topics.md and saves episodes to
# raw_data/<dataset_name>/ as ep_00000.pkl, ep_00001.pkl, ...
#
# Controls (keyboard requires THIS terminal focused; foot pedals work anywhere):
#   SPACE / left  pedal - start / stop an episode (the episode is saved when you stop)
#   DELETE / right pedal - delete the most recently saved episode
#
# Foot pedal (PCsensor FootSwitch): left pedal = start/stop, right pedal = delete
# (fixed keycodes, no calibration). Requires read access to the pedal: add this
# user to the 'input' group once with
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

# Start both RealSense camera nodes in the background. Each publishes its color
# stream on /realsense/<name>/im (and depth on /realsense/<name>/depth), which
# data_record subscribes to below. Distinct node names avoid a name collision.
#   realsense_left_cam.yaml -> D455, name "left" -> /realsense/left/im
#   top_cam.yaml            -> D435, name "top"  -> /realsense/top/im
ros2 run cameras realsense --ros-args -p config_file:=realsense_left_cam.yaml -r __node:=realsense_left &
CAMERA_PIDS=($!)
ros2 run cameras realsense --ros-args -p config_file:=top_cam.yaml -r __node:=realsense_top &
CAMERA_PIDS+=($!)

# Stop the camera nodes whenever this script exits (normal exit, Ctrl-C, or error).
cleanup() {
    for pid in "${CAMERA_PIDS[@]}"; do
        kill "${pid}" 2>/dev/null || true
        wait "${pid}" 2>/dev/null || true
    done
}
trap cleanup EXIT INT TERM

# Observation topics from src/bc/bc/topics.md.
#   state_topics -> JointState observations
#   image_topics -> camera color streams (sensor_msgs/Image)
ros2 run bc data_record --ros-args \
    -p state_topics:="['/ur/left/obs_ur_state', '/ur/left/obs_gripper', '/ur/left/obs_ur_wrench']" \
    -p image_topics:="['/realsense/left/im', '/realsense/top/im']" \
    -p dataset_name:="${DATASET}"
