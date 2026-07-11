#!/usr/bin/env bash
# Run a policy eval (rollout) on the left UR7e arm.
#
# Brings up the full on-robot inference stack via launch/rollout.py:
#   * two RealSense camera nodes (left = D455, top = D435), matching the streams
#     recorded during data collection;
#   * factr_teleop_ur7e_rollout: subscribes to the policy command topics and
#     servos the follower UR7e over ur_rtde directTorque (no Dynamixel leader);
#   * bc/policy_rollout: loads the checkpoint, runs inference, publishes the
#     command topics. save_data is enabled in the launch file.
#
# Order of operations (see launch/rollout.py header for the full rationale):
#   1. On the pendant, load + Play the ExternalControl program for the LEFT arm.
#   2. Jog the UR to initial_match_joint_pos FIRST, or the rollout node refuses
#      to start (>0.5 rad/joint away aborts, to avoid a jump):
#        ./return_init.sh --left        (requires Remote Control on the pendant)
#   3. Run THIS script. The arm holds its pose until the first policy command
#      arrives, then tracks the policy. Ctrl-C zeros torque and releases cleanly.
#
# Usage: ./run_eval_left.sh [data_dir] [config_file]
#   data_dir     absolute path to the trained checkpoint dir (latest_ckpt.ckpt,
#                agent_config.yaml, rollout_config.yaml).
#                default: /home/leo/FACTR/checkpoints/test/rollout
#   config_file  factr_teleop follower config.  default: ur7e_leader_left.yaml
set -e

DATA_DIR="${1:-/home/leo/FACTR/checkpoints/test/rollout}"
CONFIG="${2:-ur7e_leader_left.yaml}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO"

# conda's base env breaks ROS 2 rclpy; leave it before sourcing the overlay.
if command -v conda >/dev/null 2>&1 && [ -n "${CONDA_DEFAULT_ENV:-}" ]; then
    conda deactivate || true
fi

if [ ! -d "$DATA_DIR" ]; then
    echo "[run_eval_left] ERROR: data_dir does not exist: $DATA_DIR" >&2
    echo "  Pass the checkpoint dir: ./run_eval_left.sh /path/to/checkpoint/rollout" >&2
    exit 1
fi

echo "[run_eval_left] checkpoint : $DATA_DIR"
echo "[run_eval_left] config     : $CONFIG"
echo "[run_eval_left] Reminder: ExternalControl PLAYING on the pendant, and the arm"
echo "                jogged to initial_match_joint_pos (./return_init.sh --left)."

source ./factr_env
ros2 launch launch/rollout.py config_file:="${CONFIG}" data_dir:="${DATA_DIR}"
