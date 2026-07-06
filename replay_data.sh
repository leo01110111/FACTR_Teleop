#!/usr/bin/env bash
# Replay a recorded episode onto a real UR7e follower arm (bc/replay_traj.py).
#
# Streams the follower joint positions recorded on /ur/<side>/obs_ur_state back
# through the UR ExternalControl URCap with servoJ, reproducing the demonstrated
# motion. The Robotiq gripper follows the recorded /ur/<side>/obs_gripper track.
#
# The UR ExternalControl program must be running (press PLAY on the pendant) and
# the arm should be near the episode's first pose before starting.
#
# SAFETY: this moves a real arm along recorded data. Keep the pendant e-stop
# within reach. If the arm starts far from the episode's first pose, the tool
# jogs there slowly (after an ENTER confirmation) and then pauses for ENTER
# again before replaying.
#
# Usage: ./replay_data.sh [dataset_name] [episode_index] [extra replay args...]
#        ./replay_data.sh test 0
#        ./replay_data.sh test 0 --config-file ur7e_leader_right.yaml
#        ./replay_data.sh test 0 --rate-scale 0.5   # half speed
set -e

DATASET="${1:-test}"
EPISODE="${2:-0}"
shift || true
shift || true

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO"

# conda's base env breaks ROS 2 rclpy; leave it before sourcing the overlay.
if command -v conda >/dev/null 2>&1 && [ -n "${CONDA_DEFAULT_ENV:-}" ]; then
    conda deactivate || true
fi

source ./factr_env

# --config-file defaults to ur7e_leader_left.yaml inside replay_traj; pass a
# different one via the extra args (e.g. --config-file ur7e_leader_right.yaml).
python -m bc.replay_traj --dataset "${DATASET}" --episode "${EPISODE}" "$@"
