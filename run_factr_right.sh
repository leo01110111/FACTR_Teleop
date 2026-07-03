#!/usr/bin/env bash
# Launch FACTR teleop for the left UR7e arm.
#
# Order reminder: run THIS first (it opens the External Control listener and
# waits up to 60 s), THEN press Play on the External Control program on the
# pendant. Jog the UR to initial_match_joint_pos before launching or the
# start-config check aborts.
#
# Usage: ./run_factr_left.sh [config_file]   (default: ur7e_leader_left.yaml)
set -e

CONFIG="${1:-ur7e_leader_right.yaml}"
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO"

# conda's base env breaks ROS 2 rclpy; leave it before sourcing the overlay.
if command -v conda >/dev/null 2>&1 && [ -n "${CONDA_DEFAULT_ENV:-}" ]; then
    conda deactivate || true
fi

# The Dynamixel U2D2 needs the dialout group. This user is a permanent member
# (sudo usermod -aG dialout "$USER" + relogin), so we run directly -- no
# sg dialout wrapper needed. Warn if somehow not in the group.
if ! id -nG | tr ' ' '\n' | grep -qx dialout; then
    echo "[run_factr_left] WARNING: not in 'dialout' group; the U2D2 open may fail."
    echo "  Fix: sudo usermod -aG dialout \"\$USER\" then log out and back in."
fi

source ./factr_env
ros2 launch launch/factr_teleop_ur7e.py config_file:="${CONFIG}"
