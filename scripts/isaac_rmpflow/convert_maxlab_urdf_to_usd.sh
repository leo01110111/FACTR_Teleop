#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="${REPO_DIR:-/home/srianumakonda/FACTR_Teleop}"
ISAACLAB_DIR="${ISAACLAB_DIR:-/home/srianumakonda/IsaacLab}"
ISAAC_CONDA_ENV="${ISAAC_CONDA_ENV:-env_isaaclab}"
URDF_PATH="${URDF_PATH:-$REPO_DIR/configs/isaac_rmpflow/maxlab_ur7e_right/maxlab_ur7e_right.urdf}"
USD_PATH="${USD_PATH:-$REPO_DIR/generated/isaac_rmpflow/maxlab_ur7e_right_primitive.usd}"

cd "$REPO_DIR"
python scripts/isaac_rmpflow/export_maxlab_urdf.py --output "$URDF_PATH"

ISAACLAB_DIR="$ISAACLAB_DIR" ISAAC_CONDA_ENV="$ISAAC_CONDA_ENV" \
  "$REPO_DIR/scripts/isaac_rmpflow/run_isaaclab.sh" \
  "$ISAACLAB_DIR/scripts/tools/convert_urdf.py" \
  "$URDF_PATH" \
  "$USD_PATH" \
  --fix-base \
  --joint-target-type none \
  --headless

echo "Generated primitive UR7e USD: $USD_PATH"
