#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="${REPO_DIR:-/home/srianumakonda/FACTR_Teleop}"
ISAACLAB_DIR="${ISAACLAB_DIR:-/home/srianumakonda/IsaacLab}"
ISAAC_CONDA_ENV="${ISAAC_CONDA_ENV:-env_isaaclab}"
EXPORT_PYTHON="${EXPORT_PYTHON:-python}"
MJCF_PATH="${MJCF_PATH:-$REPO_DIR/generated/isaac_rmpflow/maxlab_dual_ur7e_table.xml}"
USD_PATH="${USD_PATH:-$REPO_DIR/generated/isaac_rmpflow/maxlab_dual_ur7e_table.usd}"
COLLISION_ONLY="${COLLISION_ONLY:-0}"

cd "$REPO_DIR"
EXPORT_ARGS=(--output "$MJCF_PATH")
case "$COLLISION_ONLY" in
  1|true|TRUE|yes|YES|on|ON)
    EXPORT_ARGS+=(--collision-only)
    ;;
  0|false|FALSE|no|NO|off|OFF|"")
    ;;
  *)
    echo "COLLISION_ONLY must be 1/0, true/false, yes/no, or on/off." >&2
    exit 2
    ;;
esac

"$EXPORT_PYTHON" scripts/isaac_rmpflow/export_maxlab_mjcf.py "${EXPORT_ARGS[@]}"

ISAACLAB_DIR="$ISAACLAB_DIR" ISAAC_CONDA_ENV="$ISAAC_CONDA_ENV" \
  "$REPO_DIR/scripts/isaac_rmpflow/run_isaaclab.sh" \
  "$REPO_DIR/scripts/isaac_rmpflow/convert_mjcf_enabled.py" \
  "$MJCF_PATH" \
  "$USD_PATH" \
  --fix-base \
  --import-sites \
  --headless

echo "Generated USD: $USD_PATH"
