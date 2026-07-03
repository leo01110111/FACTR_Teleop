#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="${REPO_DIR:-/home/leo/factr_teleop_ur7e}"
ISAAC_CONDA_ENV="${ISAAC_CONDA_ENV:-env_isaaclab6}"
CONDA_SH="${CONDA_SH:-/home/srianumakonda/anaconda3/etc/profile.d/conda.sh}"
ISAAC_SITE="${ISAAC_SITE:-/home/srianumakonda/anaconda3/envs/${ISAAC_CONDA_ENV}/lib/python3.12/site-packages/isaacsim}"
CUMOTION_EXT="${CUMOTION_EXT:-${ISAAC_SITE}/exts/isaacsim.robot_motion.cumotion}"
CUMOTION_PREBUNDLE="${CUMOTION_PREBUNDLE:-${CUMOTION_EXT}/pip_prebundle}"

source "$CONDA_SH"
conda activate "$ISAAC_CONDA_ENV"

export PYTHONPATH="${REPO_DIR}/scripts/isaac_cumotion:${CUMOTION_EXT}:${CUMOTION_PREBUNDLE}:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="${CUMOTION_PREBUNDLE}/_cumotion_libs:${LD_LIBRARY_PATH:-}"

server_args=("$@")
mode_set=false
for arg in "${server_args[@]}"; do
  if [[ "$arg" == "--mode" || "$arg" == --mode=* ]]; then
    mode_set=true
    break
  fi
done

if [[ "$mode_set" == false ]]; then
  server_args=(--mode rmp "${server_args[@]}")
fi

python "$REPO_DIR/scripts/isaac_cumotion/isaac6_cumotion_stream_server.py" "${server_args[@]}"
