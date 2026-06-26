#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="${REPO_DIR:-/home/srianumakonda/FACTR_Teleop}"
ISAAC_CONDA_ENV="${ISAAC_CONDA_ENV:-env_isaaclab6}"
CONDA_SH="${CONDA_SH:-/home/srianumakonda/anaconda3/etc/profile.d/conda.sh}"
ISAAC_SITE="${ISAAC_SITE:-/home/srianumakonda/anaconda3/envs/${ISAAC_CONDA_ENV}/lib/python3.12/site-packages/isaacsim}"
CUMOTION_EXT="${CUMOTION_EXT:-${ISAAC_SITE}/exts/isaacsim.robot_motion.cumotion}"
CUMOTION_PREBUNDLE="${CUMOTION_PREBUNDLE:-${CUMOTION_EXT}/pip_prebundle}"

MJCF_PATH="${MJCF_PATH:-${REPO_DIR}/generated/isaac_cumotion/maxlab_dual_ur7e_table.xml}"
USD_DIR="${USD_DIR:-${REPO_DIR}/generated/isaac_cumotion/usd}"

source "$CONDA_SH"
conda activate "$ISAAC_CONDA_ENV"

export PYTHONPATH="${REPO_DIR}/scripts/isaac_cumotion:${CUMOTION_EXT}:${CUMOTION_PREBUNDLE}:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="${CUMOTION_PREBUNDLE}/_cumotion_libs:${LD_LIBRARY_PATH:-}"

python "$REPO_DIR/scripts/isaac_cumotion/export_maxlab_mjcf.py" --output "$MJCF_PATH"
python "$REPO_DIR/scripts/isaac_cumotion/convert_mjcf_to_usd.py" "$MJCF_PATH" --output-dir "$USD_DIR" "$@"
