#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="${REPO_DIR:-/home/srianumakonda/FACTR_Teleop}"
ISAAC_CONDA_ENV="${ISAAC_CONDA_ENV:-env_isaaclab}"
CONDA_SH="${CONDA_SH:-/home/srianumakonda/anaconda3/etc/profile.d/conda.sh}"
ISAAC_SITE="${ISAAC_SITE:-/home/srianumakonda/anaconda3/envs/env_isaaclab/lib/python3.11/site-packages/isaacsim}"
LULA_PREBUNDLE="$ISAAC_SITE/exts/isaacsim.robot_motion.lula/pip_prebundle"

source "$CONDA_SH"
conda activate "$ISAAC_CONDA_ENV"

export PYTHONPATH="$LULA_PREBUNDLE${PYTHONPATH:+:$PYTHONPATH}"
export LD_LIBRARY_PATH="$LULA_PREBUNDLE/_lula_libs${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

cd "$REPO_DIR"
exec python "$REPO_DIR/scripts/isaac_rmpflow/isaac_rmpflow_zmq_server.py" "$@"
