#!/usr/bin/env bash
set -euo pipefail

ISAAC_CONDA_ENV="${ISAAC_CONDA_ENV:-env_isaaclab}"
ISAACLAB_DIR="${ISAACLAB_DIR:-/home/srianumakonda/IsaacLab}"
CONDA_SH="${CONDA_SH:-/home/srianumakonda/anaconda3/etc/profile.d/conda.sh}"

if [[ $# -lt 1 ]]; then
  echo "usage: $0 <script.py|-p-args...> [args...]" >&2
  exit 2
fi

# IsaacLab's launcher calls `tabs 4`, which fails in the Codex/CI TERM=dumb
# shell. Shadow it as a no-op; real terminal tab stops are irrelevant here.
function tabs() { :; }
export -f tabs

source "$CONDA_SH"
conda activate "$ISAAC_CONDA_ENV"
cd "$ISAACLAB_DIR"
TERM="${TERM:-dumb}" ./isaaclab.sh -p "$@"
