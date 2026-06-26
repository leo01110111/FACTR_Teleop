#!/usr/bin/env bash
set -euo pipefail

ISAAC_CONDA_ENV="${ISAAC_CONDA_ENV:-env_isaaclab}"

source /home/srianumakonda/anaconda3/etc/profile.d/conda.sh
conda activate "$ISAAC_CONDA_ENV"
python - <<'PY'
import importlib.util
import sys

print("python:", sys.executable)
print("version:", sys.version.replace("\n", " "))
for name in ("isaacsim", "isaaclab", "torch", "torchaudio", "stable_baselines3"):
    spec = importlib.util.find_spec(name)
    print(f"{name}: {'FOUND' if spec else 'missing'}")
PY
