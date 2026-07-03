#!/usr/bin/env bash
# Move a UR follower to its config's initial_match_joint_pos.
# Usage: sg dialout -c './return_init.sh [--left|--right|config.yaml] [extra args...]'
#   --left  (-l) -> ur7e_leader_left.yaml
#   --right (-r) -> ur7e_leader_right.yaml
#   a config.yaml (name or path) may still be passed explicitly.
#   defaults to ur7e_leader_left.yaml
set -e
cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate
source install/setup.bash
case "${1:-}" in
  --left|-l|left)   CONFIG="ur7e_leader_left.yaml";  shift ;;
  --right|-r|right) CONFIG="ur7e_leader_right.yaml"; shift ;;
  ""|-*)            CONFIG="ur7e_leader_left.yaml" ;;  # default; leave -* flags for python
  *)                CONFIG="$1"; shift ;;              # explicit config name/path
esac
exec python -u install/factr_teleop/lib/factr_teleop/return_ur_to_initial_match \
  --config-file "$CONFIG" "$@"
