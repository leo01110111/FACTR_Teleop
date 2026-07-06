#!/usr/bin/env bash
# One-shot DUAL-ARM teleop + data collection.
#
# This is the single entry point for a data-collection session. It replaces the
# manual sequence of:
#     ./run_factr_left.sh      (in one terminal)
#     ./run_factr_right.sh     (in another terminal)
#     ./record_data.sh <name>  (in a third terminal)
#     + walking to each pendant and pressing PLAY on ExternalControl
#
# Instead it:
#   1. Confirms both UR pendants are in REMOTE CONTROL mode.
#   2. Launches the left + right FACTR teleop nodes (same as run_factr_*.sh).
#   3. Auto-PLAYs the ExternalControl URCap program on both arms over the UR
#      Dashboard server (port 29999) -- no pressing Play on the pendant.
#   4. Runs the data-collection pipeline via ./record_data.sh (delegated, so it
#      always uses the current pipeline).
#   5. On exit (Ctrl-C / error), STOPs ExternalControl on both arms and tears
#      the teleop nodes down.
#
# REMOTE CONTROL is a prerequisite and cannot be set from software: on each
# pendant, top-right menu -> switch Local -> Remote Control. While in Remote,
# the pendant's manual jog/Play are disabled, so:
#   - Jog each follower UR to its initial_match_joint_pos BEFORE switching to
#     Remote (or the node's start-config check aborts), and
#   - keep the physical E-STOP within reach -- auto-Play removes the "human
#     presses Play" gate, so the confirmation prompt below is your last check.
#
# Controls once recording starts (keep the recorder terminal focused):
#   SPACE  - start / stop an episode (saved on stop)
#   DELETE - delete the most recently saved episode
#   Ctrl-C - end the whole session (stops both arms + nodes)
#
# Usage:
#   ./run_data_collection.sh [dataset_name]     (default dataset: test)
# Flags (may appear before or after the dataset name):
#   -y, --yes      skip the pre-start safety confirmation
#       --no-play  do NOT auto-Play; press Play on each pendant yourself
# Env overrides (optional):
#   EXTCTRL_PROG_LEFT / EXTCTRL_PROG_RIGHT  .urp to `load` before Play.
#       Default: teleop.urp on both pendants (the ExternalControl program).
set -e

# --- arm addresses (mirror src/python_utils/python_utils/global_configs.py) ---
LEFT_IP="192.168.1.2"
RIGHT_IP="192.168.2.2"
LEFT_CONFIG="ur7e_leader_left.yaml"
RIGHT_CONFIG="ur7e_leader_right.yaml"

# --- parse args -------------------------------------------------------------
DATASET=""
ASSUME_YES=0
AUTO_PLAY=1
for arg in "$@"; do
    case "$arg" in
        -y|--yes)   ASSUME_YES=1 ;;
        --no-play)  AUTO_PLAY=0 ;;
        -*)         echo "Unknown flag: $arg" >&2; exit 2 ;;
        *)
            if [ -n "$DATASET" ]; then
                echo "Unexpected extra argument: $arg" >&2; exit 2
            fi
            DATASET="$arg"
            ;;
    esac
done
DATASET="${DATASET:-test}"

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$REPO"

# conda's base env breaks ROS 2 rclpy; leave it before sourcing the overlay.
if command -v conda >/dev/null 2>&1 && [ -n "${CONDA_DEFAULT_ENV:-}" ]; then
    conda deactivate || true
fi

# The Dynamixel U2D2 needs the dialout group (see run_factr_*.sh).
if ! id -nG | tr ' ' '\n' | grep -qx dialout; then
    echo "[run_data_collection] WARNING: not in 'dialout' group; the U2D2 open may fail."
    echo "  Fix: sudo usermod -aG dialout \"\$USER\" then log out and back in."
fi

source ./factr_env

# --- UR Dashboard helpers (port 29999), mirroring return_ur_to_initial_match.py
# dash <ip> <command> -> prints the dashboard response; nonzero exit on socket error.
dash() {
    python3 - "$1" "$2" <<'PY'
import socket, sys
ip, cmd = sys.argv[1], sys.argv[2]
try:
    with socket.create_connection((ip, 29999), timeout=3.0) as s:
        s.settimeout(3.0)
        s.recv(4096)                      # consume the dashboard greeting line
        s.sendall((cmd + "\n").encode())
        print(s.recv(4096).decode(errors="replace").strip())
except OSError as e:
    print(f"DASHBOARD_ERROR: {e}", file=sys.stderr)
    sys.exit(1)
PY
}

is_remote() {  # is_remote <ip> -> 0 if the pendant is in Remote Control mode
    local resp
    resp="$(dash "$1" "is in remote control")" || return 1
    case "${resp,,}" in *true) return 0 ;; *) return 1 ;; esac
}

# play_external_control <ip> <name> <prog> -> start ExternalControl via dashboard.
play_external_control() {
    local ip="$1" name="$2" prog="$3" resp tries=0
    if [ -n "$prog" ]; then
        resp="$(dash "$ip" "load $prog")" || { echo "  [$name] dashboard unreachable at $ip"; return 1; }
        echo "  [$name] load $prog -> $resp"
    fi
    # Retry: the node's RTDE listener may not be open the instant we launch it;
    # a Play before it is up just faults the program, so we re-issue Play.
    while [ "$tries" -lt 6 ]; do
        resp="$(dash "$ip" "play")" || { echo "  [$name] dashboard unreachable at $ip"; return 1; }
        echo "  [$name] play -> $resp"
        case "$resp" in *"Starting program"*) return 0 ;; esac
        tries=$((tries + 1))
        sleep 2
    done
    echo "  [$name] ERROR: could not start ExternalControl (last response: $resp)"
    echo "  [$name] Ensure teleop.urp exists on the pendant (or set"
    echo "  [$name] EXTCTRL_PROG_${name^^}=<file>.urp), and the arm is powered."
    return 1
}

# --- preflight: Remote Control mode on both arms ----------------------------
if [ "$AUTO_PLAY" -eq 1 ]; then
    echo "[run_data_collection] Checking UR Remote Control mode..."
    NOT_REMOTE=0
    for pair in "left:$LEFT_IP" "right:$RIGHT_IP"; do
        name="${pair%%:*}"; ip="${pair##*:}"
        if is_remote "$ip"; then
            echo "  [$name] $ip: Remote Control ON"
        else
            echo "  [$name] $ip: NOT in Remote Control (or dashboard unreachable)"
            NOT_REMOTE=1
        fi
    done
    if [ "$NOT_REMOTE" -eq 1 ]; then
        echo "[run_data_collection] One or more arms are not in Remote Control mode."
        echo "  On each pendant: top-right menu -> switch Local -> Remote Control,"
        echo "  then rerun. To press Play manually instead, rerun with --no-play."
        exit 1
    fi
fi

# --- safety confirmation ----------------------------------------------------
if [ "$ASSUME_YES" -ne 1 ]; then
    echo
    echo "About to auto-start teleop on BOTH arms and begin data collection."
    if [ "$AUTO_PLAY" -eq 1 ]; then
        echo "  * ExternalControl will be PLAYED remotely -- the followers will accept"
        echo "    servo targets and move to match the leaders as soon as you match them."
    fi
    echo "  * Ensure the workspace is clear and the E-STOP is within reach."
    echo "  * Dataset -> raw_data/${DATASET}/"
    read -r -p "Continue? [y/N] " reply
    case "$reply" in [yY]|[yY][eE][sS]) ;; *) echo "Aborted."; exit 1 ;; esac
fi

# --- launch both teleop nodes (same as run_factr_left/right.sh) -------------
# Distinct node names so both can run on one ROS graph (see the launch file).
echo "[run_data_collection] Launching left teleop node..."
ros2 launch launch/factr_teleop_ur7e.py \
    config_file:="${LEFT_CONFIG}" node_name:=factr_teleop_ur7e_left &
LEFT_PID=$!
echo "[run_data_collection] Launching right teleop node..."
ros2 launch launch/factr_teleop_ur7e.py \
    config_file:="${RIGHT_CONFIG}" node_name:=factr_teleop_ur7e_right &
RIGHT_PID=$!

# --- cleanup on any exit ----------------------------------------------------
CLEANED=0
cleanup() {
    [ "$CLEANED" -eq 1 ] && return
    CLEANED=1
    trap - EXIT INT TERM
    echo
    echo "[run_data_collection] Shutting down..."
    # Stop ExternalControl on both arms so the followers are no longer under
    # external control once we let go.
    for ip in "$LEFT_IP" "$RIGHT_IP"; do
        dash "$ip" "stop" >/dev/null 2>&1 || true
    done
    # Bring the teleop nodes down (they also get the terminal's Ctrl-C; this is
    # the safety net). Their own shutdown disables Dynamixel torque.
    kill "$LEFT_PID" "$RIGHT_PID" 2>/dev/null || true
    wait "$LEFT_PID" "$RIGHT_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# --- auto-Play ExternalControl on both arms ---------------------------------
if [ "$AUTO_PLAY" -eq 1 ]; then
    echo "[run_data_collection] Starting ExternalControl (Remote Play)..."
    # Give the nodes a moment to open their RTDE listeners before Play.
    sleep 3
    play_external_control "$LEFT_IP"  "left"  "${EXTCTRL_PROG_LEFT:-teleop.urp}"  || { echo "Left ExternalControl failed."; exit 1; }
    play_external_control "$RIGHT_IP" "right" "${EXTCTRL_PROG_RIGHT:-teleop.urp}" || { echo "Right ExternalControl failed."; exit 1; }
else
    echo "[run_data_collection] --no-play: press PLAY on each pendant now."
fi

echo
echo "[run_data_collection] Teleop starting. Move each LEADER arm to match its"
echo "  follower when prompted (match prompts from both arms share this terminal)."
echo "  Then use SPACE to record episodes. Ctrl-C ends the session."
echo

# --- run the data-collection pipeline (delegated to record_data.sh) ---------
# Foreground: blocks until you Ctrl-C, then cleanup() runs. record_data.sh owns
# the camera + recorder, so pipeline changes are picked up automatically.
./record_data.sh "${DATASET}"
