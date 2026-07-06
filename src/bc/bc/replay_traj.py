# ---------------------------------------------------------------------------
# FACTR: Force-Attending Curriculum Training for Contact-Rich Policy Learning
# https://arxiv.org/abs/2502.17432
# Copyright (c) 2025 Jason Jingzhou Liu and Yulong Li

# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ---------------------------------------------------------------------------

"""Open-loop replay of a recorded episode onto a real UR7e follower arm.

An episode recorded by ``bc/data_record`` (see record_data.sh) stores, among
other topics, the follower's actual joint positions on ``/ur/<side>/obs_ur_state``
and the Robotiq gripper position on ``/ur/<side>/obs_gripper`` (column 0, raw
0..255). Replaying those joint positions back through servoJ at the recorded
timing reproduces the demonstrated motion on the arm -- useful for sanity
checking collected data, staging, and demos without the leader in the loop.

This is a standalone RTDE tool (no Dynamixel leader, no ROS graph). It connects
to the UR through the ExternalControl URCap exactly like factr_teleop_ur7e,
servoJs to the first recorded pose, then streams the recorded trajectory with
servoJ (a single control mode throughout -- mixing moveJ and servoJ makes the
controller reject the following servoJ). The Robotiq gripper is optionally
driven from the recorded gripper positions.

Example:
    python -m bc.replay_traj --config-file ur7e_leader_left.yaml \
        --dataset test --episode 0

SAFETY: this moves a real arm along recorded data. Keep the pendant e-stop within
reach. If the arm starts far from the trajectory's first pose it is jogged there
slowly; confirm with ENTER before streaming begins (skip with --force).
"""

import argparse
import os
import pickle
import socket
import threading
import time
from pathlib import Path

import numpy as np
import yaml
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface

from python_utils.global_configs import (
    ur_left_real_addresses,
    ur_right_real_addresses,
)


# --------------------------------------------------------------------------- #
# Robotiq 2F-85 socket client (kept identical to factr_teleop_ur7e's client so
# replay drives the gripper exactly as teleop did).
# --------------------------------------------------------------------------- #
class RobotiqGripper:
    """Minimal client for the Robotiq 2F-85 socket exposed by the Robotiq URCap.

    Positions are 0..255 (0 = open, 255 = closed for the 2F-85).
    """

    def __init__(self, ip, port=63352, timeout=2.0):
        self._sock = socket.create_connection((ip, port), timeout=timeout)
        self._lock = threading.Lock()

    def _cmd(self, cmd):
        with self._lock:
            self._sock.sendall((cmd + "\n").encode())
            return self._sock.recv(1024).decode().strip()

    def _get_var(self, name):
        resp = self._cmd(f"GET {name}")
        return int(resp.split()[1])

    def activate(self, speed=255, force=128, wait_timeout=5.0):
        self._cmd("SET ACT 1")
        self._cmd(f"SET SPE {int(np.clip(speed, 0, 255))}")
        self._cmd(f"SET FOR {int(np.clip(force, 0, 255))}")
        self._cmd("SET GTO 1")
        t0 = time.time()
        while self._get_var("STA") != 3 and time.time() - t0 < wait_timeout:
            time.sleep(0.1)

    def set_position(self, pos):
        self._cmd(f"SET POS {int(np.clip(pos, 0, 255))}")

    def close(self):
        try:
            self._sock.close()
        except Exception:
            pass


# --------------------------------------------------------------------------- #
# Workspace / config helpers (mirrors return_ur_to_initial_match.py).
# --------------------------------------------------------------------------- #
def _workspace_root():
    colcon_prefix = os.environ.get("COLCON_PREFIX_PATH", "").split(os.pathsep)[0]
    if colcon_prefix:
        root = Path(colcon_prefix).resolve().parent
        if (root / "src/factr_teleop/factr_teleop/configs").exists():
            return root

    for parent in Path(__file__).resolve().parents:
        if (parent / "src/factr_teleop/factr_teleop/configs").exists():
            return parent

    return Path.cwd()


def _load_config(config_file):
    config_path = Path(config_file)
    if not config_path.is_file():
        config_path = (
            _workspace_root()
            / "src/factr_teleop/factr_teleop/configs"
            / config_file
        )
    if not config_path.is_file():
        raise FileNotFoundError(f"Could not find config file: {config_file}")
    with open(config_path, "r") as f:
        return yaml.safe_load(f), config_path


def _addresses_from_config(config, override_ip):
    name = config.get("name")
    if name == "right":
        addr = dict(ur_right_real_addresses)
    elif name == "left":
        addr = dict(ur_left_real_addresses)
    else:
        raise ValueError(
            f"Config name must be 'left' or 'right'; got {name!r}"
        )
    if override_ip:
        addr["ip"] = override_ip
    return name, addr


# --------------------------------------------------------------------------- #
# Episode loading.
# --------------------------------------------------------------------------- #
def _resolve_episode_path(dataset, episode):
    """Resolve a dataset name + episode index (or a direct .pkl path) to a file."""
    p = Path(dataset)
    if p.suffix == ".pkl" and p.is_file():
        return p

    dataset_dir = Path(dataset)
    if not dataset_dir.is_dir():
        dataset_dir = _workspace_root() / "raw_data" / dataset
    if not dataset_dir.is_dir():
        raise FileNotFoundError(f"Dataset directory not found: {dataset_dir}")

    ep_path = dataset_dir / f"ep_{int(episode):05d}.pkl"
    if not ep_path.is_file():
        raise FileNotFoundError(f"Episode file not found: {ep_path}")
    return ep_path


def _find_topic(topics, side, suffix):
    """Find the recorded topic ending in ``suffix`` (prefer the arm's side)."""
    candidates = [t for t in topics if t.endswith(suffix)]
    if not candidates:
        return None
    preferred = [t for t in candidates if f"/{side}/" in t]
    return (preferred or candidates)[0]


def _load_episode(ep_path, side, num_arm_joints):
    with open(ep_path, "rb") as f:
        log = pickle.load(f)

    data = log["data"]
    timestamps = log["timestamps"]
    topics = list(data.keys())

    state_topic = _find_topic(topics, side, "obs_ur_state")
    if state_topic is None:
        raise ValueError(
            f"No 'obs_ur_state' topic in {ep_path}; recorded topics: {topics}"
        )

    q_traj = np.asarray(data[state_topic], dtype=np.float64)[:, :num_arm_joints]
    t_state = np.asarray(timestamps[state_topic], dtype=np.float64) / 1e9  # ns -> s
    if len(q_traj) < 2:
        raise ValueError(f"Trajectory too short to replay ({len(q_traj)} samples).")
    # Recorded timestamps are monotonically increasing; rebase to start at 0.
    t0 = t_state[0]
    t_state = t_state - t0

    grip_topic = _find_topic(topics, side, "obs_gripper")
    if grip_topic is not None and len(data[grip_topic]) > 0:
        # obs_gripper is [position, current] in raw Robotiq 0..255; replay position.
        g_pos = np.asarray(data[grip_topic], dtype=np.float64)[:, 0]
        t_grip = np.asarray(timestamps[grip_topic], dtype=np.float64) / 1e9 - t0
    else:
        g_pos = None
        t_grip = None

    return {
        "state_topic": state_topic,
        "q_traj": q_traj,
        "t_state": t_state,
        "grip_topic": grip_topic,
        "g_pos": g_pos,
        "t_grip": t_grip,
    }


def _fmt(vec):
    return np.array2string(
        np.asarray(vec), precision=4, floatmode="fixed", separator=", "
    )


# --------------------------------------------------------------------------- #
# RTDE connect / cleanup (mirrors return_ur_to_initial_match.py).
# --------------------------------------------------------------------------- #
def _connect_external_control(robot_ip, frequency, ur_cap_port, timeout):
    deadline = time.monotonic() + timeout
    print(
        f"Connecting with ExternalControl URCap port {ur_cap_port}. "
        "Press Play on the pendant if the program is waiting."
    )
    last_error = None
    while True:
        try:
            return RTDEControlInterface(
                robot_ip,
                frequency,
                RTDEControlInterface.FLAG_USE_EXT_UR_CAP,
                ur_cap_port,
            )
        except RuntimeError as exc:
            last_error = exc
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(
                    f"Timed out connecting through ExternalControl on "
                    f"{robot_ip}:{ur_cap_port}. Last error: {last_error}"
                ) from exc
            print(f"  waiting for ExternalControl... ({remaining:.0f}s left)")
            time.sleep(min(2.0, remaining))


def _cleanup_rtde_control(rtde_c):
    for cleanup_name in ("servoStop", "stopScript", "disconnect"):
        try:
            getattr(rtde_c, cleanup_name)()
        except Exception as exc:
            print(
                f"Warning: RTDE control cleanup {cleanup_name}() failed: "
                f"{type(exc).__name__}: {exc}"
            )


def _servo_approach(rtde_c, q_start, q_goal, speed, lookahead, gain, cmd_dt=0.01):
    """Interpolate from q_start to q_goal using servoJ.

    The whole tool stays in servoJ control mode (like teleop): mixing moveJ and
    servoJ makes the controller reject the following servoJ (returns False), so
    the approach is streamed with servoJ too. ``speed`` is the max per-joint
    speed in rad/s; the move is timed to the leading (largest-delta) joint.
    """
    q_start = np.asarray(q_start, dtype=np.float64)
    q_goal = np.asarray(q_goal, dtype=np.float64)
    max_delta = float(np.max(np.abs(q_goal - q_start)))
    if max_delta < 1e-4:
        return
    duration = max(max_delta / max(speed, 1e-3), cmd_dt)
    n_steps = int(np.ceil(duration / cmd_dt))
    for k in range(1, n_steps + 1):
        alpha = k / n_steps
        q = q_start + alpha * (q_goal - q_start)
        ok = rtde_c.servoJ(q.tolist(), 0.0, 0.0, cmd_dt, lookahead, gain)
        if not ok:
            raise RuntimeError(
                "servoJ returned False during the approach; is the "
                "ExternalControl program running (Play pressed) on the pendant?"
            )
        time.sleep(cmd_dt)


# --------------------------------------------------------------------------- #
# Gripper replay thread: drives the Robotiq from the recorded gripper positions,
# indexed by the shared replay clock. Decoupled from the arm loop because the
# Robotiq URCap socket is slow (~ms per command), exactly as in teleop.
# --------------------------------------------------------------------------- #
class _GripperReplay:
    def __init__(self, gripper, t_grip, g_pos, rate_scale):
        self._gripper = gripper
        self._t_grip = t_grip
        self._g_pos = g_pos
        self._rate_scale = rate_scale
        self._start = None
        self._running = False
        self._thread = None

    def start(self, start_time):
        self._start = start_time
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        period = 0.04  # ~25 Hz, matching factr_teleop_ur7e's gripper thread
        while self._running:
            replay_t = (time.perf_counter() - self._start) * self._rate_scale
            idx = int(np.searchsorted(self._t_grip, replay_t, side="right")) - 1
            idx = max(0, min(idx, len(self._g_pos) - 1))
            try:
                self._gripper.set_position(self._g_pos[idx])
            except Exception:
                pass
            time.sleep(period)

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.0)


# --------------------------------------------------------------------------- #
# Main.
# --------------------------------------------------------------------------- #
def main(args=None):
    parser = argparse.ArgumentParser(
        description="Replay a recorded episode onto a real UR7e follower arm."
    )
    parser.add_argument(
        "--config-file",
        default="ur7e_leader_left.yaml",
        help="FACTR config filename or path (selects arm ip/ports/servo params).",
    )
    parser.add_argument(
        "--dataset",
        default="test",
        help="Dataset name under raw_data/, a dataset dir, or a direct .pkl path.",
    )
    parser.add_argument(
        "--episode",
        type=int,
        default=0,
        help="Episode index within the dataset (ignored if --dataset is a .pkl).",
    )
    parser.add_argument(
        "--robot-ip",
        default=None,
        help="Override UR robot IP (default from config name).",
    )
    parser.add_argument(
        "--rate-scale",
        type=float,
        default=1.0,
        help="Playback speed multiplier (>1 faster, <1 slower). Default: 1.0.",
    )
    parser.add_argument(
        "--approach-speed",
        type=float,
        default=0.35,
        help="servoJ approach speed to the first pose (max joint), rad/s. "
        "Default: 0.35.",
    )
    parser.add_argument(
        "--slow-approach-speed",
        type=float,
        default=0.1,
        help="servoJ approach speed used when the arm is far (> --max-start-delta) "
        "from the first pose, rad/s. Default: 0.1.",
    )
    parser.add_argument(
        "--max-start-delta",
        type=float,
        default=0.5,
        help="If any joint is farther than this (rad) from the first recorded "
        "pose, jog there slowly (--slow-approach-*) instead of at the normal "
        "approach speed. Default: 0.5.",
    )
    parser.add_argument(
        "--no-gripper",
        action="store_true",
        help="Do not drive the Robotiq gripper during replay.",
    )
    parser.add_argument(
        "--external-control-timeout",
        type=float,
        default=180.0,
        help="Seconds to wait for the pendant ExternalControl program/Play.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Load and summarize the trajectory; do not connect or move.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Skip the ENTER confirmation before a far (slow) approach jog.",
    )
    parsed = parser.parse_args(args=args)

    config, config_path = _load_config(parsed.config_file)
    name, addr = _addresses_from_config(config, parsed.robot_ip)
    robot_ip = addr["ip"]
    ur_cap_port = addr["ur_cap_port"]
    gripper_port = addr["gripper_port"]

    frequency = float(config["controller"]["frequency"])
    num_arm_joints = int(config["arm_teleop"]["num_arm_joints"])
    servo_cfg = config["arm_teleop"].get("servo", {})
    lookahead_time = float(servo_cfg.get("lookahead_time", 0.1))
    gain = float(servo_cfg.get("gain", 300.0))
    watchdog_min_frequency = float(servo_cfg.get("watchdog_min_frequency", 20.0))

    ep_path = _resolve_episode_path(parsed.dataset, parsed.episode)
    traj = _load_episode(ep_path, name, num_arm_joints)
    q_traj = traj["q_traj"]
    t_state = traj["t_state"]
    duration = float(t_state[-1])

    print(f"Config:   {config_path}")
    print(f"Robot:    {name} at {robot_ip} (ur_cap_port={ur_cap_port})")
    print(f"Episode:  {ep_path}")
    print(f"State topic: {traj['state_topic']}  ({len(q_traj)} samples)")
    print(
        f"Duration: {duration:.2f}s  ->  {duration / max(parsed.rate_scale, 1e-6):.2f}s "
        f"at rate_scale={parsed.rate_scale}"
    )
    print(f"First pose: {_fmt(q_traj[0])}")
    print(f"Last pose:  {_fmt(q_traj[-1])}")
    if traj["g_pos"] is None:
        print("Gripper:  no recorded gripper data (arm-only replay).")
    elif parsed.no_gripper:
        print(f"Gripper:  {traj['grip_topic']} present but disabled (--no-gripper).")
    else:
        print(
            f"Gripper:  {traj['grip_topic']}  "
            f"({len(traj['g_pos'])} samples, "
            f"pos {traj['g_pos'].min():.0f}..{traj['g_pos'].max():.0f})"
        )

    if parsed.dry_run:
        print("Dry run only; not connecting or moving.")
        return

    # --- Connect RTDE ---
    rtde_r = RTDEReceiveInterface(robot_ip)
    try:
        current_q = np.asarray(rtde_r.getActualQ(), dtype=np.float64)[:num_arm_joints]
    finally:
        rtde_r.disconnect()

    start_delta = np.abs(current_q - q_traj[0])
    print(f"Current UR pose: {_fmt(current_q)}")
    print(f"Approach delta:  {_fmt(start_delta)}  (max {float(start_delta.max()):.4f} rad)")
    # When the arm is far from the first recorded pose, servo there slowly rather
    # than at the normal approach speed (a large fast jog is unsafe).
    is_far = bool(np.any(start_delta > parsed.max_start_delta))
    if is_far:
        approach_speed = parsed.slow_approach_speed
        print(
            f"Arm is far from the first pose (max {float(start_delta.max()):.3f} rad "
            f"> --max-start-delta={parsed.max_start_delta}); jogging there SLOWLY at "
            f"{approach_speed} rad/s."
        )
    else:
        approach_speed = parsed.approach_speed

    # Confirm BEFORE any streaming starts. Once servoJ begins, the arm stays in
    # servo mode with the watchdog live, so there is no safe point to block for
    # input mid-motion -- the approach flows straight into replay.
    if not parsed.force:
        input(
            "Press ENTER to connect and run (arm jogs to the first pose, then "
            "replays; Ctrl-C to abort)... "
        )

    gripper = None
    if traj["g_pos"] is not None and not parsed.no_gripper:
        gripper_cfg = config.get("gripper_teleop", {})
        try:
            gripper = RobotiqGripper(robot_ip, gripper_port)
            gripper.activate(
                speed=gripper_cfg.get("speed", 255),
                force=gripper_cfg.get("force", 128),
            )
            print("Robotiq 2F-85 connected and activated.")
        except Exception as exc:
            print(f"Warning: gripper init failed ({exc}); replaying arm only.")
            gripper = None

    rtde_c = _connect_external_control(
        robot_ip, frequency, ur_cap_port, parsed.external_control_timeout
    )
    gripper_replay = None
    try:
        rtde_c.setWatchdog(watchdog_min_frequency)

        # Servo (not moveJ) to the first recorded pose so the controller stays in
        # a single servoJ control mode through to the replay -- mixing moveJ and
        # servoJ makes the following servoJ return False.
        print(f"Jogging to first recorded pose via servoJ at {approach_speed} rad/s...")
        _servo_approach(
            rtde_c, current_q, q_traj[0], approach_speed, lookahead_time, gain
        )

        # Stream the recorded trajectory with servoJ, pacing to recorded timing.
        # Flows directly on from the approach with no pause (watchdog stays fed).
        print("Replaying...")
        start = time.perf_counter()
        if gripper is not None:
            gripper_replay = _GripperReplay(
                gripper, traj["t_grip"], traj["g_pos"], parsed.rate_scale
            )
            gripper_replay.start(start)

        last_servo_t = None
        for i in range(1, len(q_traj)):
            target_t = t_state[i] / parsed.rate_scale
            # Busy-wait-ish sleep until this sample is due (relative to replay start).
            while True:
                ahead = target_t - (time.perf_counter() - start)
                if ahead <= 0:
                    break
                time.sleep(min(ahead, 0.005))

            now = time.perf_counter()
            servo_dt = 0.008 if last_servo_t is None else now - last_servo_t
            last_servo_t = now
            # servoJ's `time` arg must reflect the real inter-command interval.
            servo_dt = float(np.clip(servo_dt, 0.002, 0.05))
            ok = rtde_c.servoJ(
                q_traj[i].tolist(), 0.0, 0.0, servo_dt, lookahead_time, gain
            )
            if not ok:
                raise RuntimeError(
                    f"servoJ returned False at sample {i}/{len(q_traj)}; the "
                    "ExternalControl program likely stopped (check the pendant)."
                )

        print(f"Replay complete ({duration / max(parsed.rate_scale, 1e-6):.2f}s).")
    except KeyboardInterrupt:
        print("\nInterrupted; stopping arm.")
    finally:
        if gripper_replay is not None:
            gripper_replay.stop()
        _cleanup_rtde_control(rtde_c)
        try:
            rtde_r.disconnect()
        except Exception:
            pass
        if gripper is not None:
            gripper.close()


if __name__ == "__main__":
    main()
