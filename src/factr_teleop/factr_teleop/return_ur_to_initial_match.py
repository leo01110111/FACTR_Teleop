import argparse
import os
import socket
import time
from pathlib import Path

import numpy as np
import yaml
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface

from python_utils.global_configs import ur_left_real_addresses, ur_right_real_addresses


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


def _robot_ip_from_config(config, override_ip):
    if override_ip:
        return override_ip

    name = config.get("name")
    if name == "right":
        return ur_right_real_addresses["ip"]
    if name == "left":
        return ur_left_real_addresses["ip"]
    raise ValueError(
        f"Config name must be 'left' or 'right' unless --robot-ip is passed; got {name!r}"
    )


def _ur_cap_port_from_config(config, override_port):
    if override_port is not None:
        return override_port

    name = config.get("name")
    if name == "right":
        return ur_right_real_addresses["ur_cap_port"]
    if name == "left":
        return ur_left_real_addresses["ur_cap_port"]
    raise ValueError(
        f"Config name must be 'left' or 'right' unless --ur-cap-port is passed; got {name!r}"
    )


def _fmt(vec):
    return np.array2string(
        np.asarray(vec),
        precision=4,
        floatmode="fixed",
        separator=", ",
    )


def _dashboard_command(robot_ip, command, timeout=2.0):
    with socket.create_connection((robot_ip, 29999), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.recv(4096)
        sock.sendall((command + "\n").encode())
        return sock.recv(4096).decode(errors="replace").strip()


def _is_remote_control(robot_ip):
    response = _dashboard_command(robot_ip, "is in remote control")
    return response.lower().endswith("true")


def _wait_for_remote_control(robot_ip, timeout):
    deadline = time.monotonic() + timeout
    print(
        "Waiting for UR Remote Control mode. On the pendant, switch Local -> Remote "
        "if needed."
    )
    while True:
        try:
            if _is_remote_control(robot_ip):
                print("UR is in Remote Control mode.")
                return
            status = "not remote"
        except OSError as exc:
            status = f"dashboard unavailable ({exc})"

        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError(
                f"Timed out waiting for Remote Control mode on {robot_ip}. "
                "Switch the pendant to Remote Control and rerun, or pass a larger "
                "--remote-timeout."
            )

        print(f"  {status}; waiting... ({remaining:.0f}s left)")
        time.sleep(min(2.0, remaining))


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
                    f"Timed out connecting through ExternalControl on {robot_ip}:{ur_cap_port}. "
                    f"Last error: {last_error}"
                ) from exc
            print(f"  waiting for ExternalControl... ({remaining:.0f}s left)")
            time.sleep(min(2.0, remaining))


def main():
    parser = argparse.ArgumentParser(
        description="Move a UR follower arm to the config's initial_match_joint_pos."
    )
    parser.add_argument(
        "--config-file",
        default="ur7e_leader_right.yaml",
        help="FACTR config filename or path. Default: ur7e_leader_right.yaml",
    )
    parser.add_argument(
        "--robot-ip",
        default=None,
        help="Override UR robot IP. By default this is selected from config name.",
    )
    parser.add_argument(
        "--ur-cap-port",
        type=int,
        default=None,
        help="Override ExternalControl custom port. Defaults to global config for the arm.",
    )
    parser.add_argument(
        "--speed",
        type=float,
        default=0.35,
        help="moveJ leading-axis speed in rad/s. Default: 0.35",
    )
    parser.add_argument(
        "--acceleration",
        type=float,
        default=0.5,
        help="moveJ leading-axis acceleration in rad/s^2. Default: 0.5",
    )
    parser.add_argument(
        "--verify-tolerance",
        type=float,
        default=0.05,
        help="Allowed final per-joint error in rad. Default: 0.05",
    )
    parser.add_argument(
        "--max-delta",
        type=float,
        default=6.5,
        help="Abort if any joint must move more than this many rad unless --force is used.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Only print the current/target/delta; do not move the robot.",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Deprecated; movement starts automatically after ExternalControl connects.",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        help="Allow moves larger than --max-delta.",
    )
    parser.add_argument(
        "--remote-timeout",
        type=float,
        default=0.0,
        help="Optional seconds to wait for Dashboard Remote Control mode. Default: 0.",
    )
    parser.add_argument(
        "--external-control-timeout",
        type=float,
        default=180.0,
        help="Seconds to wait for the pendant ExternalControl program/Play.",
    )
    args = parser.parse_args()

    config, config_path = _load_config(args.config_file)
    robot_ip = _robot_ip_from_config(config, args.robot_ip)
    ur_cap_port = _ur_cap_port_from_config(config, args.ur_cap_port)
    frequency = float(config["controller"]["frequency"])
    num_arm_joints = int(config["arm_teleop"]["num_arm_joints"])
    target = np.asarray(
        config["arm_teleop"]["initialization"]["initial_match_joint_pos"],
        dtype=float,
    )[:num_arm_joints]

    if len(target) != 6:
        raise ValueError(f"UR target must have 6 joints; got {len(target)}")

    print(f"Config: {config_path}")
    print(f"Robot:  {config.get('name')} at {robot_ip} (ur_cap_port={ur_cap_port})")
    print(f"Target initial_match_joint_pos: {_fmt(target)}")

    rtde_r = RTDEReceiveInterface(robot_ip)
    try:
        current = np.asarray(rtde_r.getActualQ(), dtype=float)
    finally:
        rtde_r.disconnect()

    delta = target - current
    abs_delta = np.abs(delta)
    print(f"Current UR joint position:      {_fmt(current)}")
    print(f"Delta target-current:           {_fmt(delta)}")
    print(f"Max abs delta: {float(np.max(abs_delta)):.4f} rad")

    if np.any(abs_delta > args.max_delta) and not args.force:
        raise RuntimeError(
            f"Refusing move: at least one joint delta exceeds --max-delta={args.max_delta}. "
            "Inspect the target/current pose, or rerun with --force if this is intentional."
        )

    if args.dry_run:
        print("Dry run only; not moving.")
        return

    if args.remote_timeout > 0:
        _wait_for_remote_control(robot_ip, args.remote_timeout)

    rtde_c = _connect_external_control(
        robot_ip, frequency, ur_cap_port, args.external_control_timeout
    )
    try:
        print("RTDE control is connected; sending moveJ.")
        ok = rtde_c.moveJ(target.tolist(), args.speed, args.acceleration, False)
        if not ok:
            raise RuntimeError("RTDE moveJ returned False")
    finally:
        try:
            rtde_c.stopScript()
        finally:
            rtde_c.disconnect()

    rtde_r = RTDEReceiveInterface(robot_ip)
    try:
        final = np.asarray(rtde_r.getActualQ(), dtype=float)
    finally:
        rtde_r.disconnect()

    final_error = np.abs(final - target)
    print(f"Final UR joint position:        {_fmt(final)}")
    print(f"Final abs error:                {_fmt(final_error)}")

    if np.any(final_error > args.verify_tolerance):
        raise RuntimeError(
            f"Move completed, but final error exceeded {args.verify_tolerance} rad."
        )

    print("UR is at initial_match_joint_pos.")


if __name__ == "__main__":
    main()
