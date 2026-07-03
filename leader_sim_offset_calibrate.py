#!/usr/bin/env python3
"""
Interactive leader-real-to-sim offset calibration.

This is a sibling to leader_offset_calibrate.py, but computes a *different*
set of offsets: `offsets_<leader_name>.json` aligns the raw motor readings to
the follower UR's joint convention (used by FACTRTeleop for teleop matching).
This script instead aligns raw motor readings to the Pinocchio/URDF model's
own joint convention (used by visualize_leader_urdf.py --live, and by the
gravity-compensation/null-space math that consumes `pin_model` directly). The
two conventions have no reason to agree -- the URDF's zero pose and joint
axis directions are whatever onshape-to-robot exported, not the UR's
calibration stance -- so they get their own offset AND their own joint
signs, saved separately as `sim_offsets_<leader_name>.yaml`
(keys: `offset`, `joint_signs`).

Torque stays OFF the whole time. Procedure:

1. Pick a target joint configuration (in the URDF's own joint convention,
   the same order printed by visualize_leader_urdf.py as "Joint order:") and
   pass it as SIM_TARGET below.
2. Physically move the real leader arm to that exact configuration.
3. Run this script and press Enter when the arm is held there; it records
   raw motor angles and computes:

       sim_offset = raw - sim_target / sign

   so that (raw - sim_offset) * sign == sim_target at the recorded pose --
   the same convention already used by get_leader_arm_positions(), but with
   --joint-signs (default: the config's dynamixel.joint_signs) instead of
   necessarily the same signs used for the follower.

Usage:
    python leader_sim_offset_calibrate.py ur7e_leader_left.yaml 0 0 0 0 0 0 --write
    python leader_sim_offset_calibrate.py ur7e_leader_left.yaml 0 0 0 0 0 0 \\
        --joint-signs -1 -1 -1 -1 -1 -1 --write
"""
import argparse
import os
import shutil
import sys
import time
from pathlib import Path

import numpy as np
import yaml


def get_workspace_root():
    for parent in Path(__file__).resolve().parents:
        if (parent / "src/factr_teleop").exists():
            return str(parent)
    return os.getcwd()


WORKSPACE_ROOT = get_workspace_root()
for package_path in (
    "src/factr_teleop",
    "src/python_utils",
    "src/factr_teleop/factr_teleop/dynamixel/python/src",
):
    full_path = os.path.join(WORKSPACE_ROOT, package_path)
    if full_path not in sys.path:
        sys.path.insert(0, full_path)

from factr_teleop.dynamixel.driver import DynamixelDriver


def resolve_config_path(config_arg):
    if os.path.isfile(config_arg):
        return config_arg
    candidate = os.path.join(
        WORKSPACE_ROOT, "src/factr_teleop/factr_teleop/configs", config_arg
    )
    if os.path.isfile(candidate):
        return candidate
    raise FileNotFoundError(
        f"Could not find config '{config_arg}' (also tried '{candidate}')."
    )


def sim_offset_path_for_config(config):
    offset_filename = f"sim_offsets_{config['dynamixel']['leader_name']}.yaml"
    return os.path.join(
        WORKSPACE_ROOT, "src/factr_teleop/factr_teleop/configs", offset_filename
    )


def fmt(vec, precision=4):
    return np.array2string(
        np.asarray(vec, dtype=float),
        precision=precision,
        floatmode="fixed",
        separator=", ",
        max_line_width=1000,
    )


def read_average_position(driver, samples, sample_period):
    readings = []
    for _ in range(samples):
        pos, _ = driver.get_positions_and_velocities()
        readings.append(pos)
        time.sleep(sample_period)
    return np.mean(np.asarray(readings, dtype=float), axis=0)


def write_sim_offsets(offset_path, offsets, joint_signs):
    if os.path.exists(offset_path):
        backup_path = f"{offset_path}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
        shutil.copy2(offset_path, backup_path)
        print(f"Backed up existing sim offsets to: {backup_path}")

    sim_config = {
        "offset": [float(x) for x in offsets],
        "joint_signs": [float(x) for x in joint_signs],
    }
    with open(offset_path, "w") as f:
        yaml.safe_dump(sim_config, f, default_flow_style=True)
    print(f"Wrote sim offsets to: {offset_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", help="Config filename or path.")
    parser.add_argument(
        "sim_target",
        type=float,
        nargs="+",
        help="Arm joint angles (rad), in the URDF's own joint convention "
        "(same order as visualize_leader_urdf.py's 'Joint order:' printout), "
        "that you will physically move the real leader arm to before "
        "recording. One value per arm joint.",
    )
    parser.add_argument(
        "--joint-signs",
        type=float,
        nargs="+",
        default=None,
        help="Sim joint signs, one per arm joint. These are independent of "
        "dynamixel.joint_signs in the config (which align to the follower's "
        "convention) -- flip a sign here if a joint moves backwards in the "
        "sim visualization. Default: the config's dynamixel.joint_signs.",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=20,
        help="Number of raw samples to average. Default: 20.",
    )
    parser.add_argument(
        "--sample-rate",
        type=float,
        default=20.0,
        help="Sampling rate in Hz while recording. Default: 20.",
    )
    parser.add_argument(
        "--write",
        action="store_true",
        help="Write the computed sim offsets YAML after review.",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="With --write, skip the final confirmation prompt.",
    )
    args = parser.parse_args()

    if args.samples <= 0:
        raise ValueError("--samples must be positive")
    if args.sample_rate <= 0:
        raise ValueError("--sample-rate must be positive")

    config_path = resolve_config_path(args.config)
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    num_arm_joints = int(config["arm_teleop"]["num_arm_joints"])
    servo_types = config["dynamixel"]["servo_types"]
    joint_ids = np.arange(len(servo_types)) + 1
    port = "/dev/serial/by-id/" + config["dynamixel"]["dynamixel_port"]
    offset_path = sim_offset_path_for_config(config)

    sim_target = np.asarray(args.sim_target, dtype=float)
    if sim_target.shape != (num_arm_joints,):
        raise ValueError(
            f"Expected {num_arm_joints} sim_target values, got {sim_target.shape[0]}"
        )

    arm_signs = (
        np.asarray(config["dynamixel"]["joint_signs"][:num_arm_joints], dtype=float)
        if args.joint_signs is None
        else np.asarray(args.joint_signs, dtype=float)
    )
    if arm_signs.shape != (num_arm_joints,):
        raise ValueError(
            f"Expected {num_arm_joints} --joint-signs values, got {arm_signs.shape[0]}"
        )

    print(f"Config:          {config_path}")
    print(f"Leader name:      {config['dynamixel']['leader_name']}")
    print(f"Port:             {port}")
    print(f"Sim offsets file: {offset_path}")
    print(f"Sim joint signs:  {fmt(arm_signs, precision=1)}")
    print(f"Sim target pose:  {fmt(sim_target)}")
    print()
    print(
        "Torque will remain OFF. Move the real leader to the sim target pose "
        "above before recording."
    )

    driver = DynamixelDriver(joint_ids, servo_types, port)
    driver.set_torque_mode(False)

    sample_period = 1.0 / args.sample_rate
    try:
        print()
        input("Press Enter when the leader is held still at the sim target pose...")
        raw = read_average_position(driver, args.samples, sample_period)
        print(f"  raw sample: {fmt(raw)}")

        sim_offsets = np.zeros(len(servo_types), dtype=float)
        sim_offsets[:num_arm_joints] = raw[:num_arm_joints] - (sim_target / arm_signs)
        # No sim-space meaning for the gripper motor; keep its raw offset at 0
        # so it is a no-op until/unless a gripper sim convention is defined.

        reported = (raw[:num_arm_joints] - sim_offsets[:num_arm_joints]) * arm_signs
        error = reported - sim_target

        print()
        print("Computed sim offsets:")
        print(f"  sim offsets: {fmt(sim_offsets)}")
        print(f"  reported:    {fmt(reported)}")
        print(f"  error:       {fmt(error)}")

        if not args.write:
            print()
            print("Dry run: sim offsets were not written. Re-run with --write to save them.")
            return

        if not args.yes:
            print()
            answer = input("Write these sim offsets to YAML? Type 'yes' to continue: ")
            if answer.strip().lower() != "yes":
                print("Not writing sim offsets.")
                return

        write_sim_offsets(offset_path, sim_offsets, arm_signs)
    finally:
        driver.set_torque_mode(False)
        driver.close()


if __name__ == "__main__":
    main()
