#!/usr/bin/env python3
"""
Interactive leader offset calibration and start-pose verification.

This opens only the GELLO/leader Dynamixels. Torque stays OFF the whole time.
The script records raw motor angles at:

1. The configured calibration pose.
2. The configured initial-match pose.

Offsets are computed from the calibration-pose sample:

    reported = (raw - offset) * sign
    offset = raw - target / sign

The initial-match sample is then used only to verify that the same offsets make
the start pose report near initial_match_joint_pos.

Usage:
    python leader_offset_calibrate.py ur7e_leader_right.yaml
    python leader_offset_calibrate.py ur7e_leader_left.yaml --write
"""
import argparse
import json
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
        get_workspace_root(),
        "src/factr_teleop/factr_teleop/configs",
        config_arg,
    )
    if os.path.isfile(candidate):
        return candidate
    raise FileNotFoundError(
        f"Could not find config '{config_arg}' (also tried '{candidate}')."
    )


def offset_path_for_config(config):
    offset_filename = f"offsets_{config['dynamixel']['leader_name']}.json"
    return os.path.join(
        get_workspace_root(),
        "src/factr_teleop/factr_teleop/configs",
        offset_filename,
    )


def fmt(vec, precision=4):
    return np.array2string(
        np.asarray(vec, dtype=float),
        precision=precision,
        floatmode="fixed",
        separator=", ",
        max_line_width=1000,
    )


def normalize_to_reference(q, reference):
    return reference + np.arctan2(np.sin(q - reference), np.cos(q - reference))


def read_average_position(driver, samples, sample_period):
    readings = []
    for _ in range(samples):
        pos, _ = driver.get_positions_and_velocities()
        readings.append(pos)
        time.sleep(sample_period)
    return np.mean(np.asarray(readings, dtype=float), axis=0)


def prompt_and_record(driver, label, target, samples, sample_period):
    print()
    print(f"Move the leader to the {label} pose:")
    print(f"  target q: {fmt(target)}")
    input("Press Enter when the leader is held still there...")
    raw = read_average_position(driver, samples, sample_period)
    print(f"  raw sample: {fmt(raw)}")
    return raw


def write_offsets(offset_path, offsets):
    if os.path.exists(offset_path):
        backup_path = f"{offset_path}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
        shutil.copy2(offset_path, backup_path)
        print(f"Backed up existing offsets to: {backup_path}")

    with open(offset_path, "w") as f:
        json.dump([float(x) for x in offsets], f, indent=2)
        f.write("\n")
    print(f"Wrote offsets to: {offset_path}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", help="Config filename or path.")
    parser.add_argument(
        "--samples",
        type=int,
        default=20,
        help="Number of raw samples to average at each pose. Default: 20.",
    )
    parser.add_argument(
        "--sample-rate",
        type=float,
        default=20.0,
        help="Sampling rate in Hz while recording each pose. Default: 20.",
    )
    parser.add_argument(
        "--match-threshold",
        type=float,
        default=0.6,
        help="L2 error threshold used by FACTR leader matching. Default: 0.6.",
    )
    parser.add_argument(
        "--write",
        action="store_true",
        help="Write the computed offsets JSON after verification.",
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
    joint_signs = np.asarray(config["dynamixel"]["joint_signs"], dtype=float)
    port = "/dev/serial/by-id/" + config["dynamixel"]["dynamixel_port"]
    offset_path = offset_path_for_config(config)

    calibration_q = np.asarray(
        config["arm_teleop"]["initialization"]["calibration_joint_pos"],
        dtype=float,
    )[:num_arm_joints]
    initial_match_q = np.asarray(
        config["arm_teleop"]["initialization"]["initial_match_joint_pos"],
        dtype=float,
    )[:num_arm_joints]
    normalize = bool(
        config["arm_teleop"]["initialization"].get("normalize_joint_angles", False)
    )

    if joint_signs.shape[0] != len(servo_types):
        raise ValueError("joint_signs length must match servo_types length")
    if calibration_q.shape != (num_arm_joints,):
        raise ValueError("calibration_joint_pos length does not match num_arm_joints")
    if initial_match_q.shape != (num_arm_joints,):
        raise ValueError("initial_match_joint_pos length does not match num_arm_joints")

    print(f"Config:       {config_path}")
    print(f"Leader name:  {config['dynamixel']['leader_name']}")
    print(f"Port:         {port}")
    print(f"Offsets file: {offset_path}")
    print(f"Joint signs:  {fmt(joint_signs, precision=1)}")
    print(f"Normalize:    {normalize}")
    print()
    print("Torque will remain OFF. This script does not connect to the UR follower.")

    driver = DynamixelDriver(joint_ids, servo_types, port)
    driver.set_torque_mode(False)

    sample_period = 1.0 / args.sample_rate
    try:
        raw_cal = prompt_and_record(
            driver,
            "CALIBRATION",
            calibration_q,
            args.samples,
            sample_period,
        )

        offsets = np.zeros(len(servo_types), dtype=float)
        arm_signs = joint_signs[:num_arm_joints]
        offsets[:num_arm_joints] = raw_cal[:num_arm_joints] - (
            calibration_q / arm_signs
        )
        offsets[-1] = raw_cal[-1]

        cal_reported = (
            raw_cal[:num_arm_joints] - offsets[:num_arm_joints]
        ) * arm_signs
        cal_error = cal_reported - calibration_q

        print()
        print("Computed offsets from calibration pose:")
        print(f"  offsets:       {fmt(offsets)}")
        print(f"  cal reported:  {fmt(cal_reported)}")
        print(f"  cal error:     {fmt(cal_error)}")

        raw_match = prompt_and_record(
            driver,
            "INITIAL MATCH",
            initial_match_q,
            args.samples,
            sample_period,
        )

        match_reported_raw_branch = (
            raw_match[:num_arm_joints] - offsets[:num_arm_joints]
        ) * arm_signs
        match_reported = match_reported_raw_branch.copy()
        if normalize:
            match_reported = normalize_to_reference(match_reported, initial_match_q)

        match_error = match_reported - initial_match_q
        match_norm = float(np.linalg.norm(match_error))
        expected_raw_match = offsets[:num_arm_joints] + (initial_match_q / arm_signs)
        raw_match_error = raw_match[:num_arm_joints] - expected_raw_match

        print()
        print("Initial-match verification using the calibration offsets:")
        print(f"  match reported before normalization: {fmt(match_reported_raw_branch)}")
        print(f"  match reported:                      {fmt(match_reported)}")
        print(f"  match target:                        {fmt(initial_match_q)}")
        print(f"  per-joint error:                     {fmt(match_error)}")
        print(f"  L2 match error:                      {match_norm:.4f}")
        print(f"  FACTR threshold:                     {args.match_threshold:.4f}")
        print(f"  expected raw at match pose:          {fmt(expected_raw_match)}")
        print(f"  raw match error:                     {fmt(raw_match_error)}")

        if match_norm <= args.match_threshold:
            print("Result: PASS. These offsets should pass the leader match gate.")
        else:
            print("Result: FAIL. The initial-match pose does not agree with calibration.")
            print("Check that both physical poses were placed correctly and that no joint slipped.")

        if not args.write:
            print()
            print("Dry run: offsets were not written. Re-run with --write to save them.")
            return

        if not args.yes:
            print()
            answer = input("Write these offsets to JSON? Type 'yes' to continue: ")
            if answer.strip().lower() != "yes":
                print("Not writing offsets.")
                return

        write_offsets(offset_path, offsets)
    finally:
        driver.set_torque_mode(False)
        driver.close()


if __name__ == "__main__":
    main()
