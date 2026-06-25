#!/usr/bin/env python3
"""
Minimal leader-only gravity compensation test.

This opens only the GELLO/leader Dynamixels, loads the configured offsets and
URDF, computes Pinocchio gravity torque, and writes current commands to the six
arm motors. It does not connect RTDE, start ROS, move the follower, or apply
force feedback.

Usage:
    python leader_grav_comp_test.py ur7e_leader_right.yaml --gain -0.5
    python leader_grav_comp_test.py ur7e_leader_right.yaml --gains-file leader_grav_comp_gains_right.yaml
    python leader_grav_comp_test.py ur7e_leader_right.yaml --gains 0 0 -1.0 0 0 0
    python leader_grav_comp_test.py ur7e_leader_right.yaml --gain -1.0 --yes

Use Ctrl-C to stop; the script zeros current and disables torque on exit.
"""
import argparse
import json
import os
import time

import numpy as np
import pinocchio as pin
import yaml

from python_utils.utils import get_workspace_root
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


def load_offsets(config):
    offset_filename = f"offsets_{config['dynamixel']['leader_name']}.json"
    offset_path = os.path.join(
        get_workspace_root(),
        "src/factr_teleop/factr_teleop/configs",
        offset_filename,
    )
    with open(offset_path, "r") as f:
        return np.array(json.load(f), dtype=float), offset_path


def build_pin_model(config):
    workspace_root = get_workspace_root()
    urdf_path = os.path.join(
        workspace_root,
        "src/factr_teleop/factr_teleop/urdf",
        config["arm_teleop"]["leader_urdf"],
    )
    urdf_dir = os.path.dirname(urdf_path)
    model, _, _ = pin.buildModelsFromUrdf(filename=urdf_path, package_dirs=urdf_dir)
    return model, model.createData(), urdf_path


def normalize_to_reference(q, reference):
    return reference + np.arctan2(np.sin(q - reference), np.cos(q - reference))


def resolve_optional_path(path_arg):
    if path_arg is None:
        return None
    if os.path.isfile(path_arg):
        return path_arg
    candidate = os.path.join(get_workspace_root(), path_arg)
    if os.path.isfile(candidate):
        return candidate
    raise FileNotFoundError(
        f"Could not find '{path_arg}' (also tried '{candidate}')."
    )


def load_gain_vector(args, config, num_arm_joints):
    if args.gains_file is not None:
        gains_path = resolve_optional_path(args.gains_file)
        with open(gains_path, "r") as f:
            gains_config = yaml.safe_load(f)
        if isinstance(gains_config, dict):
            gains = gains_config.get("gains")
        else:
            gains = gains_config
        source = gains_path
    elif args.gains is not None:
        gains = args.gains
        source = "--gains"
    else:
        scalar_gain = (
            float(args.gain)
            if args.gain is not None
            else float(config["controller"]["gravity_comp"]["gain"])
        )
        gains = [scalar_gain] * num_arm_joints
        source = "--gain/config scalar"

    gains = np.array(gains, dtype=float)
    if gains.shape != (num_arm_joints,):
        raise ValueError(
            f"Expected {num_arm_joints} gravity gains, got shape {gains.shape}"
        )
    return gains, source


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", help="Config filename or path.")
    parser.add_argument(
        "--gain",
        type=float,
        default=None,
        help="Gravity compensation gain. Defaults to controller.gravity_comp.gain.",
    )
    parser.add_argument(
        "--gains",
        nargs=6,
        type=float,
        default=None,
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
        help="Six per-joint gravity gains. Overrides --gain.",
    )
    parser.add_argument(
        "--gains-file",
        default=None,
        help="YAML file containing `gains: [j1, j2, j3, j4, j5, j6]`.",
    )
    parser.add_argument("--rate", type=float, default=100.0, help="Loop rate in Hz.")
    parser.add_argument(
        "--print-rate", type=float, default=2.0, help="Status print rate in Hz."
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Compute and print torques without enabling torque or writing current.",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="Skip the Enter-to-enable prompt.",
    )
    args = parser.parse_args()

    config_path = resolve_config_path(args.config)
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    num_arm_joints = int(config["arm_teleop"]["num_arm_joints"])
    gains, gains_source = load_gain_vector(args, config, num_arm_joints)
    servo_types = config["dynamixel"]["servo_types"]
    joint_ids = np.arange(len(servo_types)) + 1
    joint_signs = np.array(config["dynamixel"]["joint_signs"], dtype=float)
    port = "/dev/serial/by-id/" + config["dynamixel"]["dynamixel_port"]
    offsets, offset_path = load_offsets(config)
    pin_model, pin_data, urdf_path = build_pin_model(config)
    reference = np.array(
        config["arm_teleop"]["initialization"]["initial_match_joint_pos"],
        dtype=float,
    )
    normalize = bool(
        config["arm_teleop"]["initialization"].get("normalize_joint_angles", False)
    )

    print(f"Config:  {config_path}")
    print(f"Offsets: {offset_path}")
    print(f"URDF:    {urdf_path}")
    print(f"Port:    {port}")
    print(f"Gains:   {np.array2string(gains, precision=3, floatmode='fixed')}")
    print(f"Source:  {gains_source}")
    print("No RTDE, no follower, no force feedback.")
    if args.dry_run:
        print("Dry run: torque will remain OFF.\n")
    elif not args.yes:
        input("Press Enter to enable leader torque and start gravity comp...")

    driver = DynamixelDriver(joint_ids, servo_types, port)
    if args.dry_run:
        driver.set_torque_mode(False)
    else:
        driver.set_torque_mode(False)
        driver.set_operating_mode(0)
        driver.set_torque_mode(True)
        driver.set_torque(np.zeros(len(servo_types)))

    period = 1.0 / args.rate
    print_period = 1.0 / args.print_rate
    next_print = time.monotonic()
    zeros = np.zeros(num_arm_joints)

    try:
        while True:
            raw_pos, _ = driver.get_positions_and_velocities()
            q = (raw_pos[:num_arm_joints] - offsets[:num_arm_joints]) * joint_signs[
                :num_arm_joints
            ]
            if normalize:
                q = normalize_to_reference(q, reference)

            tau_model = pin.rnea(pin_model, pin_data, q, zeros, zeros)
            tau = tau_model * gains
            cmd_torque = np.append(tau, 0.0) * joint_signs
            if not args.dry_run:
                driver.set_torque(cmd_torque)

            now = time.monotonic()
            if now >= next_print:
                print(
                    "q="
                    + np.array2string(q, precision=2, floatmode="fixed")
                    + " tau_model="
                    + np.array2string(tau_model, precision=2, floatmode="fixed")
                    + " tau="
                    + np.array2string(tau, precision=2, floatmode="fixed"),
                    flush=True,
                )
                next_print = now + print_period
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        try:
            if not args.dry_run:
                driver.set_torque(np.zeros(len(servo_types)))
                time.sleep(0.05)
            driver.set_torque_mode(False)
        finally:
            driver.close()


if __name__ == "__main__":
    main()
