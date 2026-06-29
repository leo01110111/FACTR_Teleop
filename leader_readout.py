#!/usr/bin/env python3
"""
Continuously print the raw joint angles of the leader arm's motors (arm joints +
gripper) as a list. Torque is left OFF so the arm can be moved by hand.

This is a setup/diagnostic tool, e.g. for determining `joint_signs` or checking
that the gripper trigger is at its endstop before calibration. The values printed
are the RAW driver readings -- no offsets and no joint signs are applied.

Usage:
    python leader_readout.py <config>

    <config> may be a bare filename resolved against
    src/factr_teleop/factr_teleop/configs/ (e.g. ur7e_leader_left.yaml) or a path
    to a config file.
"""
import os
import sys
import time
import argparse

import numpy as np
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


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "config",
        help="Config filename (resolved against the configs dir) or path.",
    )
    parser.add_argument(
        "--rate", type=float, default=10.0, help="Print rate in Hz (default: 10).",
    )
    args = parser.parse_args()

    config_path = resolve_config_path(args.config)
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    servo_types = config["dynamixel"]["servo_types"]
    num_motors = len(servo_types)
    port = "/dev/serial/by-id/" + config["dynamixel"]["dynamixel_port"]
    joint_ids = np.arange(num_motors) + 1

    print(f"Config:   {config_path}")
    print(f"Port:     {port}")
    print(f"Motors:   {num_motors} ({num_motors - 1} arm + 1 gripper)")
    print("Torque is OFF -- move the arm by hand. Ctrl-C to quit.\n")

    driver = DynamixelDriver(joint_ids, servo_types, port)
    driver.set_torque_mode(False)

    period = 1.0 / args.rate
    try:
        while True:
            pos, _ = driver.get_positions_and_velocities()
            as_list = [round(float(p), 2) for p in pos]
            print(as_list, flush=True)
            time.sleep(period)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        driver.close()


if __name__ == "__main__":
    main()
