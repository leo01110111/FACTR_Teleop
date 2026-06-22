#!/usr/bin/env python3
"""
Live joint readout of the leader (GELLO) arm shown on tkinter sliders.

Connects to the leader arm's Dynamixel motors (torque OFF, so you move the arm by
hand) and continuously drives one slider per motor from the RAW driver readings --
no URDF, no offsets, no joint signs. The current q is also written to
/tmp/cal_pose.txt so you can capture a pose for calibration.

Usage:
    python viz_pose_slider.py <config> [--rate HZ]

    <config> may be a bare filename resolved against
    src/factr_teleop/factr_teleop/configs/ (e.g. ur7e_leader_left.yaml) or a path
    to a config file.
"""
import os
import argparse
import tkinter as tk

import numpy as np
import yaml

from python_utils.utils import get_workspace_root
from factr_teleop.dynamixel.driver import DynamixelDriver

OUT = "/tmp/cal_pose.txt"


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
        "--rate", type=float, default=20.0, help="Poll rate in Hz (default: 20).",
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
    print("Torque is OFF -- move the arm by hand. Close the window to quit.\n")

    driver = DynamixelDriver(joint_ids, servo_types, port)
    driver.set_torque_mode(False)

    root = tk.Tk()
    root.title("Leader (GELLO) live joint readout")
    val_label = tk.Label(root, text="", font=("monospace", 11), justify="left")
    val_label.pack(padx=8, pady=6)

    sliders = []
    for i in range(num_motors):
        label = f"motor {i + 1}" + (" (gripper)" if i == num_motors - 1 else "")
        row = tk.Frame(root)
        row.pack(fill="x", padx=8)
        tk.Label(row, text=label, width=18, anchor="w").pack(side="left")
        # Read-only display: sliders are driven by the arm, not the mouse.
        s = tk.Scale(row, from_=-6.2832, to=6.2832, resolution=0.01,
                     orient="horizontal", length=420, state="disabled",
                     takefocus=0)
        s.pack(side="left")
        sliders.append(s)

    period_ms = int(1000.0 / args.rate)

    def poll():
        pos, _ = driver.get_positions_and_velocities()
        for i, s in enumerate(sliders):
            # set() works even on a disabled Scale; clamp only the visual.
            s.configure(state="normal")
            s.set(float(np.clip(pos[i], -6.2832, 6.2832)))
            s.configure(state="disabled")
        txt = "  ".join(f"{v:+.3f}" for v in pos)
        val_label.config(text=f"q = [{txt}]")
        with open(OUT, "w") as f:
            f.write(" ".join(f"{v:.6f}" for v in pos) + "\n")
        root.after(period_ms, poll)

    print("Live readout running; q is written to", OUT, flush=True)
    poll()
    try:
        root.mainloop()
    finally:
        driver.close()


if __name__ == "__main__":
    main()
