#!/usr/bin/env python3
"""
Anchor a leader arm at its init (initial_match) pose and tune its offsets.

The GELLO/leader Dynamixels are made weightless with gravity compensation
(pin.rnea against the URDF) and nothing else -- no PD -- so you can freely
hand-move the leader to whatever anchor pose you want and it stays there. The
offset sliders never drive the arm; you position it by hand.

A Tk window shows one continuous (non-snapping) offset slider per arm joint,
editing offsets_<leader_name>.json -- the follower-alignment offsets that map
raw motor readings to the follower UR's joint convention. Click a slider to
focus it, then the arrow keys nudge by 0.01 rad. As you slide, the mapped
follower/UR target (raw - offset)*sign changes (shown in the window) even
though the leader itself stays put. Tune until the UR target reads the pose you
want, then Save to overwrite offsets_<leader_name>.json (existing file backed
up first).

Gravity compensation uses sim_offsets_<leader_name>.yaml (the URDF-convention
offset/signs from leader_sim_offset_calibrate.py), independent of the
follower offsets being tuned here.

This script does NOT command the UR follower; it only holds the leader and
displays the UR target. (Ask to wire in RTDE servoJ if you want the real UR to
move as you tune.)

Usage:
    python leader_init_pose_tune.py --left
    python leader_init_pose_tune.py --right
    python leader_init_pose_tune.py ur7e_leader_right.yaml --yes

Use Ctrl-C or close the window to stop; torque is zeroed and disabled on exit.
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

import pinocchio as pin  # noqa: E402
from factr_teleop.dynamixel.driver import DynamixelDriver  # noqa: E402


LEADER_CONFIGS = {
    "--left": "ur7e_leader_left.yaml",
    "-l": "ur7e_leader_left.yaml",
    "left": "ur7e_leader_left.yaml",
    "--right": "ur7e_leader_right.yaml",
    "-r": "ur7e_leader_right.yaml",
    "right": "ur7e_leader_right.yaml",
}


def resolve_config_path(config_arg):
    config_arg = LEADER_CONFIGS.get(config_arg, config_arg)
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


def offset_path_for_config(config):
    offset_filename = f"offsets_{config['dynamixel']['leader_name']}.json"
    return os.path.join(
        WORKSPACE_ROOT, "src/factr_teleop/factr_teleop/configs", offset_filename
    )


def load_offsets(offset_path, num_motors):
    if os.path.isfile(offset_path):
        with open(offset_path, "r") as f:
            return np.array(json.load(f), dtype=float)
    return np.zeros(num_motors, dtype=float)


def save_offsets(offset_path, offsets):
    if os.path.exists(offset_path):
        backup_path = f"{offset_path}.bak.{time.strftime('%Y%m%d_%H%M%S')}"
        shutil.copy2(offset_path, backup_path)
        print(f"Backed up existing offsets to: {backup_path}")
    with open(offset_path, "w") as f:
        json.dump([float(x) for x in offsets], f)
    print(f"Wrote offsets to: {offset_path}")


def load_sim_offsets(config):
    """URDF-convention offset + joint_signs used for gravity comp (see
    leader_sim_offset_calibrate.py). Distinct from offsets_<leader>.json."""
    filename = f"sim_offsets_{config['dynamixel']['leader_name']}.yaml"
    path = os.path.join(WORKSPACE_ROOT, "src/factr_teleop/factr_teleop/configs", filename)
    if not os.path.isfile(path):
        raise FileNotFoundError(
            f"No sim offsets file found at {path}. Run leader_sim_offset_calibrate.py "
            "first -- gravity compensation needs it to map raw readings into the "
            "URDF's own joint convention."
        )
    with open(path, "r") as f:
        sim_config = yaml.safe_load(f)
    return (
        np.array(sim_config["offset"], dtype=float),
        np.array(sim_config["joint_signs"], dtype=float),
    )


def build_pin_model(config):
    urdf_path = os.path.join(
        WORKSPACE_ROOT,
        "src/factr_teleop/factr_teleop/urdf",
        config["arm_teleop"]["leader_urdf"],
    )
    model = pin.buildModelFromUrdf(urdf_path)
    return model, model.createData()


def normalize_to_reference(q, reference):
    return reference + np.arctan2(np.sin(q - reference), np.cos(q - reference))


def bind_arrow_nudge(scale, step=0.01):
    """Left/Down nudge the focused Tk Scale down by `step` rad, Right/Up nudge
    up; click grabs keyboard focus so the arrows act on the last-touched
    slider. With resolution=-1 the value is not snapped, so steps are exact."""

    def nudge(delta):
        scale.set(scale.get() + delta)  # clamps to from_/to; no rounding (resolution=-1)
        return "break"

    scale.bind("<Left>", lambda e: nudge(-step))
    scale.bind("<Down>", lambda e: nudge(-step))
    scale.bind("<Right>", lambda e: nudge(step))
    scale.bind("<Up>", lambda e: nudge(step))
    scale.bind("<Button-1>", lambda e: scale.focus_set(), add="+")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "config",
        nargs="?",
        default=None,
        help="A config filename (resolved against the configs dir) or path. "
        "Alternatively use --left/--right.",
    )
    side = parser.add_mutually_exclusive_group()
    side.add_argument(
        "--left", "-l", dest="side", action="store_const", const="ur7e_leader_left.yaml",
        help="Shortcut for the left leader config.",
    )
    side.add_argument(
        "--right", "-r", dest="side", action="store_const", const="ur7e_leader_right.yaml",
        help="Shortcut for the right leader config.",
    )
    parser.add_argument(
        "--rate", type=float, default=100.0, help="Control/update loop rate in Hz (default: 100)."
    )
    parser.add_argument(
        "--window",
        type=float,
        default=np.pi,
        help="+/- range each offset slider spans around its loaded value (rad). Default: pi.",
    )
    parser.add_argument(
        "--yes", action="store_true", help="Skip the Enter-to-enable-torque prompt."
    )
    args = parser.parse_args()

    config_arg = args.side or args.config
    if config_arg is None:
        parser.error("provide a config, or one of --left/--right.")

    config_path = resolve_config_path(config_arg)
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    num_arm_joints = int(config["arm_teleop"]["num_arm_joints"])
    servo_types = config["dynamixel"]["servo_types"]
    num_motors = len(servo_types)
    joint_ids = np.arange(num_motors) + 1
    joint_signs = np.array(config["dynamixel"]["joint_signs"], dtype=float)
    arm_signs = joint_signs[:num_arm_joints]
    port = "/dev/serial/by-id/" + config["dynamixel"]["dynamixel_port"]

    offset_path = offset_path_for_config(config)
    offsets = load_offsets(offset_path, num_motors)

    sim_offsets, sim_joint_signs = load_sim_offsets(config)
    sim_arm_signs = sim_joint_signs[:num_arm_joints]
    pin_model, pin_data = build_pin_model(config)

    init_pose = np.array(
        config["arm_teleop"]["initialization"]["initial_match_joint_pos"], dtype=float
    )[:num_arm_joints]
    normalize = bool(
        config["arm_teleop"]["initialization"].get("normalize_joint_angles", False)
    )
    # Per-joint gravity-comp gains straight from the selected config, same as
    # the main teleop's controller.gravity_comp.gain.
    grav_gain = np.array(config["controller"]["gravity_comp"]["gain"], dtype=float)[
        :num_arm_joints
    ]

    print(f"Config:      {config_path}")
    print(f"Offsets:     {offset_path}")
    print(f"Port:        {port}")
    print(f"Init pose:   {np.array2string(init_pose, precision=3, floatmode='fixed')}")
    print(f"Grav gains:  {np.array2string(grav_gain, precision=3, floatmode='fixed')}")
    print("The leader floats under gravity comp -- hand-move it to your anchor pose.")
    print("Sliding an offset changes the UR target only; the leader is not driven. Save when matched.")
    if not args.yes:
        input("Press Enter to enable leader torque (gravity comp only)...")

    driver = DynamixelDriver(joint_ids, servo_types, port)
    driver.set_torque_mode(False)
    driver.set_operating_mode(0)  # current control
    driver.set_torque_mode(True)
    driver.set_torque(np.zeros(num_motors))

    zeros = np.zeros(num_arm_joints)

    def hold_and_read():
        """Read state, command gravity compensation only (no PD, so the arm is
        weightless and hand-movable), and return raw_pos."""
        raw_pos, _ = driver.get_positions_and_velocities()
        raw_pos = raw_pos[:num_arm_joints]

        # Gravity comp: rnea in the URDF convention, converted to motor torque
        # by *sim_joint_signs (matches leader_grav_comp_test.py).
        q_sim = (raw_pos - sim_offsets[:num_arm_joints]) * sim_arm_signs
        tau_grav = pin.rnea(pin_model, pin_data, q_sim, zeros, zeros) * sim_arm_signs
        tau_grav *= grav_gain

        driver.set_torque(np.append(tau_grav, 0.0))
        return raw_pos

    import tkinter as tk

    root = tk.Tk()
    root.title("Leader init-pose offset tuning")

    values_label = tk.Label(root, font=("monospace", 10), justify="left")
    values_label.pack(padx=8, pady=(8, 0))

    for i in range(num_arm_joints):
        frame = tk.Frame(root)
        frame.pack(fill="x", padx=8, pady=4)
        tk.Label(frame, text=f"joint {i} offset", width=16, anchor="w").pack(side="left")

        def make_offset_callback(idx):
            def callback(value):
                offsets[idx] = float(value)

            return callback

        loaded = float(offsets[i])
        scale = tk.Scale(
            frame,
            from_=loaded - args.window,
            to=loaded + args.window,
            resolution=-1,  # no snapping: full floating-point resolution
            orient="horizontal",
            length=320,
            command=make_offset_callback(i),
        )
        scale.set(loaded)
        bind_arrow_nudge(scale)
        scale.pack(side="left", fill="x", expand=True)

    def save():
        # offsets is the full num_motors-length array; the gripper entry
        # (index -1) is left at its loaded value since no slider edits it.
        save_offsets(offset_path, offsets)

    tk.Button(root, text="Save to JSON", command=save).pack(pady=(4, 8))

    period_ms = int(1000 / args.rate)

    def poll():
        raw_pos = hold_and_read()

        # Mapped follower/UR target from the CURRENT slider offsets.
        ur_target = (raw_pos - offsets[:num_arm_joints]) * arm_signs
        if normalize:
            ur_target = normalize_to_reference(ur_target, init_pose)

        ur_str = " ".join(f"{v:.4f}" for v in ur_target)
        offsets_str = " ".join(f"{v:.4f}" for v in offsets[:num_arm_joints])
        err = float(np.linalg.norm(ur_target - init_pose))
        values_label.config(
            text=(
                f"UR target = [{ur_str}]  (|target-init|={err:.3f})\n"
                f"init pose = [{' '.join(f'{v:.4f}' for v in init_pose)}]\n"
                f"offsets   = [{offsets_str}]"
            )
        )
        root.after(period_ms, poll)

    root.after(period_ms, poll)
    print("Tuning window open. Adjust sliders, Save when matched. Ctrl-C / close to quit.")
    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("\nStopping.")
    finally:
        try:
            driver.set_torque(np.zeros(num_motors))
            driver.set_torque_mode(False)
        finally:
            driver.close()


if __name__ == "__main__":
    main()
