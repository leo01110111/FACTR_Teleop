#!/usr/bin/env python3
"""
Visualize the Pinocchio model used for leader-arm gravity compensation in
Meshcat, so its configuration can be checked against the real leader arm.

Two modes:

  Static (default): opens the model at a given joint configuration (zero by
  default, or --joints), so you can eyeball link placement, mesh alignment,
  and joint ordering against the physical arm held in the same pose.

  Live (--live): connects to the leader's Dynamixels (torque stays OFF) and
  continuously updates the Meshcat model to match the real, live joint
  readings, so you can move the physical arm by hand and watch the model
  track it. This uses sim_offsets_<leader_name>.json (produced by
  leader_sim_offset_calibrate.py), NOT offsets_<leader_name>.json -- the
  latter aligns the leader to the follower UR's joint convention, which has
  no reason to agree with the URDF's own zero pose.

  Sliders (--sliders): opens a small Tk window with one slider per arm
  joint, driven by the URDF's own joint limits, so you can jog the model
  interactively without touching hardware -- e.g. to pick a sim_target pose
  to hand to leader_sim_offset_calibrate.py. Click a slider to focus it, then
  the arrow keys nudge it by 0.1 for fine adjustment.

  Live tuning (--live --tune): like --live, but also opens a Tk window with
  an offset slider and a sign toggle per arm joint. Dragging them re-maps the
  live raw reading in real time, so you can dial in sim_offsets_
  <leader_name>.yaml interactively (instead of via leader_sim_offset_
  calibrate.py's single-pose capture) while watching the model track the
  physical arm. Click a slider to focus it, then the arrow keys nudge it by
  0.1 for fine adjustment. A Save button overwrites the YAML (after backing up
  the existing file, same as leader_sim_offset_calibrate.py).

Usage:
    python visualize_leader_urdf.py ur7e_leader_left.yaml
    python visualize_leader_urdf.py ur7e_leader_left.yaml --live
    python visualize_leader_urdf.py ur7e_leader_left.yaml --live --tune
    python visualize_leader_urdf.py ur7e_leader_left.yaml --sliders
    python visualize_leader_urdf.py ur7e_leader_left.yaml --joints 0.2 -1.4 0.3 0 0 0
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

import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer


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


def load_sim_offsets(config):
    """
    Raw-motor -> Pinocchio/URDF-convention offset and joint_signs, as produced
    by leader_sim_offset_calibrate.py. These are NOT the same as
    offsets_<leader_name>.json / dynamixel.joint_signs, which align raw motor
    readings to the follower UR's joint convention instead -- the URDF's own
    zero pose and joint axis directions have no reason to agree with the UR's
    calibration stance.
    """
    offset_path = sim_offset_path_for_config(config)
    if not os.path.isfile(offset_path):
        raise FileNotFoundError(
            f"No sim offsets file found at {offset_path}. Run "
            "leader_sim_offset_calibrate.py first to generate it."
        )
    with open(offset_path, "r") as f:
        sim_config = yaml.safe_load(f)
    offset = np.asarray(sim_config["offset"], dtype=float)
    joint_signs = np.asarray(sim_config["joint_signs"], dtype=float)
    return offset, joint_signs


def save_sim_offsets(offset_path, offsets, joint_signs):
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


def build_robot(config):
    urdf_path = os.path.join(
        WORKSPACE_ROOT,
        "src/factr_teleop/factr_teleop/urdf",
        config["arm_teleop"]["leader_urdf"],
    )
    urdf_dir = os.path.dirname(urdf_path)
    model, collision_model, visual_model = pin.buildModelsFromUrdf(
        urdf_path, package_dirs=[urdf_dir]
    )
    print(f"URDF:        {urdf_path}")
    print(f"Joint order: {list(model.names)[1:]}")  # skip 'universe'
    return model, visual_model


def bind_arrow_nudge(scale, step=0.1):
    """Make Left/Down nudge the focused Tk Scale down by `step` and Right/Up
    nudge it up, instead of the widget's default resolution-sized step. Also
    grabs keyboard focus on click so the arrow keys act on whichever slider was
    last touched."""

    def nudge(delta):
        scale.set(scale.get() + delta)  # .set() clamps to from_/to and rounds to resolution
        return "break"

    scale.bind("<Left>", lambda e: nudge(-step))
    scale.bind("<Down>", lambda e: nudge(-step))
    scale.bind("<Right>", lambda e: nudge(step))
    scale.bind("<Up>", lambda e: nudge(step))
    scale.bind("<Button-1>", lambda e: scale.focus_set(), add="+")


def run_sliders(viz, model, q, num_arm_joints, arm_joint_limits_min, arm_joint_limits_max):
    import tkinter as tk

    joint_names = list(model.names)[1 : 1 + num_arm_joints]

    root = tk.Tk()
    root.title("Leader sim joint sliders")

    values_label = tk.Label(root, font=("monospace", 10), justify="left")
    values_label.pack(padx=8, pady=(8, 0))

    def refresh_label():
        joints_str = " ".join(f"{v:.5f}" for v in q[:num_arm_joints])
        values_label.config(
            text=f"q = [{joints_str}]\n"
            f"leader_sim_offset_calibrate.py <config> {joints_str}"
        )

    def make_callback(i):
        def callback(value):
            q[i] = float(value)
            viz.display(q)
            refresh_label()

        return callback

    sliders = []
    for i in range(num_arm_joints):
        frame = tk.Frame(root)
        frame.pack(fill="x", padx=8, pady=4)
        tk.Label(frame, text=joint_names[i], width=16, anchor="w").pack(side="left")
        scale = tk.Scale(
            frame,
            from_=float(arm_joint_limits_min[i]),
            to=float(arm_joint_limits_max[i]),
            resolution=0.0005,
            orient="horizontal",
            length=320,
            command=make_callback(i),
        )
        bind_arrow_nudge(scale)
        scale.pack(side="left", fill="x", expand=True)
        sliders.append(scale)

    def reset():
        for scale in sliders:
            scale.set(0.0)

    tk.Button(root, text="Reset to zero", command=reset).pack(pady=(4, 8))

    refresh_label()
    print("Sliders window open. Close it (or Ctrl-C) to quit.")
    root.mainloop()


def run_live_tuning(
    viz,
    model,
    q,
    num_arm_joints,
    driver,
    offsets,
    joint_signs,
    period_ms,
    offset_path,
    offset_range=50.0,
):
    """
    Live-mirrors the leader's raw joint readings into the sim model (like
    plain --live), but with an offset slider and a sign toggle per arm joint
    that can be adjusted while it runs, and a Save button that overwrites
    sim_offsets_<leader_name>.yaml with the current offsets/joint_signs.
    """
    import tkinter as tk

    joint_names = list(model.names)[1 : 1 + num_arm_joints]

    root = tk.Tk()
    root.title("Leader sim live tuning")

    values_label = tk.Label(root, font=("monospace", 10), justify="left")
    values_label.pack(padx=8, pady=(8, 0))

    sign_buttons = []
    offset_scales = []
    for i in range(num_arm_joints):
        frame = tk.Frame(root)
        frame.pack(fill="x", padx=8, pady=4)
        tk.Label(frame, text=joint_names[i], width=16, anchor="w").pack(side="left")

        def make_offset_callback(idx):
            def callback(value):
                offsets[idx] = float(value)

            return callback

        scale = tk.Scale(
            frame,
            from_=-offset_range,
            to=offset_range,
            resolution=0.0005,
            orient="horizontal",
            length=280,
            command=make_offset_callback(i),
        )
        scale.set(float(offsets[i]))
        bind_arrow_nudge(scale)
        scale.pack(side="left", fill="x", expand=True)
        offset_scales.append(scale)

        sign_var = tk.StringVar(value=f"sign: {int(joint_signs[i]):+d}")

        def make_sign_callback(idx, var):
            def callback():
                joint_signs[idx] = -joint_signs[idx]
                var.set(f"sign: {int(joint_signs[idx]):+d}")

            return callback

        tk.Button(
            frame, textvariable=sign_var, width=9, command=make_sign_callback(i, sign_var)
        ).pack(side="left", padx=(6, 0))
        sign_buttons.append(sign_var)

    def save():
        # `offsets` is already the full num_motors-length array (with the
        # gripper's trailing entry untouched by the arm-joint sliders above).
        save_sim_offsets(offset_path, offsets, joint_signs[:num_arm_joints])

    tk.Button(root, text="Save to YAML", command=save).pack(pady=(4, 8))

    def poll():
        raw_pos, _ = driver.get_positions_and_velocities()
        arm_pos = (raw_pos[:num_arm_joints] - offsets[:num_arm_joints]) * joint_signs[
            :num_arm_joints
        ]
        q[:num_arm_joints] = arm_pos
        viz.display(q)
        joints_str = " ".join(f"{v:.5f}" for v in q[:num_arm_joints])
        offsets_str = " ".join(f"{v:.4f}" for v in offsets[:num_arm_joints])
        signs_str = " ".join(f"{int(v):+d}" for v in joint_signs[:num_arm_joints])
        values_label.config(
            text=f"q = [{joints_str}]\noffset = [{offsets_str}]\nsigns = [{signs_str}]"
        )
        root.after(period_ms, poll)

    root.after(period_ms, poll)
    print("Live tuning window open. Close it (or Ctrl-C) to quit.")
    root.mainloop()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "config", help="Config filename (resolved against the configs dir) or path."
    )
    parser.add_argument(
        "--live",
        action="store_true",
        help="Connect to the real leader Dynamixels and mirror live joint readings "
        "(same offset/sign convention as FACTRTeleop). Torque stays OFF.",
    )
    parser.add_argument(
        "--sliders",
        action="store_true",
        help="Open a Tk window with a slider per arm joint to jog the model "
        "interactively, bounded by the URDF's configured joint limits.",
    )
    parser.add_argument(
        "--joints",
        type=float,
        nargs="+",
        default=None,
        help="Static arm joint angles (rad), length num_arm_joints. Default: zeros.",
    )
    parser.add_argument(
        "--rate", type=float, default=20.0, help="Live update rate in Hz (default: 20)."
    )
    parser.add_argument(
        "--tune",
        action="store_true",
        help="With --live, open offset/sign sliders that re-map the live "
        "reading in real time, plus a Save button to overwrite "
        "sim_offsets_<leader_name>.yaml.",
    )
    parser.add_argument(
        "--offset-range",
        type=float,
        default=50.0,
        help="With --tune, +/- range of the offset sliders (rad). Default: 50.",
    )
    args = parser.parse_args()

    if args.live and args.sliders:
        raise ValueError("--live and --sliders are mutually exclusive.")
    if args.tune and not args.live:
        raise ValueError("--tune requires --live.")

    config_path = resolve_config_path(args.config)
    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    num_arm_joints = int(config["arm_teleop"]["num_arm_joints"])
    model, visual_model = build_robot(config)

    viz = MeshcatVisualizer(model, visual_model, visual_model)
    viz.initViewer(open=False)
    viz.loadViewerModel()
    print(f"Meshcat URL: {viz.viewer.url()}")

    q = pin.neutral(model)

    if args.live:
        from factr_teleop.dynamixel.driver import DynamixelDriver

        servo_types = config["dynamixel"]["servo_types"]
        joint_ids = np.arange(len(servo_types)) + 1
        port = "/dev/serial/by-id/" + config["dynamixel"]["dynamixel_port"]
        offsets, joint_signs = load_sim_offsets(config)
        offset_path = sim_offset_path_for_config(config)

        print(f"Port:        {port}")
        print("Torque is OFF -- move the leader by hand. Ctrl-C to quit.")

        driver = DynamixelDriver(joint_ids, servo_types, port)
        driver.set_torque_mode(False)
        try:
            if args.tune:
                run_live_tuning(
                    viz,
                    model,
                    q,
                    num_arm_joints,
                    driver,
                    offsets,
                    joint_signs,
                    period_ms=int(1000 / args.rate),
                    offset_path=offset_path,
                    offset_range=args.offset_range,
                )
            else:
                period = 1.0 / args.rate
                while True:
                    raw_pos, _ = driver.get_positions_and_velocities()
                    arm_pos = (
                        raw_pos[:num_arm_joints] - offsets[:num_arm_joints]
                    ) * joint_signs[:num_arm_joints]
                    q[:num_arm_joints] = arm_pos
                    viz.display(q)
                    time.sleep(period)
        except KeyboardInterrupt:
            print("\nStopped.")
        finally:
            driver.close()
    elif args.sliders:
        limits_min = np.asarray(config["arm_teleop"]["arm_joint_limits_min"], dtype=float)
        limits_max = np.asarray(config["arm_teleop"]["arm_joint_limits_max"], dtype=float)
        viz.display(q)
        try:
            run_sliders(viz, model, q, num_arm_joints, limits_min, limits_max)
        except KeyboardInterrupt:
            print("\nStopped.")
    else:
        if args.joints is not None:
            if len(args.joints) != num_arm_joints:
                raise ValueError(
                    f"--joints expects {num_arm_joints} values, got {len(args.joints)}"
                )
            q[:num_arm_joints] = args.joints
        viz.display(q)
        print(f"Displaying static configuration q = {q[:num_arm_joints].tolist()}")
        print("Open the Meshcat URL above in a browser. Ctrl-C to quit.")
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == "__main__":
    main()
