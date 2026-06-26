#!/usr/bin/env python3
"""Build a MaxLab dual-UR visual USD with Isaac's Robotiq 2F-85 asset.

This is a visualization/asset assembly helper. The real cuMotion RMPFlow
runtime uses URDF/XRDF, not this USD.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np

from view_cumotion_scene import (
    DEFAULT_SCENE_METADATA,
    REPO_DIR,
    _apply_transform,
    _apply_usd_joint_pose,
    _load_yaml,
)


DEFAULT_SOURCE_USD = REPO_DIR / "generated/isaac_cumotion/usd/maxlab_dual_ur7e_table/maxlab_dual_ur7e_table.usda"
DEFAULT_GRIPPER_USD = Path(
    "/home/srianumakonda/anaconda3/envs/env_isaaclab6/lib/python3.12/site-packages/"
    "isaacsim/exts/isaacsim.asset.transformer.rules/data/tests/ur10e/Robotiq/2F-85/Robotiq_2F_85.usda"
)
DEFAULT_OUTPUT_USD = (
    REPO_DIR
    / "generated/isaac_cumotion/usd/maxlab_dual_ur7e_table_isaac_robotiq/"
    "maxlab_dual_ur7e_table_isaac_robotiq.usda"
)

SIDES = ("left", "right")


def _xform_matrix(prim):
    from pxr import UsdGeom

    result = UsdGeom.Xformable(prim).GetLocalTransformation()
    return result[0] if isinstance(result, tuple) else result


def _usd_matrix_to_np(matrix) -> np.ndarray:
    usd_matrix = np.asarray([[float(matrix[i][j]) for j in range(4)] for i in range(4)], dtype=np.float64)
    return usd_matrix.T


def _axis_angle(axis: tuple[float, float, float], angle_rad: float) -> np.ndarray:
    axis_arr = np.asarray(axis, dtype=np.float64)
    axis_arr /= np.linalg.norm(axis_arr)
    x, y, z = axis_arr
    c, s = math.cos(angle_rad), math.sin(angle_rad)
    C = 1.0 - c
    out = np.eye(4, dtype=np.float64)
    out[:3, :3] = np.asarray(
        [
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
        ],
        dtype=np.float64,
    )
    return out


def _rpy_transform(roll: float, pitch: float, yaw: float) -> np.ndarray:
    return (
        _axis_angle((1.0, 0.0, 0.0), roll)
        @ _axis_angle((0.0, 1.0, 0.0), pitch)
        @ _axis_angle((0.0, 0.0, 1.0), yaw)
    )


def _rotation_matrix_to_quat_wxyz(rotation: np.ndarray) -> tuple[float, float, float, float]:
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return 0.25 * s, (rotation[2, 1] - rotation[1, 2]) / s, (rotation[0, 2] - rotation[2, 0]) / s, (
            rotation[1, 0] - rotation[0, 1]
        ) / s
    if rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        return (rotation[2, 1] - rotation[1, 2]) / s, 0.25 * s, (rotation[0, 1] + rotation[1, 0]) / s, (
            rotation[0, 2] + rotation[2, 0]
        ) / s
    if rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        return (rotation[0, 2] - rotation[2, 0]) / s, (rotation[0, 1] + rotation[1, 0]) / s, 0.25 * s, (
            rotation[1, 2] + rotation[2, 1]
        ) / s
    s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
    return (rotation[1, 0] - rotation[0, 1]) / s, (rotation[0, 2] + rotation[2, 0]) / s, (
        rotation[1, 2] + rotation[2, 1]
    ) / s, 0.25 * s


def _add_fixed_gripper_joint(stage, joint_path: str, wrist_path: str, gripper_path: str, attach_transform: np.ndarray) -> None:
    from pxr import Gf, UsdPhysics

    joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    joint.CreateBody0Rel().SetTargets([wrist_path])
    joint.CreateBody1Rel().SetTargets([f"{gripper_path}/base_link"])
    joint.CreateLocalPos0Attr().Set(Gf.Vec3f(*(float(v) for v in attach_transform[:3, 3])))
    w, x, y, z = _rotation_matrix_to_quat_wxyz(attach_transform[:3, :3])
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(float(w), float(x), float(y), float(z)))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))


def _side_wrist_path(root: str, side: str) -> str:
    return (
        f"{root}/Geometry/{side}_robot_mount/{side}_base/{side}_shoulder_link/"
        f"{side}_upper_arm_link/{side}_forearm_link/{side}_wrist_1_link/"
        f"{side}_wrist_2_link/{side}_wrist_3_link"
    )


def _make_invisible(stage, path: str) -> None:
    from pxr import UsdGeom

    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        UsdGeom.Imageable(prim).MakeInvisible()


def _reference_stage(stage, path: str, usd_path: Path, physics_variant: str | None) -> None:
    from pxr import UsdGeom

    xform = UsdGeom.Xform.Define(stage, path)
    prim = xform.GetPrim()
    prim.GetReferences().AddReference(str(usd_path.resolve()))
    if physics_variant:
        prim.GetVariantSets().GetVariantSet("Physics").SetVariantSelection(physics_variant)


def _attach_isaac_gripper(
    stage,
    *,
    source_root: str,
    visual_root: str,
    side: str,
    gripper_usd: Path,
    adjust_rpy: tuple[float, float, float],
    physics_variant: str | None,
) -> str:
    wrist_source_path = _side_wrist_path(source_root, side)
    wrist_stage_path = _side_wrist_path(visual_root, side)
    old_mount_stage_path = f"{wrist_stage_path}/{side}_grip_base_mount"
    new_gripper_path = f"{wrist_stage_path}/{side}_isaac_robotiq_2f85"

    old_mount = stage.GetPrimAtPath(old_mount_stage_path)
    if not old_mount.IsValid():
        raise RuntimeError(f"missing old gripper mount path: {old_mount_stage_path}")

    attach_transform = _usd_matrix_to_np(_xform_matrix(old_mount)) @ _rpy_transform(*adjust_rpy)
    _make_invisible(stage, old_mount_stage_path)

    _reference_stage(stage, new_gripper_path, gripper_usd, physics_variant=physics_variant)
    _apply_transform(stage.GetPrimAtPath(new_gripper_path), attach_transform)
    if physics_variant is not None and physics_variant != "None":
        _add_fixed_gripper_joint(
            stage,
            f"{new_gripper_path}/{side}_robotiq_wrist_fixed_joint",
            wrist_stage_path,
            new_gripper_path,
            attach_transform,
        )

    # Keep the source path in the error text if the source USD hierarchy changes.
    if not stage.GetPrimAtPath(wrist_stage_path).IsValid():
        raise RuntimeError(f"missing assembled wrist path {wrist_stage_path}; source was {wrist_source_path}")
    return new_gripper_path


def build(args: argparse.Namespace) -> Path:
    from pxr import Sdf, Usd, UsdGeom

    source_usd = args.source_usd.resolve()
    gripper_usd = args.gripper_usd.resolve()
    output_usd = args.output_usd.resolve()
    if not source_usd.exists():
        raise FileNotFoundError(source_usd)
    if not gripper_usd.exists():
        raise FileNotFoundError(gripper_usd)

    stage = Usd.Stage.CreateNew(str(output_usd))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    visual_root = "/maxlab_visual_usd"
    root = UsdGeom.Xform.Define(stage, visual_root)
    stage.SetDefaultPrim(root.GetPrim())
    _reference_stage(stage, visual_root, source_usd, physics_variant="none")

    scene = _load_yaml(args.scene_metadata.resolve())
    _apply_usd_joint_pose(
        stage,
        visual_root,
        source_usd,
        scene,
        args.usd_pose,
        simulation_app=None,
        gripper_fraction=0.0,
    )

    source_stage = Usd.Stage.Open(str(source_usd))
    source_root = str(source_stage.GetDefaultPrim().GetPath())
    adjust_rpy = tuple(math.radians(value) for value in args.gripper_adjust_rpy_deg)
    attached_paths = [
        _attach_isaac_gripper(
            stage,
            source_root=source_root,
            visual_root=visual_root,
            side=side,
            gripper_usd=gripper_usd,
            adjust_rpy=adjust_rpy,
            physics_variant="None",
        )
        for side in SIDES
    ]

    output_usd.parent.mkdir(parents=True, exist_ok=True)
    stage.GetRootLayer().Save()
    print(f"wrote {output_usd}")
    print("attached Isaac Robotiq 2F-85:")
    for path in attached_paths:
        print(f"  {path}")
    print(f"source gripper: {gripper_usd}")
    return output_usd


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-usd", type=Path, default=DEFAULT_SOURCE_USD)
    parser.add_argument("--gripper-usd", type=Path, default=DEFAULT_GRIPPER_USD)
    parser.add_argument("--scene-metadata", type=Path, default=DEFAULT_SCENE_METADATA)
    parser.add_argument("--output-usd", type=Path, default=DEFAULT_OUTPUT_USD)
    parser.add_argument("--usd-pose", choices=("initial_match", "sim_home"), default="initial_match")
    parser.add_argument(
        "--gripper-adjust-rpy-deg",
        nargs=3,
        type=float,
        default=(0.0, 0.0, 90.0),
        metavar=("ROLL", "PITCH", "YAW"),
        help="Additional local RPY rotation after the original wrist-to-gripper mount transform.",
    )
    return parser.parse_args()


def main() -> None:
    build(parse_args())


if __name__ == "__main__":
    main()
