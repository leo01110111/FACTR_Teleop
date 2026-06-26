#!/usr/bin/env python3
"""Visualize the MaxLab UR7e cuMotion scene and XRDF collision spheres."""

from __future__ import annotations

import argparse
import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Iterable

import numpy as np
import yaml


REPO_DIR = Path("/home/srianumakonda/FACTR_Teleop")
DEFAULT_CONFIG_DIR = REPO_DIR / "configs/isaac_cumotion/maxlab_ur7e_right"
DEFAULT_SCENE_METADATA = REPO_DIR / "configs/isaac_cumotion/maxlab_ur7e_scene.yaml"

JOINT_NAMES = (
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
)

ROBOTIQ_OPEN_JOINT_DEGREES = {
    "grip_left_driver_joint": 0.0,
    "grip_right_driver_joint": 0.0,
    "grip_left_coupler_joint": 0.0,
    "grip_right_coupler_joint": 0.0,
    "grip_left_follower_joint": 0.0,
    "grip_right_follower_joint": 0.0,
    "grip_left_spring_link_joint": 0.0,
    "grip_right_spring_link_joint": 0.0,
}

ROBOTIQ_JOINT_CLOSE_SCALE = {
    "grip_left_driver_joint": 1.0,
    "grip_right_driver_joint": 1.0,
    "grip_left_coupler_joint": -2.0,
    "grip_right_coupler_joint": -2.0,
    "grip_left_follower_joint": 1.0,
    "grip_right_follower_joint": 1.0,
    "grip_left_spring_link_joint": 1.0,
    "grip_right_spring_link_joint": 1.0,
}

DEFAULT_ROBOTIQ_CLOSE_DEG = 42.0
SIDE_COLORS = {
    "left": (0.1, 0.35, 1.0),
    "right": (1.0, 0.22, 0.18),
}


def _load_yaml(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"expected YAML mapping in {path}")
    return data


def _yaw_rotation(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.asarray([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def _axis_angle_rotation(axis: Iterable[float], angle: float) -> np.ndarray:
    axis = np.asarray(list(axis), dtype=np.float64)
    norm = np.linalg.norm(axis)
    if norm <= 0.0:
        return np.eye(3, dtype=np.float64)
    x, y, z = axis / norm
    c, s = math.cos(angle), math.sin(angle)
    C = 1.0 - c
    return np.asarray(
        [
            [c + x * x * C, x * y * C - z * s, x * z * C + y * s],
            [y * x * C + z * s, c + y * y * C, y * z * C - x * s],
            [z * x * C - y * s, z * y * C + x * s, c + z * z * C],
        ],
        dtype=np.float64,
    )


def _rpy_rotation(rpy: Iterable[float]) -> np.ndarray:
    roll, pitch, yaw = np.asarray(list(rpy), dtype=np.float64)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.asarray([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]], dtype=np.float64)
    ry = np.asarray([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]], dtype=np.float64)
    rz = np.asarray([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)
    return rz @ ry @ rx


def _xyz_rpy_transform(xyz: Iterable[float], rpy: Iterable[float]) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = _rpy_rotation(rpy)
    transform[:3, 3] = np.asarray(list(xyz), dtype=np.float64)
    return transform


def _rotation_transform(rotation: np.ndarray) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    return transform


def _translation_transform(translation: Iterable[float]) -> np.ndarray:
    transform = np.eye(4, dtype=np.float64)
    transform[:3, 3] = np.asarray(list(translation), dtype=np.float64)
    return transform


def _quat_wxyz_rotation(quat: Iterable[float]) -> np.ndarray:
    w, x, y, z = np.asarray(list(quat), dtype=np.float64)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm <= 0.0 or not math.isfinite(norm):
        raise ValueError("invalid quaternion")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _base_transform(base_config: dict) -> tuple[np.ndarray, np.ndarray]:
    transform = base_config.get("world_from_backend_base", base_config)
    translation = np.asarray(transform["pos"], dtype=np.float64)
    if "quat_wxyz" in transform:
        return _quat_wxyz_rotation(transform["quat_wxyz"]), translation
    return _yaw_rotation(float(transform["yaw_rad"])), translation


def _world_from_base_transform(base_config: dict) -> np.ndarray:
    rotation, translation = _base_transform(base_config)
    transform = np.eye(4, dtype=np.float64)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def _parse_float_list(value: str | None, default: tuple[float, ...]) -> np.ndarray:
    if value is None:
        return np.asarray(default, dtype=np.float64)
    return np.asarray([float(part) for part in value.split()], dtype=np.float64)


def _usd_matrix(transform: np.ndarray):
    from pxr import Gf

    # USD matrices are row-major with translation stored in the final row.
    usd_transform = transform.T
    return Gf.Matrix4d(*(float(value) for value in usd_transform.reshape(-1)))


def _set_display_color(prim, color, opacity: float = 1.0):
    from pxr import UsdGeom

    gprim = UsdGeom.Gprim(prim)
    gprim.CreateDisplayColorAttr([tuple(float(v) for v in color)])
    gprim.CreateDisplayOpacityAttr([float(opacity)])


def _make_preview_material(stage, path: str, color, opacity: float):
    from pxr import Gf, Sdf, UsdShade

    material = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        Gf.Vec3f(float(color[0]), float(color[1]), float(color[2]))
    )
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(opacity))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.45)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def _bind_material(prim, material) -> None:
    from pxr import UsdShade

    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)


def _apply_transform(prim, transform: np.ndarray) -> None:
    from pxr import UsdGeom

    xformable = UsdGeom.Xformable(prim)
    xformable.ClearXformOpOrder()
    xformable.AddTransformOp().Set(_usd_matrix(transform))


def _add_sphere(stage, path: str, center, radius: float, color, opacity: float = 1.0, material=None) -> None:
    from pxr import UsdGeom

    sphere = UsdGeom.Sphere.Define(stage, path)
    sphere.CreateRadiusAttr(float(radius))
    sphere.AddTranslateOp().Set(tuple(float(v) for v in center))
    _set_display_color(sphere.GetPrim(), color, opacity)
    if material is not None:
        _bind_material(sphere.GetPrim(), material)


def _add_cube(stage, path: str, center, scale, color, opacity: float = 1.0) -> None:
    from pxr import UsdGeom

    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(tuple(float(v) for v in center))
    cube.AddScaleOp().Set(tuple(float(v) for v in scale))
    _set_display_color(cube.GetPrim(), color, opacity)


def _add_box_visual(stage, path: str, transform: np.ndarray, size, color, opacity: float) -> None:
    from pxr import UsdGeom

    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    scale = np.eye(4, dtype=np.float64)
    scale[0, 0], scale[1, 1], scale[2, 2] = np.asarray(size, dtype=np.float64)
    _apply_transform(cube.GetPrim(), transform @ scale)
    _set_display_color(cube.GetPrim(), color, opacity)


def _add_cylinder_visual(stage, path: str, transform: np.ndarray, radius: float, length: float, color, opacity: float) -> None:
    from pxr import UsdGeom

    cylinder = UsdGeom.Cylinder.Define(stage, path)
    cylinder.CreateRadiusAttr(float(radius))
    cylinder.CreateHeightAttr(float(length))
    _apply_transform(cylinder.GetPrim(), transform)
    _set_display_color(cylinder.GetPrim(), color, opacity)


def _add_visual_sphere(stage, path: str, transform: np.ndarray, radius: float, color, opacity: float) -> None:
    from pxr import UsdGeom

    sphere = UsdGeom.Sphere.Define(stage, path)
    sphere.CreateRadiusAttr(float(radius))
    _apply_transform(sphere.GetPrim(), transform)
    _set_display_color(sphere.GetPrim(), color, opacity)


def _add_frame_marker(stage, path: str, center, color) -> None:
    _add_sphere(stage, f"{path}/base", center, 0.045, color)


def _add_usd_reference(stage, path: str, usd_path: Path, physics_variant: str = "none") -> None:
    from pxr import UsdGeom

    if not usd_path.exists():
        raise FileNotFoundError(f"visual USD does not exist: {usd_path}")
    xform = UsdGeom.Xform.Define(stage, path)
    prim = xform.GetPrim()
    prim.GetReferences().AddReference(str(usd_path.resolve()))
    prim.GetVariantSets().GetVariantSet("Physics").SetVariantSelection(physics_variant)


def _joint_pose_from_scene(scene: dict, pose_name: str) -> dict[str, np.ndarray]:
    if pose_name == "source":
        return {}
    if pose_name == "sim_home":
        return {
            side: np.asarray(scene["sim_home_position_from_openpi_scene"][side], dtype=np.float64)
            for side in ("left", "right")
        }
    if pose_name != "initial_match":
        raise ValueError(f"unsupported USD pose: {pose_name}")

    poses = {}
    for side in ("left", "right"):
        q_backend = np.asarray(scene["factr_initial_match_joint_pos_real"][side], dtype=np.float64).copy()
        q_backend[5] += float(scene["factr_real_to_sim_offsets"][f"{side}_wrist_3"])
        poses[side] = q_backend
    return poses


def _set_revolute_joint_degrees(prim, position_deg: float) -> None:
    from pxr import PhysxSchema, UsdPhysics

    drive_api = UsdPhysics.DriveAPI.Apply(prim, "angular")
    drive_api.CreateTargetPositionAttr().Set(float(position_deg))
    joint_state = PhysxSchema.JointStateAPI.Apply(prim, "angular")
    joint_state.CreatePositionAttr().Set(float(position_deg))
    joint_state.CreateVelocityAttr().Set(0.0)


def _usd_quat_to_rotation(quat) -> np.ndarray:
    imaginary = quat.GetImaginary()
    return _quat_wxyz_rotation((quat.GetReal(), imaginary[0], imaginary[1], imaginary[2]))


def _joint_local_transform(prim, suffix: str) -> np.ndarray:
    from pxr import Gf

    pos_attr = prim.GetAttribute(f"physics:localPos{suffix}")
    rot_attr = prim.GetAttribute(f"physics:localRot{suffix}")
    pos = pos_attr.Get() if pos_attr and pos_attr.HasValue() else Gf.Vec3f(0.0, 0.0, 0.0)
    rot = rot_attr.Get() if rot_attr and rot_attr.HasValue() else Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    return _translation_transform(pos) @ _rotation_transform(_usd_quat_to_rotation(rot))


def _joint_axis_vector(prim) -> np.ndarray:
    axis = prim.GetAttribute("physics:axis").Get()
    if axis == "X":
        return np.asarray([1.0, 0.0, 0.0], dtype=np.float64)
    if axis == "Y":
        return np.asarray([0.0, 1.0, 0.0], dtype=np.float64)
    if axis == "Z":
        return np.asarray([0.0, 0.0, 1.0], dtype=np.float64)
    raise ValueError(f"unsupported revolute axis {axis!r} on {prim.GetPath()}")


def _first_relationship_target(prim, relationship_name: str):
    relationship = prim.GetRelationship(relationship_name)
    targets = relationship.GetTargets() if relationship else []
    return targets[0] if targets else None


def _load_visual_joint_info(usd_path: Path, articulation_path: str) -> dict:
    from pxr import Usd, UsdPhysics

    source_stage = Usd.Stage.Open(str(usd_path.resolve()))
    source_root = source_stage.GetDefaultPrim().GetPath()
    source_stage.GetDefaultPrim().GetVariantSets().GetVariantSet("Physics").SetVariantSelection("physx")
    joint_infos = {}
    for prim in source_stage.Traverse():
        if not prim.IsA(UsdPhysics.RevoluteJoint):
            continue
        body1_path = _first_relationship_target(prim, "physics:body1")
        if body1_path is None:
            continue
        body1_suffix = str(body1_path).removeprefix(str(source_root))
        joint_infos[prim.GetName()] = {
            "axis": _joint_axis_vector(prim),
            "body1_path": f"{articulation_path}{body1_suffix}",
            "joint0": _joint_local_transform(prim, "0"),
            "joint1": _joint_local_transform(prim, "1"),
        }
    return joint_infos


def _apply_revolute_body_xforms(stage, joint_infos_by_name: dict, joint_position_rad_by_name: dict[str, float]) -> int:
    count = 0
    for joint_name, position_rad in joint_position_rad_by_name.items():
        joint_info = joint_infos_by_name.get(joint_name)
        if joint_info is None:
            continue
        body1_prim = stage.GetPrimAtPath(joint_info["body1_path"])
        if not body1_prim.IsValid():
            continue
        rotation = _rotation_transform(_axis_angle_rotation(joint_info["axis"], float(position_rad)))
        joint0 = joint_info["joint0"]
        joint1 = joint_info["joint1"]
        body1_from_body0 = joint0 @ rotation @ np.linalg.inv(joint1)
        _apply_transform(body1_prim, body1_from_body0)
        count += 1
    return count


def _build_articulation_dof_targets(
    stage, articulation_path: str, pose_by_side: dict[str, np.ndarray], simulation_app
) -> list[tuple[object, list[int], list[float]]]:
    from isaacsim.core.experimental.prims import Articulation
    from isaacsim.core.simulation_manager import SimulationManager
    from pxr import UsdPhysics

    if not stage.GetPrimAtPath("/World/PhysicsScene").IsValid():
        UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
    simulation_app.update()
    if SimulationManager.get_physics_sim_view() is None:
        SimulationManager.initialize_physics()

    targets = []
    for side, q in pose_by_side.items():
        root_path = f"{articulation_path}/Geometry/{side}_robot_mount"
        if not stage.GetPrimAtPath(root_path).IsValid():
            continue
        articulation = Articulation(root_path)
        dof_position_by_name = {
            f"{side}_{joint_name}": float(position)
            for joint_name, position in zip(JOINT_NAMES, q)
        }
        for gripper_joint_name, position_deg in ROBOTIQ_OPEN_JOINT_DEGREES.items():
            dof_position_by_name[f"{side}_{gripper_joint_name}"] = float(np.deg2rad(position_deg))

        dof_indices = []
        dof_positions = []
        for index, dof_name in enumerate(articulation.dof_names):
            if dof_name in dof_position_by_name:
                dof_indices.append(index)
                dof_positions.append(dof_position_by_name[dof_name])
        if not dof_indices:
            continue
        targets.append((articulation, dof_indices, dof_positions))
    return targets


def _set_articulation_dof_targets(targets: list[tuple[object, list[int], list[float]]]) -> int:
    total_set = 0
    for articulation, dof_indices, dof_positions in targets:
        articulation.set_dof_positions(dof_positions, dof_indices=dof_indices)
        articulation.set_dof_position_targets(dof_positions, dof_indices=dof_indices)
        total_set += len(dof_indices)
    return total_set


def _apply_articulation_dof_pose(
    stage, articulation_path: str, pose_by_side: dict[str, np.ndarray], simulation_app
) -> list[tuple[object, list[int], list[float]]]:
    targets = _build_articulation_dof_targets(stage, articulation_path, pose_by_side, simulation_app)
    total_set = _set_articulation_dof_targets(targets)
    for _ in range(5):
        simulation_app.update()
        _set_articulation_dof_targets(targets)
    _set_articulation_dof_targets(targets)
    if total_set:
        print(f"applied live articulation pose to {total_set} DOFs")
    return targets


def _set_usd_pose_handles(stage, handles: tuple[list[tuple[object, list[int], list[float]]], tuple[dict, dict] | None]) -> None:
    articulation_targets, joint_xform_targets = handles
    _set_articulation_dof_targets(articulation_targets)
    if joint_xform_targets is not None:
        joint_infos_by_name, joint_position_rad_by_name = joint_xform_targets
        _apply_revolute_body_xforms(stage, joint_infos_by_name, joint_position_rad_by_name)


def _set_gripper_fraction(
    joint_infos_by_name: dict,
    joint_position_rad_by_name: dict[str, float],
    sides: Iterable[str],
    fraction: float,
    close_angle_deg: float,
) -> int:
    fraction = max(0.0, min(1.0, float(fraction)))
    updated = 0
    for side in sides:
        for joint_name, scale in ROBOTIQ_JOINT_CLOSE_SCALE.items():
            full_joint_name = f"{side}_{joint_name}"
            if full_joint_name not in joint_infos_by_name:
                continue
            position_deg = float(close_angle_deg) * float(scale) * fraction
            joint_position_rad_by_name[full_joint_name] = float(np.deg2rad(position_deg))
            updated += 1
    return updated


def _set_usd_gripper_fraction(
    stage,
    handles: tuple[list[tuple[object, list[int], list[float]]], tuple[dict, dict] | None],
    sides: Iterable[str],
    fraction: float,
    close_angle_deg: float,
) -> int:
    _, joint_xform_targets = handles
    if joint_xform_targets is None:
        return 0
    joint_infos_by_name, joint_position_rad_by_name = joint_xform_targets
    updated = _set_gripper_fraction(joint_infos_by_name, joint_position_rad_by_name, sides, fraction, close_angle_deg)
    _apply_revolute_body_xforms(stage, joint_infos_by_name, joint_position_rad_by_name)
    return updated


def _apply_usd_joint_pose(
    stage,
    articulation_path: str,
    visual_usd: Path,
    scene: dict,
    pose_name: str,
    simulation_app,
    gripper_fraction: float = 0.0,
    gripper_close_angle_deg: float = DEFAULT_ROBOTIQ_CLOSE_DEG,
) -> tuple[list[tuple[object, list[int], list[float]]], tuple[dict, dict] | None]:
    pose_by_side = _joint_pose_from_scene(scene, pose_name)
    if not pose_by_side:
        return [], None

    joint_infos_by_name = _load_visual_joint_info(visual_usd, articulation_path)
    expected_names = []
    for side, q in pose_by_side.items():
        for joint_name, position in zip(JOINT_NAMES, q):
            expected_names.append(f"{side}_{joint_name}")

    missing = [name for name in expected_names if name not in joint_infos_by_name]
    if missing:
        available = sorted(name for name in joint_infos_by_name if name.startswith(("left_", "right_")))
        raise RuntimeError(f"visual USD is missing expected arm joints: {missing}; available joints: {available}")

    joint_position_rad_by_name = {}
    for side, q in pose_by_side.items():
        for joint_name, position in zip(JOINT_NAMES, q):
            full_joint_name = f"{side}_{joint_name}"
            joint_position_rad_by_name[full_joint_name] = float(position)

    gripper_joint_count = _set_gripper_fraction(
        joint_infos_by_name,
        joint_position_rad_by_name,
        pose_by_side.keys(),
        gripper_fraction,
        gripper_close_angle_deg,
    )
    transformed_joint_count = _apply_revolute_body_xforms(stage, joint_infos_by_name, joint_position_rad_by_name)

    pose_text = ", ".join(
        f"{side}=[{', '.join(f'{value:.4f}' for value in q)}]" for side, q in pose_by_side.items()
    )
    print(f"applied USD joint pose '{pose_name}': {pose_text}")
    if gripper_joint_count:
        gripper_state = "open" if gripper_fraction <= 0.0 else f"{gripper_fraction:.2f} closed"
        print(f"applied Robotiq {gripper_state} pose to {gripper_joint_count} gripper joints")
    if transformed_joint_count:
        print(f"applied visual body transforms for {transformed_joint_count} revolute joints")
    return [], (joint_infos_by_name, joint_position_rad_by_name)


def _collision_centers(
    config_dir: Path,
    scene: dict,
    side: str,
    pose_name: str = "initial_match",
) -> list[tuple[str, np.ndarray, float]]:
    from isaac6_cumotion_stream_server import CuMotionRmpPolicy, _load_collision_spheres

    offsets = scene["factr_real_to_sim_offsets"]
    wrist_3_offset = float(offsets[f"{side}_wrist_3"])
    policy = CuMotionRmpPolicy(config_dir, wrist_3_offset=wrist_3_offset)
    pose_by_side = _joint_pose_from_scene(scene, pose_name)
    if side not in pose_by_side:
        raise ValueError(f"pose {pose_name!r} does not define side {side!r}")
    q_backend = pose_by_side[side]
    q_col = np.expand_dims(q_backend, 1)
    world_from_base_R, world_from_base_t = _base_transform(scene["bases"][side])

    spheres = []
    for link_name, center_link, radius in _load_collision_spheres(config_dir / "robot.xrdf"):
        pose = policy.kinematics.pose(q_col, link_name)
        center_base = np.asarray(pose.rotation.matrix(), dtype=np.float64) @ center_link
        center_base = center_base + np.asarray(pose.translation, dtype=np.float64)
        center_world = world_from_base_R @ center_base + world_from_base_t
        spheres.append((link_name, center_world, radius))
    return spheres


def _load_urdf_visual_model(urdf_path: Path) -> tuple[dict, dict, dict]:
    root = ET.parse(urdf_path).getroot()
    materials = {}
    for material in root.findall("material"):
        name = material.attrib.get("name")
        color = material.find("color")
        if name and color is not None and "rgba" in color.attrib:
            rgba = [float(part) for part in color.attrib["rgba"].split()]
            materials[name] = tuple(rgba)

    links = {}
    for link in root.findall("link"):
        visuals = []
        for visual in link.findall("visual"):
            origin = visual.find("origin")
            xyz = _parse_float_list(origin.attrib.get("xyz") if origin is not None else None, (0.0, 0.0, 0.0))
            rpy = _parse_float_list(origin.attrib.get("rpy") if origin is not None else None, (0.0, 0.0, 0.0))
            geometry = visual.find("geometry")
            material = visual.find("material")
            material_name = material.attrib.get("name") if material is not None else ""
            if geometry is None:
                continue
            shape = None
            if geometry.find("box") is not None:
                shape = ("box", _parse_float_list(geometry.find("box").attrib["size"], (1.0, 1.0, 1.0)))
            elif geometry.find("cylinder") is not None:
                cylinder = geometry.find("cylinder")
                shape = ("cylinder", (float(cylinder.attrib["radius"]), float(cylinder.attrib["length"])))
            elif geometry.find("sphere") is not None:
                shape = ("sphere", float(geometry.find("sphere").attrib["radius"]))
            if shape is not None:
                visuals.append(
                    {
                        "name": visual.attrib.get("name", f"visual_{len(visuals)}"),
                        "origin": _xyz_rpy_transform(xyz, rpy),
                        "shape": shape,
                        "material": material_name,
                    }
                )
        links[link.attrib["name"]] = visuals

    joints = {}
    children = {}
    for joint in root.findall("joint"):
        name = joint.attrib["name"]
        parent = joint.find("parent").attrib["link"]
        child = joint.find("child").attrib["link"]
        origin = joint.find("origin")
        axis = joint.find("axis")
        xyz = _parse_float_list(origin.attrib.get("xyz") if origin is not None else None, (0.0, 0.0, 0.0))
        rpy = _parse_float_list(origin.attrib.get("rpy") if origin is not None else None, (0.0, 0.0, 0.0))
        joints[name] = {
            "type": joint.attrib.get("type", "fixed"),
            "parent": parent,
            "child": child,
            "origin": _xyz_rpy_transform(xyz, rpy),
            "axis": _parse_float_list(axis.attrib.get("xyz") if axis is not None else None, (0.0, 0.0, 1.0)),
        }
        children.setdefault(parent, []).append(name)
    return materials, links, joints, children


def _link_transforms(children: dict, joints: dict, q_by_joint: dict[str, float], base_transform: np.ndarray) -> dict:
    transforms = {"base": base_transform}
    stack = ["base"]
    while stack:
        parent = stack.pop()
        parent_transform = transforms[parent]
        for joint_name in children.get(parent, []):
            joint = joints[joint_name]
            joint_transform = parent_transform @ joint["origin"]
            if joint["type"] in ("revolute", "continuous"):
                joint_transform = joint_transform @ _rotation_transform(
                    _axis_angle_rotation(joint["axis"], q_by_joint.get(joint_name, 0.0))
                )
            transforms[joint["child"]] = joint_transform
            stack.append(joint["child"])
    return transforms


def _add_urdf_visuals(
    stage,
    side_root: str,
    urdf_path: Path,
    scene: dict,
    side: str,
    pose_name: str = "initial_match",
) -> int:
    from pxr import UsdGeom

    materials, links, joints, children = _load_urdf_visual_model(urdf_path)
    pose_by_side = _joint_pose_from_scene(scene, pose_name)
    if side not in pose_by_side:
        raise ValueError(f"pose {pose_name!r} does not define side {side!r}")
    q_by_joint = dict(zip(JOINT_NAMES, pose_by_side[side]))
    transforms = _link_transforms(children, joints, q_by_joint, _world_from_base_transform(scene["bases"][side]))
    count = 0
    UsdGeom.Xform.Define(stage, side_root)
    for link_name, visuals in links.items():
        link_transform = transforms.get(link_name)
        if link_transform is None:
            continue
        for index, visual in enumerate(visuals):
            rgba = materials.get(visual["material"], (0.6, 0.6, 0.6, 1.0))
            color, opacity = rgba[:3], rgba[3]
            shape_type, shape_value = visual["shape"]
            prim_path = f"{side_root}/urdf_visuals/{link_name}_{index:03d}_{visual['name']}"
            transform = link_transform @ visual["origin"]
            if shape_type == "box":
                _add_box_visual(stage, prim_path, transform, shape_value, color, opacity)
            elif shape_type == "cylinder":
                radius, length = shape_value
                _add_cylinder_visual(stage, prim_path, transform, radius, length, color, opacity)
            elif shape_type == "sphere":
                _add_visual_sphere(stage, prim_path, transform, shape_value, color, opacity)
            count += 1
    return count


def _add_collision_sphere_overlay(
    stage,
    config_dir: Path,
    scene: dict,
    sphere_opacity: float,
    pose_name: str = "initial_match",
) -> int:
    from pxr import UsdGeom

    UsdGeom.Xform.Define(stage, "/World/Materials")
    sphere_materials = {
        side: _make_preview_material(stage, f"/World/Materials/{side}_collision_sphere", color, sphere_opacity)
        for side, color in SIDE_COLORS.items()
    }
    sphere_count = 0
    for side, color in SIDE_COLORS.items():
        _, base_t = _base_transform(scene["bases"][side])
        UsdGeom.Xform.Define(stage, f"/World/{side}")
        _add_frame_marker(stage, f"/World/{side}", base_t, color)
        for index, (link_name, center, radius) in enumerate(_collision_centers(config_dir, scene, side, pose_name)):
            sphere_path = f"/World/{side}/collision_spheres/{link_name}_{index:03d}"
            _add_sphere(
                stage,
                sphere_path,
                center,
                radius,
                color,
                opacity=sphere_opacity,
                material=sphere_materials[side],
            )
            sphere_count += 1
    return sphere_count


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config-dir", type=Path, default=DEFAULT_CONFIG_DIR)
    parser.add_argument("--scene-metadata", type=Path, default=DEFAULT_SCENE_METADATA)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration-s", type=float, default=3600.0)
    parser.add_argument("--output-usd", type=Path)
    parser.add_argument(
        "--visual-mode",
        choices=("arms", "spheres", "both", "usd", "usd_spheres"),
        default="usd_spheres",
    )
    parser.add_argument(
        "--visual-usd",
        type=Path,
        default=REPO_DIR / "generated/isaac_cumotion/usd/maxlab_dual_ur7e_table/maxlab_dual_ur7e_table.usda",
        help="Generated MaxLab visual USD to reference when --visual-mode usd is used.",
    )
    parser.add_argument(
        "--usd-pose",
        choices=("initial_match", "sim_home", "source"),
        default="initial_match",
        help="Joint posture applied to the referenced visual USD and sphere overlay.",
    )
    parser.add_argument(
        "--sphere-pose",
        choices=("initial_match", "sim_home"),
        default=None,
        help="Joint posture used for XRDF collision spheres. Defaults to --usd-pose, or initial_match when --usd-pose source.",
    )
    parser.add_argument(
        "--gripper-fraction",
        type=float,
        default=0.0,
        help="Static Robotiq close fraction for --visual-mode usd. 0.0 is open; 1.0 is closed.",
    )
    parser.add_argument(
        "--gripper-close-angle-deg",
        type=float,
        default=DEFAULT_ROBOTIQ_CLOSE_DEG,
        help="Robotiq driver close angle used by the visual gripper transform.",
    )
    parser.add_argument(
        "--gripper-cycle",
        action="store_true",
        help="Animate the visual Robotiq grippers open/closed in --visual-mode usd.",
    )
    parser.add_argument(
        "--gripper-cycle-period-s",
        type=float,
        default=2.0,
        help="Seconds per open-close-open cycle when --gripper-cycle is set.",
    )
    parser.add_argument("--sphere-opacity", type=float, default=0.35)
    parser.add_argument("--width", type=int, default=1440)
    parser.add_argument("--height", type=int, default=900)
    args = parser.parse_args()

    from isaacsim import SimulationApp

    simulation_app = SimulationApp(
        {
            "headless": bool(args.headless),
            "width": int(args.width),
            "height": int(args.height),
        }
    )

    try:
        import omni.usd
        from pxr import UsdGeom

        stage = omni.usd.get_context().get_stage()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        UsdGeom.Xform.Define(stage, "/World")
        stage.SetDefaultPrim(stage.GetPrimAtPath("/World"))

        scene = _load_yaml(args.scene_metadata.resolve())
        sphere_pose = args.sphere_pose or ("initial_match" if args.usd_pose == "source" else args.usd_pose)
        if args.visual_mode in ("usd", "usd_spheres"):
            visual_root = "/World/maxlab_visual_usd"
            print(f"loading visual USD: {args.visual_usd.resolve()}", flush=True)
            _add_usd_reference(stage, visual_root, args.visual_usd, physics_variant="none")
            print(f"applying USD pose: {args.usd_pose}", flush=True)
            try:
                usd_pose_handles = _apply_usd_joint_pose(
                    stage,
                    visual_root,
                    args.visual_usd,
                    scene,
                    args.usd_pose,
                    simulation_app,
                    gripper_fraction=args.gripper_fraction,
                    gripper_close_angle_deg=args.gripper_close_angle_deg,
                )
            except BaseException as exc:
                print(f"failed to apply USD pose: {type(exc).__name__}: {exc}", flush=True)
                raise
            visual_count = 1
            sphere_count = 0
            if args.visual_mode == "usd_spheres":
                sphere_count = _add_collision_sphere_overlay(
                    stage,
                    args.config_dir.resolve(),
                    scene,
                    args.sphere_opacity,
                    sphere_pose,
                )
            if args.output_usd:
                _set_usd_pose_handles(stage, usd_pose_handles)
                args.output_usd.parent.mkdir(parents=True, exist_ok=True)
                stage.GetRootLayer().Export(str(args.output_usd))
                print(f"exported {args.output_usd}")
            print(f"referenced visual USD: {args.visual_usd.resolve()}")
            if sphere_count:
                print(f"overlaid {sphere_count} XRDF collision spheres")
            if args.gripper_cycle:
                print(
                    "cycling Robotiq grippers: "
                    f"period={args.gripper_cycle_period_s:.2f}s, close_angle={args.gripper_close_angle_deg:.1f}deg"
                )
            start_t = time.monotonic()
            stop_t = time.monotonic() + max(float(args.duration_s), 0.0)
            while simulation_app.is_running() and time.monotonic() < stop_t:
                if args.gripper_cycle:
                    elapsed = time.monotonic() - start_t
                    period = max(float(args.gripper_cycle_period_s), 0.1)
                    fraction = 0.5 - 0.5 * math.cos(2.0 * math.pi * elapsed / period)
                    _set_usd_gripper_fraction(
                        stage,
                        usd_pose_handles,
                        ("left", "right"),
                        fraction,
                        args.gripper_close_angle_deg,
                    )
                else:
                    _set_usd_pose_handles(stage, usd_pose_handles)
                simulation_app.update()
            return

        half_extents = np.asarray(scene["table"]["half_extents"], dtype=np.float64)
        table_center = np.asarray([0.0, 0.0, float(scene["table"]["board_top_z"]) - half_extents[2]], dtype=np.float64)
        _add_cube(stage, "/World/table", table_center, 2.0 * half_extents, (0.45, 0.45, 0.42))

        sphere_count = 0
        visual_count = 0
        for side, color in SIDE_COLORS.items():
            _, base_t = _base_transform(scene["bases"][side])
            UsdGeom.Xform.Define(stage, f"/World/{side}")
            _add_frame_marker(stage, f"/World/{side}", base_t, color)
            if args.visual_mode in ("arms", "both"):
                visual_count += _add_urdf_visuals(
                    stage,
                    f"/World/{side}",
                    args.config_dir.resolve() / "robot.urdf",
                    scene,
                    side,
                    sphere_pose,
                )
            if args.visual_mode == "arms":
                continue
        if args.visual_mode in ("spheres", "both"):
            sphere_count = _add_collision_sphere_overlay(
                stage,
                args.config_dir.resolve(),
                scene,
                args.sphere_opacity,
                sphere_pose,
            )

        if args.output_usd:
            args.output_usd.parent.mkdir(parents=True, exist_ok=True)
            stage.GetRootLayer().Export(str(args.output_usd))
            print(f"exported {args.output_usd}")

        print(f"visualized {visual_count} URDF visual primitives and {sphere_count} XRDF collision spheres")
        stop_t = time.monotonic() + max(float(args.duration_s), 0.0)
        while simulation_app.is_running() and time.monotonic() < stop_t:
            simulation_app.update()
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
