#!/usr/bin/env python3
"""Visualize the MaxLab UR7e cuMotion scene and XRDF collision spheres."""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path
from typing import Iterable

import numpy as np
import yaml


REPO_DIR = Path("/home/srianumakonda/FACTR_Teleop")
DEFAULT_CONFIG_DIR = REPO_DIR / "configs/isaac_cumotion/maxlab_ur7e_right"
DEFAULT_SCENE_METADATA = REPO_DIR / "configs/isaac_cumotion/maxlab_ur7e_scene.yaml"


def _load_yaml(path: Path) -> dict:
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"expected YAML mapping in {path}")
    return data


def _yaw_rotation(yaw: float) -> np.ndarray:
    c, s = math.cos(yaw), math.sin(yaw)
    return np.asarray([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


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


def _set_display_color(prim, color):
    from pxr import UsdGeom

    UsdGeom.Gprim(prim).CreateDisplayColorAttr([tuple(float(v) for v in color)])


def _add_sphere(stage, path: str, center, radius: float, color) -> None:
    from pxr import UsdGeom

    sphere = UsdGeom.Sphere.Define(stage, path)
    sphere.CreateRadiusAttr(float(radius))
    sphere.AddTranslateOp().Set(tuple(float(v) for v in center))
    _set_display_color(sphere.GetPrim(), color)


def _add_cube(stage, path: str, center, scale, color) -> None:
    from pxr import UsdGeom

    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    cube.AddTranslateOp().Set(tuple(float(v) for v in center))
    cube.AddScaleOp().Set(tuple(float(v) for v in scale))
    _set_display_color(cube.GetPrim(), color)


def _add_frame_marker(stage, path: str, center, color) -> None:
    _add_sphere(stage, f"{path}/base", center, 0.045, color)


def _collision_centers(config_dir: Path, scene: dict, side: str) -> list[tuple[str, np.ndarray, float]]:
    from isaac6_cumotion_stream_server import CuMotionRmpPolicy, _load_collision_spheres

    offsets = scene["factr_real_to_sim_offsets"]
    wrist_3_offset = float(offsets[f"{side}_wrist_3"])
    policy = CuMotionRmpPolicy(config_dir, wrist_3_offset=wrist_3_offset)
    q_real = np.asarray(scene["factr_initial_match_joint_pos_real"][side], dtype=np.float64)
    q_backend = q_real.copy()
    q_backend[5] += wrist_3_offset
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


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config-dir", type=Path, default=DEFAULT_CONFIG_DIR)
    parser.add_argument("--scene-metadata", type=Path, default=DEFAULT_SCENE_METADATA)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration-s", type=float, default=3600.0)
    parser.add_argument("--output-usd", type=Path)
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
        half_extents = np.asarray(scene["table"]["half_extents"], dtype=np.float64)
        table_center = np.asarray([0.0, 0.0, float(scene["table"]["board_top_z"]) - half_extents[2]], dtype=np.float64)
        _add_cube(stage, "/World/table", table_center, 2.0 * half_extents, (0.45, 0.45, 0.42))

        side_colors = {
            "left": (0.1, 0.35, 1.0),
            "right": (1.0, 0.22, 0.18),
        }
        sphere_count = 0
        for side, color in side_colors.items():
            _, base_t = _base_transform(scene["bases"][side])
            UsdGeom.Xform.Define(stage, f"/World/{side}")
            _add_frame_marker(stage, f"/World/{side}", base_t, color)
            for index, (link_name, center, radius) in enumerate(
                _collision_centers(args.config_dir.resolve(), scene, side)
            ):
                sphere_path = f"/World/{side}/collision_spheres/{link_name}_{index:03d}"
                _add_sphere(stage, sphere_path, center, radius, color)
                sphere_count += 1

        if args.output_usd:
            args.output_usd.parent.mkdir(parents=True, exist_ok=True)
            stage.GetRootLayer().Export(str(args.output_usd))
            print(f"exported {args.output_usd}")

        print(f"visualized {sphere_count} XRDF collision spheres")
        stop_t = time.monotonic() + max(float(args.duration_s), 0.0)
        while simulation_app.is_running() and time.monotonic() < stop_t:
            simulation_app.update()
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
