#!/usr/bin/env python3
"""Create/open a primitive dual-UR7e Isaac scene from MaxLab metadata."""

from __future__ import annotations

import argparse
import math
from pathlib import Path


REPO_DIR = Path("/home/srianumakonda/FACTR_Teleop")
DEFAULT_ROBOT_USD = REPO_DIR / "generated/isaac_rmpflow/maxlab_ur7e_right_primitive.usd"
DEFAULT_SCENE_USD = REPO_DIR / "generated/isaac_rmpflow/maxlab_dual_ur7e_primitive_scene.usd"
DEFAULT_DESCRIPTOR = REPO_DIR / "configs/isaac_rmpflow/maxlab_ur7e_right/rmpflow/maxlab_ur7e_right_robot_description.yaml"

LEFT_INITIAL_REAL = [1.5700, -1.5700, 1.5700, -1.5700, -1.5700, -1.5700]
RIGHT_INITIAL_REAL = [-1.5700, -1.5700, -1.5700, -1.5700, 1.5700, 0.0000]
LEFT_WRIST_3_OFFSET = math.pi / 2.0
RIGHT_WRIST_3_OFFSET = math.pi

TABLE_BOXES = [
    ("table_top", (0.0, 0.0, 0.74), (0.8625, 0.57, 0.02)),
    ("plate_left", (-0.7425, -0.005, 0.763), (0.09, 0.09, 0.003)),
    ("plate_right", (0.7425, -0.005, 0.763), (0.09, 0.09, 0.003)),
    ("board_mid", (0.0, 0.0, 0.7675), (0.6525, 0.57, 0.0075)),
    ("board_left_front", (-0.7575, -0.3325, 0.7675), (0.105, 0.2375, 0.0075)),
    ("board_left_back", (-0.7575, 0.3275, 0.7675), (0.105, 0.2425, 0.0075)),
    ("board_right_front", (0.7575, -0.3325, 0.7675), (0.105, 0.2375, 0.0075)),
    ("board_right_back", (0.7575, 0.3275, 0.7675), (0.105, 0.2425, 0.0075)),
]


def _cube(stage, path: str, translate: tuple[float, float, float], scale: tuple[float, float, float]) -> None:
    from pxr import Gf, UsdGeom

    prim = UsdGeom.Cube.Define(stage, path)
    prim.AddTranslateOp().Set(Gf.Vec3d(*translate))
    prim.AddScaleOp().Set(Gf.Vec3d(*scale))


def _with_wrist_offset(q_real: list[float], wrist_3_offset: float) -> list[float]:
    q_sim = list(q_real)
    q_sim[5] += wrist_3_offset
    return q_sim


def _reference_robot(
    stage,
    path: str,
    robot_usd: Path,
    translate: list[float],
    quat_wxyz: list[float],
    q_real: list[float],
    q_sim: list[float],
) -> None:
    from pxr import Gf, Sdf, UsdGeom

    xform = UsdGeom.Xform.Define(stage, path)
    xform.GetPrim().GetReferences().AddReference(str(robot_usd), "/maxlab_ur7e")
    ops = {op.GetOpName(): op for op in xform.GetOrderedXformOps()}
    translate_op = ops.get("xformOp:translate") or xform.AddTranslateOp()
    orient_op = ops.get("xformOp:orient") or xform.AddOrientOp()
    translate_op.Set(Gf.Vec3d(*translate))
    orient_op.Set(Gf.Quatd(quat_wxyz[0], Gf.Vec3d(quat_wxyz[1], quat_wxyz[2], quat_wxyz[3])))
    prim = xform.GetPrim()
    prim.CreateAttribute("factr:initial_q_real", Sdf.ValueTypeNames.DoubleArray).Set(q_real)
    prim.CreateAttribute("factr:initial_q_sim", Sdf.ValueTypeNames.DoubleArray).Set(q_sim)


def _load_collision_spheres(descriptor_path: Path) -> list[tuple[str, list[float], float]]:
    import yaml

    with descriptor_path.open("r", encoding="utf-8") as stream:
        descriptor = yaml.safe_load(stream)

    spheres: list[tuple[str, list[float], float]] = []
    for group in descriptor.get("collision_spheres", []):
        for link_name, link_spheres in group.items():
            for sphere in link_spheres:
                spheres.append((link_name, sphere["center"], float(sphere["radius"])))
    return spheres


def _add_collision_sphere_overlays(stage, robot_path: str, descriptor_path: Path) -> int:
    from pxr import Gf, UsdGeom

    count = 0
    for link_name, center, radius in _load_collision_spheres(descriptor_path):
        sphere_path = f"{robot_path}/{link_name}/lula_collision_spheres/sphere_{count:03d}"
        sphere = UsdGeom.Sphere.Define(stage, sphere_path)
        sphere.CreateRadiusAttr(radius)
        sphere.AddTranslateOp().Set(Gf.Vec3d(*center))
        sphere.CreateDisplayColorAttr([Gf.Vec3f(0.95, 0.68, 0.1)])
        sphere.CreateDisplayOpacityAttr([0.45])
        count += 1
    return count


def create_scene(robot_usd: Path, scene_usd: Path, descriptor_path: Path) -> int:
    from pxr import Gf, Usd, UsdGeom, UsdLux

    if not robot_usd.exists():
        raise FileNotFoundError(f"Robot USD missing; run convert_maxlab_urdf_to_usd.sh first: {robot_usd}")
    if not descriptor_path.exists():
        raise FileNotFoundError(f"RMPFlow descriptor missing: {descriptor_path}")

    scene_usd.parent.mkdir(parents=True, exist_ok=True)
    if scene_usd.exists():
        scene_usd.unlink()
    stage = Usd.Stage.CreateNew(str(scene_usd))
    if stage is None:
        raise RuntimeError(f"Could not create scene USD: {scene_usd}")
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    light = UsdLux.DistantLight.Define(stage, "/World/key_light")
    light.CreateIntensityAttr(600.0)
    light.AddRotateXYZOp().Set(Gf.Vec3f(-45.0, 0.0, 30.0))

    for name, translate, scale in TABLE_BOXES:
        _cube(stage, f"/World/{name}", translate, scale)
    for x in (-0.8125, 0.8125):
        for y in (-0.52, 0.52):
            _cube(stage, f"/World/table_leg_{'p' if x > 0 else 'n'}x_{'p' if y > 0 else 'n'}y", (x, y, 0.36), (0.02, 0.02, 0.36))

    left_initial_sim = _with_wrist_offset(LEFT_INITIAL_REAL, LEFT_WRIST_3_OFFSET)
    right_initial_sim = _with_wrist_offset(RIGHT_INITIAL_REAL, RIGHT_WRIST_3_OFFSET)
    _reference_robot(
        stage,
        "/World/left_ur7e",
        robot_usd,
        [-0.7425, -0.005, 0.766],
        [0.70710678, 0.0, 0.0, 0.70710678],
        LEFT_INITIAL_REAL,
        left_initial_sim,
    )
    _reference_robot(
        stage,
        "/World/right_ur7e",
        robot_usd,
        [0.7425, -0.005, 0.766],
        [0.70710678, 0.0, 0.0, -0.70710678],
        RIGHT_INITIAL_REAL,
        right_initial_sim,
    )
    sphere_count = _add_collision_sphere_overlays(stage, "/World/left_ur7e", descriptor_path)
    sphere_count += _add_collision_sphere_overlays(stage, "/World/right_ur7e", descriptor_path)
    stage.GetRootLayer().Save()
    return sphere_count


def main() -> None:
    from isaaclab.app import AppLauncher

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-usd", type=Path, default=DEFAULT_ROBOT_USD)
    parser.add_argument("--scene-usd", type=Path, default=DEFAULT_SCENE_USD)
    parser.add_argument("--descriptor", type=Path, default=DEFAULT_DESCRIPTOR)
    AppLauncher.add_app_launcher_args(parser)
    args_cli = parser.parse_args()

    app_launcher = AppLauncher(args_cli)
    simulation_app = app_launcher.app

    import carb
    import omni.kit.app
    from pxr import Usd

    robot_usd = args_cli.robot_usd.resolve()
    scene_usd = args_cli.scene_usd.resolve()
    descriptor_path = args_cli.descriptor.resolve()
    try:
        sphere_count = create_scene(robot_usd, scene_usd, descriptor_path)
        stage = Usd.Stage.Open(str(scene_usd))
        prim_count = len(list(stage.Traverse())) if stage is not None else 0
        print(f"[INFO] Wrote primitive dual-UR7e scene: {scene_usd}", flush=True)
        print(f"[INFO] Scene prim_count: {prim_count}", flush=True)
        print(f"[INFO] Lula collision sphere overlays: {sphere_count}", flush=True)
        print("[INFO] Left base:  pos=[-0.7425, -0.005, 0.766], yaw=+90deg", flush=True)
        print("[INFO] Right base: pos=[ 0.7425, -0.005, 0.766], yaw=-90deg", flush=True)
        print(f"[INFO] Left FACTR initial q real={LEFT_INITIAL_REAL} sim={_with_wrist_offset(LEFT_INITIAL_REAL, LEFT_WRIST_3_OFFSET)}", flush=True)
        print(f"[INFO] Right FACTR initial q real={RIGHT_INITIAL_REAL} sim={_with_wrist_offset(RIGHT_INITIAL_REAL, RIGHT_WRIST_3_OFFSET)}", flush=True)

        app = omni.kit.app.get_app_interface()
        settings = carb.settings.get_settings()
        if not bool(settings.get("/app/window/enabled")) and not bool(settings.get("/app/livestream/enabled")):
            for _ in range(10):
                app.update()
            return
        while app.is_running():
            app.update()
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
