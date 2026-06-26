#!/usr/bin/env python3
"""Export the MaxLab dual-UR7e table scene as a standalone MJCF file.

The source scene is procedural in `/home/srianumakonda/maxlab/sim/build_urtable.py`.
This materializes it, removes the task block by default, and copies referenced
UR/Robotiq meshes beside the generated XML so Isaac's MJCF importer can resolve
them deterministically.
"""

from __future__ import annotations

import argparse
import os
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


MAXLAB_SIM = Path("/home/srianumakonda/maxlab/sim")
UR_ASSET_DIR = MAXLAB_SIM / "universal_robots_ur5e" / "assets"
GRIPPER_ASSET_DIR = MAXLAB_SIM / "gripper" / "assets"
DEFAULT_OUTPUT = Path("generated/isaac_cumotion/maxlab_dual_ur7e_table.xml")


def _indent(tree: ET.ElementTree) -> None:
    if hasattr(ET, "indent"):
        ET.indent(tree, space="  ")


def _remove_body_by_name(root: ET.Element, name: str) -> bool:
    for parent in root.iter():
        for child in list(parent):
            if child.tag == "body" and child.attrib.get("name") == name:
                parent.remove(child)
                return True
    return False


def _rewrite_mesh_paths(root: ET.Element, xml_dir: Path) -> None:
    mesh_root = xml_dir / "meshes"
    for mesh in root.iter("mesh"):
        file_name = mesh.attrib.get("file")
        if not file_name:
            continue
        path = Path(file_name)
        if path.suffix == ".obj":
            src = UR_ASSET_DIR / path.name
            dst = mesh_root / "ur7e" / path.name
        elif path.suffix == ".stl":
            src = GRIPPER_ASSET_DIR / path.name
            dst = mesh_root / "robotiq_2f85" / path.name
        else:
            continue
        if not src.exists():
            raise FileNotFoundError(f"could not resolve mesh {file_name!r} from {src}")
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        mesh.set("file", os.path.relpath(dst, xml_dir))


def _remove_mesh_geoms(root: ET.Element) -> None:
    for parent in root.iter():
        for child in list(parent):
            if child.tag == "geom" and ("mesh" in child.attrib or child.attrib.get("type") == "mesh"):
                parent.remove(child)


def _remove_mesh_assets(root: ET.Element) -> None:
    asset = root.find("asset")
    if asset is None:
        return
    for child in list(asset):
        if child.tag == "mesh":
            asset.remove(child)


def _inline_geom_defaults(root: ET.Element) -> None:
    defaults: dict[str, dict[str, str]] = {}
    for default in root.iter("default"):
        class_name = default.attrib.get("class")
        geom = default.find("geom")
        if class_name and geom is not None:
            defaults[class_name] = dict(geom.attrib)

    for geom in root.iter("geom"):
        class_name = geom.attrib.get("class")
        if not class_name or class_name not in defaults:
            continue
        for key, value in defaults[class_name].items():
            geom.attrib.setdefault(key, value)


def _prune_empty_leaf_bodies(root: ET.Element) -> None:
    keep_tags = {"geom", "joint", "site", "camera", "light", "inertial"}
    changed = True
    while changed:
        changed = False
        for parent in root.iter():
            for child in list(parent):
                if child.tag != "body":
                    continue
                if any(grandchild.tag == "body" for grandchild in child):
                    continue
                if any(grandchild.tag in keep_tags for grandchild in child):
                    continue
                parent.remove(child)
                changed = True


def _add_placeholder_geoms_to_empty_bodies(root: ET.Element) -> None:
    for body in root.iter("body"):
        if body.find("geom") is not None or body.find("inertial") is not None:
            continue
        body_name = body.attrib.get("name", "body")
        ET.SubElement(
            body,
            "geom",
            {
                "name": f"{body_name}_placeholder",
                "type": "sphere",
                "size": "0.001",
                "rgba": "0 0 0 0",
                "contype": "0",
                "conaffinity": "0",
                "mass": "0.000001",
            },
        )


def _remove_meshes(root: ET.Element) -> None:
    _remove_mesh_geoms(root)
    _remove_mesh_assets(root)
    _inline_geom_defaults(root)
    _prune_empty_leaf_bodies(root)
    _add_placeholder_geoms_to_empty_bodies(root)


def export(output: Path, keep_block: bool, keep_cameras: bool, collision_only: bool) -> Path:
    sys.path.insert(0, str(MAXLAB_SIM))
    cwd = Path.cwd()
    os.chdir(MAXLAB_SIM)
    try:
        import build_urtable

        spec = build_urtable.build_spec()
        spec.compile()
        xml_text = spec.to_xml()
    finally:
        os.chdir(cwd)

    root = ET.fromstring(xml_text)
    if not keep_block:
        _remove_body_by_name(root, "block")
        keyframe = root.find("keyframe")
        if keyframe is not None:
            root.remove(keyframe)
    if not keep_cameras:
        _remove_body_by_name(root, "top1_body")
        _remove_body_by_name(root, "top2_body")
    if collision_only:
        _remove_meshes(root)
    else:
        _rewrite_mesh_paths(root, output.parent)

    tree = ET.ElementTree(root)
    _indent(tree)
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return output.resolve()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--keep-block", action="store_true")
    parser.add_argument("--keep-cameras", action="store_true")
    parser.add_argument(
        "--collision-only",
        action="store_true",
        help="Remove visual mesh geoms/assets and keep the primitive collision model.",
    )
    args = parser.parse_args()
    print(export(args.output, args.keep_block, args.keep_cameras, args.collision_only))


if __name__ == "__main__":
    main()
