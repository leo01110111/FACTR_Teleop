#!/usr/bin/env python3
"""Generate a self-contained primitive URDF from the MaxLab UR7e MJCF.

The MaxLab source keeps the legacy `ur5e.xml` path, but project comments say
this retuned model should be treated as the UR7e arm. This exporter preserves
the MJCF kinematic body tree and primitive collision geoms, and intentionally
does not reference external mesh packages.
"""

from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path


DEFAULT_INPUT = Path("/home/srianumakonda/maxlab/sim/universal_robots_ur5e/ur5e.xml")
DEFAULT_OUTPUT = Path("configs/isaac_rmpflow/maxlab_ur7e_right/maxlab_ur7e_right.urdf")
ARM_JOINTS = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
}
ROBOTIQ_BOXES = [
    # name, parent, xyz, rpy, size xyz, mass, material
    ("robotiq_base", "tool0", [0.0, 0.0, 0.035], [0.0, 0.0, 0.0], [0.080, 0.070, 0.070], "0.78", "robotiq_black"),
    ("robotiq_right_finger", "robotiq_base", [0.0, 0.046, 0.078], [0.0, 0.0, 0.0], [0.022, 0.014, 0.080], "0.04", "robotiq_gray"),
    ("robotiq_left_finger", "robotiq_base", [0.0, -0.046, 0.078], [0.0, 0.0, 0.0], [0.022, 0.014, 0.080], "0.04", "robotiq_gray"),
    ("robotiq_right_pad", "robotiq_right_finger", [0.0, -0.006, 0.052], [0.0, 0.0, 0.0], [0.022, 0.008, 0.056], "0.004", "robotiq_pad"),
    ("robotiq_left_pad", "robotiq_left_finger", [0.0, 0.006, 0.052], [0.0, 0.0, 0.0], [0.022, 0.008, 0.056], "0.004", "robotiq_pad"),
]
PINCH_CENTER_FROM_TOOL0 = [0.0, 0.0, 0.165]


def _numbers(value: str | None, default: list[float]) -> list[float]:
    if value is None:
        return default
    return [float(part) for part in value.split()]


def _fmt(values: list[float]) -> str:
    return " ".join(f"{value:.9g}" for value in values)


def _quat_to_rpy(quat: list[float]) -> list[float]:
    w, x, y, z = quat
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm == 0.0:
        return [0.0, 0.0, 0.0]
    w, x, y, z = (w / norm, x / norm, y / norm, z / norm)

    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return [roll, pitch, yaw]


def _origin(parent: ET.Element, source: ET.Element) -> ET.Element:
    origin = ET.SubElement(parent, "origin")
    origin.set("xyz", _fmt(_numbers(source.attrib.get("pos"), [0.0, 0.0, 0.0])))
    origin.set("rpy", _fmt(_quat_to_rpy(_numbers(source.attrib.get("quat"), [1.0, 0.0, 0.0, 0.0]))))
    return origin


def _add_inertial(link: ET.Element, body: ET.Element) -> None:
    inertial_src = body.find("inertial")
    inertial = ET.SubElement(link, "inertial")
    if inertial_src is None:
        ET.SubElement(inertial, "mass", {"value": "0.1"})
        ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        ET.SubElement(inertial, "inertia", {"ixx": "0.001", "ixy": "0", "ixz": "0", "iyy": "0.001", "iyz": "0", "izz": "0.001"})
        return

    ET.SubElement(inertial, "mass", {"value": inertial_src.attrib.get("mass", "0.1")})
    origin = ET.SubElement(inertial, "origin")
    origin.set("xyz", _fmt(_numbers(inertial_src.attrib.get("pos"), [0.0, 0.0, 0.0])))
    origin.set("rpy", _fmt(_quat_to_rpy(_numbers(inertial_src.attrib.get("quat"), [1.0, 0.0, 0.0, 0.0]))))
    diag = _numbers(inertial_src.attrib.get("diaginertia"), [0.001, 0.001, 0.001])
    ET.SubElement(
        inertial,
        "inertia",
        {"ixx": str(diag[0]), "ixy": "0", "ixz": "0", "iyy": str(diag[1]), "iyz": "0", "izz": str(diag[2])},
    )


def _geom_type(geom: ET.Element) -> str | None:
    if "mesh" in geom.attrib or geom.attrib.get("class") == "visual":
        return None
    if "type" in geom.attrib:
        return geom.attrib["type"]
    if geom.attrib.get("class") == "collision":
        return "capsule"
    if geom.attrib.get("class") == "eef_collision":
        return "cylinder"
    return None


def _add_geometry(parent: ET.Element, geom: ET.Element, *, material: bool) -> bool:
    geom_type = _geom_type(geom)
    if geom_type is None:
        return False
    geometry = ET.SubElement(parent, "geometry")
    size = _numbers(geom.attrib.get("size"), [0.04])
    if geom_type == "box":
        ET.SubElement(geometry, "box", {"size": _fmt([2.0 * value for value in size[:3]])})
    elif geom_type in {"capsule", "cylinder"}:
        radius = size[0]
        length = 2.0 * size[1] if len(size) > 1 else 0.04
        ET.SubElement(geometry, "cylinder", {"radius": str(radius), "length": str(length)})
    elif geom_type == "sphere":
        ET.SubElement(geometry, "sphere", {"radius": str(size[0])})
    elif geom_type == "plane":
        return False
    else:
        return False
    if material:
        ET.SubElement(parent, "material", {"name": "collision_blue"})
    return True


def _add_geom_pair(link: ET.Element, geom: ET.Element, index: int) -> None:
    for tag, material in (("visual", True), ("collision", False)):
        element = ET.SubElement(link, tag, {"name": geom.attrib.get("name", f"geom_{index}")})
        _origin(element, geom)
        if not _add_geometry(element, geom, material=material):
            link.remove(element)


def _add_link(robot: ET.Element, body: ET.Element) -> None:
    link = ET.SubElement(robot, "link", {"name": body.attrib["name"]})
    _add_inertial(link, body)
    added_geom = False
    for index, geom in enumerate(body.findall("geom")):
        before = len(link)
        _add_geom_pair(link, geom, index)
        added_geom = added_geom or len(link) > before
    if body.attrib["name"] == "base" and not added_geom:
        visual = ET.SubElement(link, "visual", {"name": "base_primitive"})
        ET.SubElement(visual, "origin", {"xyz": "0 0 0.06", "rpy": "0 0 0"})
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(geometry, "cylinder", {"radius": "0.08", "length": "0.12"})
        ET.SubElement(visual, "material", {"name": "base_gray"})
        collision = ET.SubElement(link, "collision", {"name": "base_primitive"})
        ET.SubElement(collision, "origin", {"xyz": "0 0 0.06", "rpy": "0 0 0"})
        geometry = ET.SubElement(collision, "geometry")
        ET.SubElement(geometry, "cylinder", {"radius": "0.08", "length": "0.12"})


def _box_link(robot: ET.Element, name: str, size: list[float], mass: str, material: str) -> None:
    link = ET.SubElement(robot, "link", {"name": name})
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass", {"value": mass})
    ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    sx, sy, sz = size
    m = float(mass)
    ET.SubElement(
        inertial,
        "inertia",
        {
            "ixx": str(m * (sy * sy + sz * sz) / 12.0),
            "ixy": "0",
            "ixz": "0",
            "iyy": str(m * (sx * sx + sz * sz) / 12.0),
            "iyz": "0",
            "izz": str(m * (sx * sx + sy * sy) / 12.0),
        },
    )
    for tag, include_material in (("visual", True), ("collision", False)):
        element = ET.SubElement(link, tag, {"name": name})
        ET.SubElement(element, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        geometry = ET.SubElement(element, "geometry")
        ET.SubElement(geometry, "box", {"size": _fmt(size)})
        if include_material:
            ET.SubElement(element, "material", {"name": material})


def _fixed_joint(robot: ET.Element, name: str, parent: str, child: str, xyz: list[float], rpy: list[float]) -> None:
    joint = ET.SubElement(robot, "joint", {"name": name, "type": "fixed"})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    ET.SubElement(joint, "origin", {"xyz": _fmt(xyz), "rpy": _fmt(rpy)})


def _add_robotiq_primitive(robot: ET.Element) -> None:
    """Add a fixed primitive Robotiq 2F-85 envelope from the MaxLab gripper layout.

    The source MaxLab gripper is `gripper/robotiq-2f85.xml`. This primitive
    version keeps the base, finger, and pad extents for collision/visualization
    without importing STL meshes or movable mimic joints into the 6-DOF arm
    RMPFlow cspace.
    """
    for name, parent, xyz, rpy, size, mass, material in ROBOTIQ_BOXES:
        _box_link(robot, name, size, mass, material)
        _fixed_joint(robot, f"{parent}_to_{name}", parent, name, xyz, rpy)


def _add_pinch_center_frame(robot: ET.Element) -> None:
    """Add a visual-only frame at the primitive Robotiq pad midpoint.

    This frame is for validating the tool/TCP geometry in Isaac before choosing
    whether RMPFlow should target the gripper pinch point instead of tool0.
    """
    link = ET.SubElement(robot, "link", {"name": "pinch_center"})
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass", {"value": "0.001"})
    ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(
        inertial,
        "inertia",
        {"ixx": "0.000001", "ixy": "0", "ixz": "0", "iyy": "0.000001", "iyz": "0", "izz": "0.000001"},
    )
    visual = ET.SubElement(link, "visual", {"name": "pinch_center_marker"})
    ET.SubElement(visual, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "sphere", {"radius": "0.008"})
    ET.SubElement(visual, "material", {"name": "pinch_green"})
    _fixed_joint(robot, "tool0_to_pinch_center", "tool0", "pinch_center", PINCH_CENTER_FROM_TOOL0, [0.0, 0.0, 0.0])


def _joint_limits(name: str, mjcf_class: str | None) -> dict[str, str]:
    lower, upper = (-6.28319, 6.28319)
    if name == "elbow_joint" or mjcf_class == "size3_limited":
        lower, upper = (-3.1415, 3.1415)
    effort = "28.0" if mjcf_class == "size1" else "150.0"
    return {"lower": str(lower), "upper": str(upper), "effort": effort, "velocity": "3.141592653589793"}


def _add_joint(robot: ET.Element, parent_name: str, body: ET.Element) -> None:
    joint_src = body.find("joint")
    if joint_src is None:
        joint = ET.SubElement(robot, "joint", {"name": f"{parent_name}_to_{body.attrib['name']}", "type": "fixed"})
    else:
        joint_name = joint_src.attrib["name"]
        joint_type = "revolute" if joint_name in ARM_JOINTS else "fixed"
        joint = ET.SubElement(robot, "joint", {"name": joint_name, "type": joint_type})
    ET.SubElement(joint, "parent", {"link": parent_name})
    ET.SubElement(joint, "child", {"link": body.attrib["name"]})
    _origin(joint, body)
    if joint_src is not None and joint.attrib["type"] == "revolute":
        axis = _numbers(joint_src.attrib.get("axis"), [0.0, 1.0, 0.0])
        ET.SubElement(joint, "axis", {"xyz": _fmt(axis)})
        ET.SubElement(joint, "limit", _joint_limits(joint_src.attrib["name"], joint_src.attrib.get("class")))


def _walk(robot: ET.Element, body: ET.Element, parent_name: str | None = None) -> None:
    _add_link(robot, body)
    if parent_name is not None:
        _add_joint(robot, parent_name, body)
    for child in body.findall("body"):
        _walk(robot, child, body.attrib["name"])


def export_urdf(input_path: Path, output_path: Path) -> None:
    source = ET.parse(input_path).getroot()
    base = source.find("./worldbody/body[@name='base']")
    if base is None:
        raise RuntimeError(f"Could not find base body in {input_path}")

    robot = ET.Element("robot", {"name": "maxlab_ur7e"})
    ET.SubElement(robot, "material", {"name": "collision_blue"}).append(ET.Element("color", {"rgba": "0.2 0.42 0.78 0.55"}))
    ET.SubElement(robot, "material", {"name": "base_gray"}).append(ET.Element("color", {"rgba": "0.55 0.56 0.58 1"}))
    ET.SubElement(robot, "material", {"name": "robotiq_black"}).append(ET.Element("color", {"rgba": "0.05 0.05 0.05 1"}))
    ET.SubElement(robot, "material", {"name": "robotiq_gray"}).append(ET.Element("color", {"rgba": "0.46 0.46 0.46 1"}))
    ET.SubElement(robot, "material", {"name": "robotiq_pad"}).append(ET.Element("color", {"rgba": "0.18 0.18 0.18 1"}))
    ET.SubElement(robot, "material", {"name": "pinch_green"}).append(ET.Element("color", {"rgba": "0.1 0.8 0.25 1"}))
    _walk(robot, base)

    tool = ET.SubElement(robot, "link", {"name": "tool0"})
    ET.SubElement(tool, "inertial").extend(
        [
            ET.Element("mass", {"value": "0.01"}),
            ET.Element("origin", {"xyz": "0 0 0", "rpy": "0 0 0"}),
            ET.Element("inertia", {"ixx": "0.00001", "ixy": "0", "ixz": "0", "iyy": "0.00001", "iyz": "0", "izz": "0.00001"}),
        ]
    )
    tool_visual = ET.SubElement(tool, "visual", {"name": "tool0_marker"})
    ET.SubElement(tool_visual, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    tool_geometry = ET.SubElement(tool_visual, "geometry")
    ET.SubElement(tool_geometry, "sphere", {"radius": "0.01"})
    ET.SubElement(tool_visual, "material", {"name": "collision_blue"})
    tool_collision = ET.SubElement(tool, "collision", {"name": "tool0_marker"})
    ET.SubElement(tool_collision, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    tool_geometry = ET.SubElement(tool_collision, "geometry")
    ET.SubElement(tool_geometry, "sphere", {"radius": "0.01"})
    tool_joint = ET.SubElement(robot, "joint", {"name": "wrist_3_to_tool0", "type": "fixed"})
    ET.SubElement(tool_joint, "parent", {"link": "wrist_3_link"})
    ET.SubElement(tool_joint, "child", {"link": "tool0"})
    ET.SubElement(tool_joint, "origin", {"xyz": "0 0.1 0", "rpy": "3.141592654 0 -1.570796327"})
    _add_pinch_center_frame(robot)
    _add_robotiq_primitive(robot)

    tree = ET.ElementTree(robot)
    if hasattr(ET, "indent"):
        ET.indent(tree, space="  ")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    export_urdf(args.input, args.output)
    print(args.output.resolve())


if __name__ == "__main__":
    main()
