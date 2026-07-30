#!/usr/bin/env python3
"""Generate the formation-experiment Gazebo assets from checked-in manifests."""

from __future__ import annotations

import argparse
import difflib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


SCENES = (
    "uav6_knot_obstacles",
    "ugv4_figure_eight_obstacles",
    "ugv4_figure_eight_scout_obstacles",
)
MATERIAL = "0.5 0.5 0.5 1"


def indent_xml(element: ET.Element, level: int = 0) -> None:
    """Backport ElementTree.indent for the ROS Noetic Python 3.8 baseline."""
    indentation = "\n" + level * "  "
    child_indentation = "\n" + (level + 1) * "  "
    if len(element):
        if not element.text or not element.text.strip():
            element.text = child_indentation
        for child in element:
            indent_xml(child, level + 1)
            if not child.tail or not child.tail.strip():
                child.tail = child_indentation
        element[-1].tail = indentation
    elif level and (not element.tail or not element.tail.strip()):
        element.tail = indentation


def serialize_xml(element: ET.Element) -> bytes:
    return b'<?xml version="1.0"?>\n' + ET.tostring(element, encoding="utf-8") + b"\n"


def number(value: float) -> str:
    # Python's shortest-round-trip representation keeps source decimals
    # readable without losing any binary64 information.
    return repr(float(value))


def vector(values: list[float]) -> str:
    return " ".join(number(value) for value in values)


def quaternion_to_rpy(quaternion: list[float]) -> tuple[float, float, float]:
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 0.0:
        raise ValueError("zero-length quaternion")
    x, y, z, w = (component / norm for component in (x, y, z, w))
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def add_geometry(parent: ET.Element, obstacle: dict, manifest: dict) -> None:
    geometry = ET.SubElement(parent, "geometry")
    obstacle_type = obstacle["type"]
    scale = obstacle["scale"]
    if obstacle_type == "cube":
        box = ET.SubElement(geometry, "box")
        ET.SubElement(box, "size").text = vector(scale)
    elif obstacle_type == "sphere":
        if not math.isclose(scale[0], scale[1]) or not math.isclose(scale[0], scale[2]):
            raise ValueError(f"{obstacle['name']}: sphere scale must be isotropic")
        sphere = ET.SubElement(geometry, "sphere")
        ET.SubElement(sphere, "radius").text = number(scale[0])
    elif obstacle_type == "cylinder":
        if not math.isclose(scale[0], scale[1]):
            raise ValueError(f"{obstacle['name']}: cylinder x/y scale must be equal")
        cylinder = ET.SubElement(geometry, "cylinder")
        ET.SubElement(cylinder, "radius").text = number(scale[0])
        ET.SubElement(cylinder, "length").text = number(scale[2])
    elif obstacle_type == "v_polytope":
        template = manifest["templates"][obstacle["v_polytope_type"]]
        mesh = ET.SubElement(geometry, "mesh")
        ET.SubElement(mesh, "uri").text = template["mesh_uri"]
        ET.SubElement(mesh, "scale").text = vector(scale)
    else:
        raise ValueError(f"{obstacle['name']}: unsupported type {obstacle_type}")


def world_xml(manifest: dict) -> bytes:
    sdf = ET.Element("sdf", {"version": "1.6"})
    world = ET.SubElement(sdf, "world", {"name": manifest["scene"]})
    physics = ET.SubElement(world, "physics", {"name": "ode", "type": "ode"})
    ET.SubElement(physics, "max_step_size").text = "0.001"
    ET.SubElement(physics, "real_time_update_rate").text = "1000"
    for model_uri in ("model://sun", "model://ground_plane"):
        include = ET.SubElement(world, "include")
        ET.SubElement(include, "uri").text = model_uri

    for obstacle in manifest["obstacles"]:
        model = ET.SubElement(world, "model", {"name": f"xgc2_obstacle_{obstacle['name']}"})
        ET.SubElement(model, "static").text = "true"
        roll, pitch, yaw = quaternion_to_rpy(obstacle["orientation_xyzw"])
        pose = [*obstacle["position"], roll, pitch, yaw]
        ET.SubElement(model, "pose", {"relative_to": "world"}).text = vector(pose)
        link = ET.SubElement(model, "link", {"name": "body"})
        collision = ET.SubElement(link, "collision", {"name": "collision"})
        add_geometry(collision, obstacle, manifest)
        visual = ET.SubElement(link, "visual", {"name": "visual"})
        add_geometry(visual, obstacle, manifest)
        material = ET.SubElement(visual, "material")
        ET.SubElement(material, "ambient").text = MATERIAL
        ET.SubElement(material, "diffuse").text = MATERIAL

    indent_xml(sdf)
    return serialize_xml(sdf)


def obj_text(template_name: str, template: dict) -> str:
    lines = [
        f"# Exact convex collision mesh for paper-leader template {template_name}.",
        "# Vertices remain unscaled; each world collision applies the source scale.",
        f"o {template_name}",
    ]
    lines.extend(f"v {vector(vertex)}" for vertex in template["vertices"])
    lines.extend("f " + " ".join(str(index + 1) for index in triangle) for triangle in template["triangles"])
    return "\n".join(lines) + "\n"


def model_config(model_name: str) -> bytes:
    root = ET.Element("model")
    ET.SubElement(root, "name").text = model_name
    ET.SubElement(root, "version").text = "1.0"
    sdf = ET.SubElement(root, "sdf", {"version": "1.6"})
    sdf.text = "model.sdf"
    author = ET.SubElement(root, "author")
    ET.SubElement(author, "name").text = "XGC2"
    ET.SubElement(author, "email").text = "lxk36@users.noreply.github.com"
    ET.SubElement(root, "description").text = "Exact convex collision template for XGC2 scenes."
    indent_xml(root)
    return serialize_xml(root)


def model_sdf(model_name: str, mesh_uri: str) -> bytes:
    root = ET.Element("sdf", {"version": "1.6"})
    model = ET.SubElement(root, "model", {"name": model_name})
    ET.SubElement(model, "static").text = "true"
    link = ET.SubElement(model, "link", {"name": "body"})
    for element_name in ("collision", "visual"):
        element = ET.SubElement(link, element_name, {"name": element_name})
        geometry = ET.SubElement(element, "geometry")
        mesh = ET.SubElement(geometry, "mesh")
        ET.SubElement(mesh, "uri").text = mesh_uri
    indent_xml(root)
    return serialize_xml(root)


def expected_assets(package_root: Path) -> dict[Path, bytes]:
    assets: dict[Path, bytes] = {}
    templates: dict[str, dict] = {}
    for scene_name in SCENES:
        scene_dir = package_root / "worlds" / scene_name
        manifest = json.loads((scene_dir / "source_manifest.json").read_text(encoding="utf-8"))
        if manifest["scene"] != scene_name:
            raise ValueError(f"{scene_dir}: manifest scene does not match directory")
        assets[scene_dir / f"{scene_name}.world"] = world_xml(manifest)
        for template_name, template in manifest["templates"].items():
            previous = templates.setdefault(template_name, template)
            if previous != template:
                raise ValueError(f"template {template_name} has conflicting definitions")

    for template_name, template in templates.items():
        uri = template["mesh_uri"]
        prefix = "model://"
        if not uri.startswith(prefix):
            raise ValueError(f"{template_name}: mesh URI must use model://")
        relative = Path(uri[len(prefix) :])
        if len(relative.parts) < 3 or relative.parts[1] != "meshes":
            raise ValueError(f"{template_name}: expected model://<model>/meshes/<file> URI")
        model_name = relative.parts[0]
        model_dir = package_root / "models" / model_name
        assets[model_dir / Path(*relative.parts[1:])] = obj_text(template_name, template).encode()
        assets[model_dir / "model.config"] = model_config(model_name)
        assets[model_dir / "model.sdf"] = model_sdf(model_name, uri)
    return assets


def apply_or_check(package_root: Path, check: bool) -> bool:
    failed = False
    for path, expected in expected_assets(package_root).items():
        current = path.read_bytes() if path.exists() else b""
        if current == expected:
            continue
        if check:
            failed = True
            before = current.decode(errors="replace").splitlines(keepends=True)
            after = expected.decode(errors="replace").splitlines(keepends=True)
            sys.stderr.writelines(
                difflib.unified_diff(before, after, fromfile=str(path), tofile=f"{path} (generated)")
            )
        else:
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(expected)
            print(f"generated {path.relative_to(package_root)}")
    return not failed


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--package-root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
        help="gazebo_sim_worlds package root",
    )
    parser.add_argument("--check", action="store_true", help="fail if generated assets are stale")
    arguments = parser.parse_args()
    return 0 if apply_or_check(arguments.package_root.resolve(), arguments.check) else 1


if __name__ == "__main__":
    raise SystemExit(main())
