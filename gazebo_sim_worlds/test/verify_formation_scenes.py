#!/usr/bin/env python3
"""Verify paper-leader source data and exact Gazebo collision assets."""

from __future__ import annotations

import argparse
from collections import Counter
import json
import math
import os
from pathlib import Path
import subprocess
import sys
from typing import Optional
import xml.etree.ElementTree as ET


SCENES = (
    "uav6_knot_obstacles",
    "ugv4_figure_eight_obstacles",
    "ugv4_figure_eight_scout_obstacles",
    "uav6_ugv4_crossing_obstacles",
)
REFERENCE_SOURCE_ENV = "XGC2_FORMATION_REFERENCE_SRC"
TOLERANCE = 1.0e-9


def close(left: float, right: float, tolerance: float = TOLERANCE) -> bool:
    return math.isclose(float(left), float(right), rel_tol=0.0, abs_tol=tolerance)


def vector_close(left: list[float], right: list[float], tolerance: float = TOLERANCE) -> bool:
    return len(left) == len(right) and all(close(a, b, tolerance) for a, b in zip(left, right))


def subtract(left: list[float], right: list[float]) -> list[float]:
    return [left[index] - right[index] for index in range(3)]


def cross(left: list[float], right: list[float]) -> list[float]:
    return [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]


def dot(left: list[float], right: list[float]) -> float:
    return sum(a * b for a, b in zip(left, right))


def parse_obj(path: Path) -> tuple[list[list[float]], list[tuple[int, int, int]]]:
    vertices: list[list[float]] = []
    triangles: list[tuple[int, int, int]] = []
    for line_number, raw_line in enumerate(path.read_text(encoding="utf-8").splitlines(), 1):
        line = raw_line.strip()
        if not line or line.startswith("#") or line.startswith("o "):
            continue
        fields = line.split()
        if fields[0] == "v":
            if len(fields) != 4:
                raise AssertionError(f"{path}:{line_number}: vertex must have 3 coordinates")
            vertices.append([float(value) for value in fields[1:]])
        elif fields[0] == "f":
            if len(fields) != 4:
                raise AssertionError(f"{path}:{line_number}: collision faces must be triangles")
            triangle = tuple(int(value.split("/")[0]) - 1 for value in fields[1:])
            triangles.append(triangle)
        else:
            raise AssertionError(f"{path}:{line_number}: unsupported OBJ record {fields[0]}")
    return vertices, triangles


def verify_closed_convex_mesh(
    path: Path, vertices: list[list[float]], triangles: list[tuple[int, int, int]]
) -> None:
    if len(vertices) < 4 or len(triangles) < 4:
        raise AssertionError(f"{path}: collision mesh is not a 3D closed polyhedron")
    edge_counts: Counter[tuple[int, int]] = Counter()
    referenced: set[int] = set()
    for triangle in triangles:
        if len(set(triangle)) != 3 or any(index < 0 or index >= len(vertices) for index in triangle):
            raise AssertionError(f"{path}: invalid triangle {triangle}")
        referenced.update(triangle)
        for start, end in zip(triangle, (triangle[1], triangle[2], triangle[0])):
            edge_counts[tuple(sorted((start, end)))] += 1
        a, b, c = (vertices[index] for index in triangle)
        normal = cross(subtract(b, a), subtract(c, a))
        if math.sqrt(dot(normal, normal)) <= TOLERANCE:
            raise AssertionError(f"{path}: degenerate triangle {triangle}")
        signed_distances = [dot(normal, subtract(vertex, a)) for vertex in vertices]
        if min(signed_distances) < -TOLERANCE and max(signed_distances) > TOLERANCE:
            raise AssertionError(f"{path}: triangle {triangle} is not on a convex hull plane")
    open_edges = [edge for edge, count in edge_counts.items() if count != 2]
    if open_edges:
        raise AssertionError(f"{path}: mesh is not closed; bad edges: {open_edges[:5]}")
    if referenced != set(range(len(vertices))):
        raise AssertionError(f"{path}: mesh contains unreferenced vertices")


def parse_vector(element: ET.Element) -> list[float]:
    if element.text is None:
        raise AssertionError(f"{element.tag}: missing vector text")
    return [float(value) for value in element.text.split()]


def expected_geometry(obstacle: dict, manifest: dict) -> tuple[str, dict]:
    obstacle_type = obstacle["type"]
    scale = obstacle["scale"]
    if obstacle_type == "cube":
        return "box", {"size": scale}
    if obstacle_type == "sphere":
        return "sphere", {"radius": scale[0]}
    if obstacle_type == "cylinder":
        return "cylinder", {"radius": scale[0], "length": scale[2]}
    if obstacle_type == "v_polytope":
        template = manifest["templates"][obstacle["v_polytope_type"]]
        return "mesh", {"uri": template["mesh_uri"], "scale": scale}
    raise AssertionError(f"unsupported obstacle type {obstacle_type}")


def read_geometry(geometry: ET.Element) -> tuple[str, dict]:
    children = list(geometry)
    if len(children) != 1:
        raise AssertionError("geometry must contain exactly one shape")
    shape = children[0]
    if shape.tag == "box":
        return "box", {"size": parse_vector(shape.find("size"))}
    if shape.tag == "sphere":
        return "sphere", {"radius": float(shape.findtext("radius"))}
    if shape.tag == "cylinder":
        return "cylinder", {
            "radius": float(shape.findtext("radius")),
            "length": float(shape.findtext("length")),
        }
    if shape.tag == "mesh":
        return "mesh", {
            "uri": shape.findtext("uri"),
            "scale": parse_vector(shape.find("scale")),
        }
    raise AssertionError(f"unsupported Gazebo shape {shape.tag}")


def geometry_equal(left: tuple[str, dict], right: tuple[str, dict]) -> bool:
    if left[0] != right[0] or left[1].keys() != right[1].keys():
        return False
    for key in left[1]:
        a, b = left[1][key], right[1][key]
        if isinstance(a, list):
            if not vector_close(a, b):
                return False
        elif isinstance(a, str):
            if a != b:
                return False
        elif not close(a, b):
            return False
    return True


def spawn_position(obstacle: dict) -> list[float]:
    """SDF spawn position: source pose, minus the runtime-motion spawn lead.

    Mirrors tools/generate_formation_scenes.py. ``position`` stays the source
    truth at mission t = 0 so the upstream comparison keeps checking it against
    paper-leader; a body whose twist is attached at runtime spawns earlier
    along its own velocity by ``spawn_lead_seconds``, because the scene
    plugin's MotionController binds its origin and its zero of time when the
    configure Job is served.
    """
    motion = obstacle.get("motion")
    if not motion:
        return list(obstacle["position"])
    lead = float(motion.get("spawn_lead_seconds", 0.0))
    if not math.isfinite(lead) or lead < 0.0:
        raise AssertionError(f"{obstacle['name']}: spawn_lead_seconds must be finite and >= 0")
    if lead == 0.0:
        return list(obstacle["position"])
    velocity = motion.get("linear_velocity") or [0.0, 0.0, 0.0]
    return [
        float(obstacle["position"][axis]) - float(velocity[axis]) * lead
        for axis in range(3)
    ]


def quaternion_to_rpy(quaternion: list[float]) -> list[float]:
    x, y, z, w = quaternion
    norm = math.sqrt(sum(component * component for component in quaternion))
    x, y, z, w = (component / norm for component in quaternion)
    return [
        math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)),
        math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x)))),
        math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)),
    ]


def resolve_model_uri(package_root: Path, uri: str) -> Path:
    prefix = "model://"
    if not uri.startswith(prefix):
        raise AssertionError(f"expected model URI, got {uri}")
    return package_root / "models" / uri[len(prefix) :]


def verify_world(package_root: Path, scene_name: str) -> None:
    scene_dir = package_root / "worlds" / scene_name
    manifest = json.loads((scene_dir / "source_manifest.json").read_text(encoding="utf-8"))
    world_path = scene_dir / f"{scene_name}.world"
    root = ET.parse(str(world_path)).getroot()
    world = root.find("world")
    if world is None or world.attrib.get("name") != scene_name:
        raise AssertionError(f"{world_path}: wrong world name")
    models = {model.attrib["name"]: model for model in world.findall("model")}
    expected_names = {f"xgc2_obstacle_{item['name']}" for item in manifest["obstacles"]}
    if set(models) != expected_names:
        raise AssertionError(f"{world_path}: managed model set differs from manifest")

    loaded_meshes: dict[str, tuple[list[list[float]], list[tuple[int, int, int]]]] = {}
    for obstacle in manifest["obstacles"]:
        model_name = f"xgc2_obstacle_{obstacle['name']}"
        model = models[model_name]
        if model.findtext("static") != "true":
            raise AssertionError(f"{model_name}: obstacle must be static")
        pose = parse_vector(model.find("pose"))
        expected_pose = [*spawn_position(obstacle), *quaternion_to_rpy(obstacle["orientation_xyzw"])]
        if not vector_close(pose, expected_pose):
            raise AssertionError(f"{model_name}: pose differs: {pose} != {expected_pose}")
        collision = model.find("link/collision/geometry")
        visual = model.find("link/visual/geometry")
        if collision is None or visual is None:
            raise AssertionError(f"{model_name}: collision and visual are both required")
        actual = read_geometry(collision)
        visual_geometry = read_geometry(visual)
        expected = expected_geometry(obstacle, manifest)
        if not geometry_equal(actual, expected):
            raise AssertionError(f"{model_name}: collision differs: {actual} != {expected}")
        if not geometry_equal(visual_geometry, expected):
            raise AssertionError(f"{model_name}: visual differs from collision")

        if actual[0] == "mesh":
            mesh_uri = actual[1]["uri"]
            mesh_path = resolve_model_uri(package_root, mesh_uri)
            template_name = obstacle["v_polytope_type"]
            if mesh_path.stem != template_name:
                raise AssertionError(
                    f"{model_name}: mesh filename '{mesh_path.stem}' cannot publish "
                    f"the required v_polytope alias '{template_name}'"
                )
            if mesh_uri not in loaded_meshes:
                loaded_meshes[mesh_uri] = parse_obj(mesh_path)
                verify_closed_convex_mesh(mesh_path, *loaded_meshes[mesh_uri])
            mesh_vertices, _ = loaded_meshes[mesh_uri]
            template = manifest["templates"][template_name]
            if len(mesh_vertices) != len(template["vertices"]):
                raise AssertionError(f"{model_name}: mesh vertex count differs from source template")
            scaled_mesh = sorted(
                tuple(vertex[index] * obstacle["scale"][index] for index in range(3))
                for vertex in mesh_vertices
            )
            scaled_source = sorted(
                tuple(vertex[index] * obstacle["scale"][index] for index in range(3))
                for vertex in template["vertices"]
            )
            if any(not vector_close(list(a), list(b)) for a, b in zip(scaled_mesh, scaled_source)):
                raise AssertionError(f"{model_name}: physical mesh vertices differ from source polytope")

    catalog = package_root / "worlds" / "catalog"
    expected_links = {
        catalog / f"{scene_name}.world": Path(f"../{scene_name}/{scene_name}.world"),
        catalog / f"{scene_name}.md": Path(f"../{scene_name}/README.md"),
    }
    for link, target in expected_links.items():
        if not link.is_symlink() or Path(os.readlink(str(link))) != target or not link.resolve().exists():
            raise AssertionError(f"{link}: missing or incorrect catalog symlink")


def default_reference_source() -> Optional[Path]:
    configured = os.environ.get(REFERENCE_SOURCE_ENV)
    return Path(configured).expanduser() if configured else None


def verify_upstream(
    package_root: Path, paper_src: Optional[Path], required: bool
) -> None:
    if paper_src is None:
        message = (
            "external formation reference source not configured; "
            "checked-in source snapshots remain fully verified"
        )
        if required:
            raise AssertionError(
                f"{message}; pass --paper-leader-src or set {REFERENCE_SOURCE_ENV}"
            )
        print(f"warning: {message}", file=sys.stderr)
        return

    paper_src = paper_src.resolve()
    try:
        import yaml
    except ImportError as error:
        if required:
            raise AssertionError("PyYAML is required for upstream comparison") from error
        print("warning: PyYAML unavailable; upstream comparison skipped", file=sys.stderr)
        return

    missing: list[Path] = []
    for scene_name in SCENES:
        scene_dir = package_root / "worlds" / scene_name
        manifest = json.loads((scene_dir / "source_manifest.json").read_text(encoding="utf-8"))
        source = manifest["source"]
        if "obstacles" not in source:
            # A scene may be DERIVED rather than copied (ugv4_figure_eight_scout
            # is placed by a slot sweep, not transcribed from an upstream
            # obstacles.yaml). Such a manifest declares `derivation` instead and
            # has no upstream file to compare against; its checked-in snapshot is
            # still fully verified by verify_world above.
            print(f"note: {scene_name} is derived, not transcribed; no upstream comparison")
            continue
        obstacle_path = paper_src / source["obstacles"]
        library_path = paper_src / source["polytope_library"]
        missing.extend(path for path in (obstacle_path, library_path) if not path.exists())
        if not obstacle_path.exists() or not library_path.exists():
            continue
        upstream = yaml.safe_load(obstacle_path.read_text(encoding="utf-8"))
        source_obstacles = upstream["obstacles"]
        if len(source_obstacles) != len(manifest["obstacles"]):
            raise AssertionError(f"{obstacle_path}: obstacle count differs from manifest")
        for source, snapshot in zip(source_obstacles, manifest["obstacles"]):
            fields = {
                "id": source["id"],
                "name": source["name"],
                "type": source["type"],
                "scale": source["geometry"]["scale"],
                "position": source["pose"]["position"],
                "orientation_xyzw": source["pose"]["orientation"],
            }
            if source["type"] == "v_polytope":
                fields["v_polytope_type"] = source["geometry"]["v_polytope_type"]
            if fields != {key: snapshot[key] for key in fields}:
                raise AssertionError(f"{obstacle_path}: source differs for {source['name']}")
        library = yaml.safe_load(library_path.read_text(encoding="utf-8"))
        for name, template in manifest["templates"].items():
            if library[name]["vertices"] != template["vertices"]:
                raise AssertionError(f"{library_path}: template {name} differs from manifest")
    if missing:
        message = "paper-leader inputs unavailable: " + ", ".join(str(path) for path in missing)
        if required:
            raise AssertionError(message)
        print(f"warning: {message}; upstream comparison skipped", file=sys.stderr)


def verify_all(
    package_root: Path,
    paper_leader_src: Optional[Path],
    require_upstream: bool,
) -> None:
    generator = package_root / "tools" / "generate_formation_scenes.py"
    subprocess.run([sys.executable, str(generator), "--package-root", str(package_root), "--check"], check=True)
    for scene_name in SCENES:
        verify_world(package_root, scene_name)
        manifest = json.loads(
            (package_root / "worlds" / scene_name / "source_manifest.json").read_text(encoding="utf-8")
        )
        print(
            f"verified {scene_name} {len(manifest['obstacles'])}/{len(manifest['obstacles'])}: "
            "type, position, quaternion, scale, collision, and convex vertices"
        )
    verify_upstream(package_root, paper_leader_src, require_upstream)
    print("verified upstream source snapshots, exact Gazebo assets, convex meshes, and catalog links")


def test_formation_scenes() -> None:
    """Catkin/nosetests entry point using the checked-in source snapshots."""
    verify_all(
        Path(__file__).resolve().parents[1],
        default_reference_source(),
        False,
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--package-root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
        help="gazebo_sim_worlds package root",
    )
    parser.add_argument(
        "--paper-leader-src",
        type=Path,
        default=default_reference_source(),
        help=(
            "optional extracted reference workspace src directory "
            f"(default: ${REFERENCE_SOURCE_ENV} when set)"
        ),
    )
    parser.add_argument("--require-upstream", action="store_true")
    arguments = parser.parse_args()
    verify_all(
        arguments.package_root.resolve(),
        arguments.paper_leader_src,
        arguments.require_upstream,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
