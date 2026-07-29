#!/usr/bin/env python3
"""Rostest contract for collision-derived geometry and state publication."""

from __future__ import annotations

import json
import math
from pathlib import Path
import threading
import time
import unittest

import rospy
import rostest
from std_msgs.msg import Bool

from xgc2_gazebo_scene.msg import ConvexPart
from xgc2_gazebo_scene.msg import ObstacleDefinitionArray
from xgc2_gazebo_scene.msg import ObstacleStateArray
from xgc2_geometry_msgs.msg import ConvexBodyArray
from xgc2_geometry_msgs.msg import GeometryLibrary


TOLERANCE = 1.0e-6
MESH_TOLERANCE = 2.0e-6


def close(left: float, right: float, tolerance: float = TOLERANCE) -> bool:
    return math.isclose(float(left), float(right), rel_tol=0.0, abs_tol=tolerance)


def normalized(quaternion: list[float]) -> list[float]:
    norm = math.sqrt(sum(value * value for value in quaternion))
    return [value / norm for value in quaternion]


def sorted_vertices(vertices) -> list[tuple[float, float, float]]:
    return sorted((float(vertex.x), float(vertex.y), float(vertex.z)) for vertex in vertices)


class SceneGroundTruthTest(unittest.TestCase):
    def setUp(self) -> None:
        manifest_path = Path(rospy.get_param("~manifest"))
        self.manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        self.geometry = None
        self.state = None
        self.geometry_library = None
        self.instances = None
        self.physical_collision = None
        self.condition = threading.Condition()
        self.geometry_subscriber = rospy.Subscriber(
            "/xgc2/simulation/obstacles/geometry",
            ObstacleDefinitionArray,
            self._on_geometry,
            queue_size=1,
        )
        self.state_subscriber = rospy.Subscriber(
            "/xgc2/simulation/obstacles/state",
            ObstacleStateArray,
            self._on_state,
            queue_size=1,
        )
        self.geometry_library_subscriber = rospy.Subscriber(
            "/xgc2/simulation/obstacles/geometry_library",
            GeometryLibrary,
            self._on_geometry_library,
            queue_size=1,
        )
        self.instances_subscriber = rospy.Subscriber(
            "/xgc2/simulation/obstacles/instances",
            ConvexBodyArray,
            self._on_instances,
            queue_size=1,
        )
        self.physical_collision_subscriber = rospy.Subscriber(
            "/xgc2/simulation/physical_collision",
            Bool,
            self._on_physical_collision,
            queue_size=1,
        )

    def _on_geometry(self, message: ObstacleDefinitionArray) -> None:
        with self.condition:
            self.geometry = message
            self.condition.notify_all()

    def _on_state(self, message: ObstacleStateArray) -> None:
        with self.condition:
            self.state = message
            self.condition.notify_all()

    def _on_geometry_library(self, message: GeometryLibrary) -> None:
        with self.condition:
            self.geometry_library = message
            self.condition.notify_all()

    def _on_instances(self, message: ConvexBodyArray) -> None:
        with self.condition:
            self.instances = message
            self.condition.notify_all()

    def _on_physical_collision(self, message: Bool) -> None:
        with self.condition:
            self.physical_collision = bool(message.data)
            self.condition.notify_all()

    def wait_for_messages(self) -> None:
        timeout = float(rospy.get_param("~timeout", 30.0))
        deadline = time.monotonic() + timeout
        with self.condition:
            while (
                self.geometry is None
                or self.state is None
                or self.geometry_library is None
                or self.instances is None
                or self.physical_collision is None
            ):
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    self.fail("timed out waiting for the scene geometry/state topics")
                self.condition.wait(timeout=min(0.25, remaining))

    def test_collision_truth_matches_manifest(self) -> None:
        self.wait_for_messages()
        self.assertEqual("world", self.geometry.header.frame_id)
        self.assertEqual("world", self.state.header.frame_id)
        self.assertEqual("world", self.geometry_library.header.frame_id)
        self.assertEqual("world", self.instances.header.frame_id)
        self.assertFalse(self.physical_collision)
        self.assertEqual(self.geometry.scene_epoch, self.state.scene_epoch)
        self.assertEqual(self.geometry.scene_revision, self.state.scene_revision)

        definitions = {obstacle.name: obstacle for obstacle in self.geometry.obstacles}
        states = {obstacle.name: obstacle for obstacle in self.state.obstacles}
        expected_names = {obstacle["name"] for obstacle in self.manifest["obstacles"]}
        self.assertEqual(expected_names, set(definitions))
        self.assertEqual(expected_names, set(states))
        standard_instances = {instance.name: instance for instance in self.instances.instances}
        self.assertEqual(expected_names, set(standard_instances))
        templates = {template.type: template for template in self.geometry_library.templates}

        shape_types = {
            "cube": ConvexPart.SHAPE_BOX,
            "sphere": ConvexPart.SHAPE_SPHERE,
            "cylinder": ConvexPart.SHAPE_CYLINDER,
            "v_polytope": ConvexPart.SHAPE_CONVEX_MESH,
        }
        for obstacle in self.manifest["obstacles"]:
            name = obstacle["name"]
            definition = definitions[name]
            state = states[name]
            standard_instance = standard_instances[name]
            self.assertEqual(f"xgc2_obstacle_{name}", definition.model_name)
            self.assertEqual(definition.model_name, state.model_name)
            self.assertEqual(1, definition.generation)
            self.assertEqual(definition.generation, state.generation)
            self.assertEqual(1, len(definition.parts))
            part = definition.parts[0]
            self.assertEqual("body/collision", part.part_id)
            self.assertEqual(shape_types[obstacle["type"]], part.shape)
            self.assertTrue(close(0.0, part.local_pose.position.x))
            self.assertTrue(close(0.0, part.local_pose.position.y))
            self.assertTrue(close(0.0, part.local_pose.position.z))
            self.assertTrue(close(1.0, abs(part.local_pose.orientation.w)))

            scale = obstacle["scale"]
            expected_standard_type = obstacle["type"]
            if expected_standard_type == "v_polytope":
                template = self.manifest["templates"][obstacle["v_polytope_type"]]
                expected_standard_type = f"convex_mesh:{template['mesh_uri']}"
            self.assertEqual(expected_standard_type, standard_instance.geometry_type)
            self.assertTrue(standard_instance.is_static)
            self.assertTrue(close(scale[0], standard_instance.scale.x))
            self.assertTrue(close(scale[1], standard_instance.scale.y))
            self.assertTrue(close(scale[2], standard_instance.scale.z))
            self.assertTrue(close(0.0, standard_instance.velocity.linear.x))
            self.assertTrue(close(0.0, standard_instance.velocity.linear.y))
            self.assertTrue(close(0.0, standard_instance.velocity.linear.z))
            self.assertTrue(close(0.0, standard_instance.velocity.angular.x))
            self.assertTrue(close(0.0, standard_instance.velocity.angular.y))
            self.assertTrue(close(0.0, standard_instance.velocity.angular.z))
            if obstacle["type"] == "cube":
                self.assertTrue(close(scale[0], part.size.x))
                self.assertTrue(close(scale[1], part.size.y))
                self.assertTrue(close(scale[2], part.size.z))
                unit_box = sorted(
                    (x, y, z)
                    for x in (-0.5, 0.5)
                    for y in (-0.5, 0.5)
                    for z in (-0.5, 0.5)
                )
                self.assertEqual(unit_box, sorted_vertices(templates["cube"].support_points))
            elif obstacle["type"] == "sphere":
                self.assertTrue(close(scale[0], part.radius))
                self.assertEqual(0, len(templates["sphere"].support_points))
            elif obstacle["type"] == "cylinder":
                self.assertTrue(close(scale[0], part.radius))
                self.assertTrue(close(scale[2], part.length))
                self.assertEqual(0, len(templates["cylinder"].support_points))
            else:
                template = self.manifest["templates"][obstacle["v_polytope_type"]]
                self.assertEqual(template["mesh_uri"], part.mesh_uri)
                self.assertTrue(close(scale[0], part.mesh_scale.x))
                self.assertTrue(close(scale[1], part.mesh_scale.y))
                self.assertTrue(close(scale[2], part.mesh_scale.z))
                expected_vertices = sorted(
                    tuple(vertex[index] * scale[index] for index in range(3))
                    for vertex in template["vertices"]
                )
                actual_vertices = sorted_vertices(part.conservative_vertices)
                self.assertEqual(len(expected_vertices), len(actual_vertices))
                for expected, actual in zip(expected_vertices, actual_vertices):
                    for expected_component, actual_component in zip(expected, actual):
                        self.assertTrue(close(expected_component, actual_component, MESH_TOLERANCE))
                standard_template_vertices = sorted_vertices(
                    templates[expected_standard_type].support_points
                )
                legacy_template_type = f"v_polytope:{obstacle['v_polytope_type']}"
                legacy_template_vertices = sorted_vertices(
                    templates[legacy_template_type].support_points
                )
                expected_template_vertices = sorted(tuple(vertex) for vertex in template["vertices"])
                self.assertEqual(len(expected_template_vertices), len(standard_template_vertices))
                self.assertEqual(standard_template_vertices, legacy_template_vertices)
                for expected, actual in zip(expected_template_vertices, standard_template_vertices):
                    for expected_component, actual_component in zip(expected, actual):
                        self.assertTrue(close(expected_component, actual_component, MESH_TOLERANCE))

            self.assertTrue(close(obstacle["position"][0], state.pose.position.x))
            self.assertTrue(close(obstacle["position"][1], state.pose.position.y))
            self.assertTrue(close(obstacle["position"][2], state.pose.position.z))
            expected_quaternion = normalized(obstacle["orientation_xyzw"])
            actual_quaternion = [
                state.pose.orientation.x,
                state.pose.orientation.y,
                state.pose.orientation.z,
                state.pose.orientation.w,
            ]
            quaternion_dot = abs(sum(a * b for a, b in zip(expected_quaternion, actual_quaternion)))
            self.assertTrue(close(1.0, quaternion_dot))
            self.assertTrue(close(obstacle["position"][0], standard_instance.pose.position.x))
            self.assertTrue(close(obstacle["position"][1], standard_instance.pose.position.y))
            self.assertTrue(close(obstacle["position"][2], standard_instance.pose.position.z))
            standard_quaternion = [
                standard_instance.pose.orientation.x,
                standard_instance.pose.orientation.y,
                standard_instance.pose.orientation.z,
                standard_instance.pose.orientation.w,
            ]
            standard_dot = abs(sum(a * b for a, b in zip(expected_quaternion, standard_quaternion)))
            self.assertTrue(close(1.0, standard_dot))

        expected_template_types = {"cube", "sphere", "cylinder"}
        expected_template_types.update(
            f"convex_mesh:{template['mesh_uri']}" for template in self.manifest["templates"].values()
        )
        expected_template_types.update(
            f"v_polytope:{template_name}" for template_name in self.manifest["templates"]
        )
        self.assertEqual(expected_template_types, set(templates))


if __name__ == "__main__":
    rospy.init_node("scene_ground_truth_test")
    rostest.rosrun("xgc2_gazebo_scene", "scene_ground_truth_test", SceneGroundTruthTest)
