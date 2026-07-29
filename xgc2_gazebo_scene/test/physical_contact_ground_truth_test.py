#!/usr/bin/env python3
"""Rostest for the Gazebo-engine physical-contact regression truth."""

from __future__ import annotations

import threading
import time
import unittest

import rospy
import rostest
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty


class PhysicalContactGroundTruthTest(unittest.TestCase):
    def setUp(self) -> None:
        self.status = None
        self.detail = ""
        self.condition = threading.Condition()
        self.status_subscriber = rospy.Subscriber(
            "/xgc2/simulation/physical_collision",
            Bool,
            self._on_status,
            queue_size=1,
        )
        self.detail_subscriber = rospy.Subscriber(
            "/xgc2/simulation/physical_collision_detail",
            String,
            self._on_detail,
            queue_size=1,
        )

    def _on_status(self, message: Bool) -> None:
        with self.condition:
            self.status = bool(message.data)
            self.condition.notify_all()

    def _on_detail(self, message: String) -> None:
        with self.condition:
            self.detail = message.data
            self.condition.notify_all()

    def _wait_for(self, predicate, description: str) -> None:
        deadline = time.monotonic() + float(rospy.get_param("~timeout", 20.0))
        with self.condition:
            while not predicate():
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    self.fail("timed out waiting for {}".format(description))
                self.condition.wait(timeout=min(0.1, remaining))

    def test_ground_is_allowed_and_obstacle_contact_is_forbidden(self) -> None:
        self._wait_for(lambda: self.status is not None, "initial collision status")
        self.assertFalse(self.status)

        # The dynamic box rests on the ordinary static ground throughout this
        # interval. A ground contact must not poison the regression latch.
        # Wall time is intentional: this assertion should also finish with a
        # useful failure if the Gazebo simulation clock itself stalls.
        time.sleep(1.0)
        self.assertFalse(self.status)
        self.assertEqual("", self.detail)

        rospy.wait_for_service("/gazebo/set_model_state", timeout=10.0)
        set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        state = ModelState()
        state.model_name = "test_robot_1"
        state.reference_frame = "world"
        state.pose.position.x = 2.0
        state.pose.position.y = 0.0
        state.pose.position.z = 0.5
        state.pose.orientation.w = 1.0
        response = set_model_state(state)
        self.assertTrue(response.success, response.status_message)

        self._wait_for(lambda: self.status is True and bool(self.detail), "forbidden contact latch")
        self.assertIn("model1=", self.detail)
        self.assertIn("model2=", self.detail)
        self.assertIn("test_robot_1", self.detail)
        self.assertIn("xgc2_obstacle_test_box", self.detail)

        rospy.wait_for_service("/gazebo/reset_simulation", timeout=10.0)
        reset_simulation = rospy.ServiceProxy("/gazebo/reset_simulation", Empty)
        reset_simulation()
        self._wait_for(
            lambda: self.status is False and self.detail == "",
            "cleared contact latch after world reset",
        )


if __name__ == "__main__":
    rospy.init_node("physical_contact_ground_truth_test")
    rostest.rosrun(
        "xgc2_gazebo_scene",
        "physical_contact_ground_truth_test",
        PhysicalContactGroundTruthTest,
    )
