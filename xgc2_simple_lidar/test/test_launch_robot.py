#!/usr/bin/env python3
"""The robot launcher waits for the session ROS master; it never lets roslaunch start a second master."""
import importlib.machinery
import importlib.util
import io
import json
from pathlib import Path
import sys
import unittest
from unittest import mock

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/launch_robot.py'
ROBOT = {'id': 'ugv7', 'kind': 'scout_mini', 'namespace': '/ugv7', 'runMode': 'simulation',
         'initialPose': {'x': 1, 'y': 2, 'z': .181, 'yaw': .3},
         'authoredSimulationSensors': {'simpleLidar': False},
         'scout': {'lidarSimulationEnabled': True, 'imageSimulationEnabled': False}}
SERVICE_REF = {'service': 'xgc2.simulation', 'api_version': 'v1', 'instance_id': 'i1'}


def load():
    loader = importlib.machinery.SourceFileLoader('launch_robot_under_test', str(SCRIPT))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


class LaunchRobotTest(unittest.TestCase):
    def launch(self):
        module = load()
        calls = []
        stdin = mock.Mock(buffer=io.BytesIO(json.dumps({'robot': ROBOT, 'simulationServiceRef': SERVICE_REF}).encode()))
        argv = ['launch_robot.py', '--profile', 'scout', '--namespace', '/ugv7', '--target-id', 'local',
                'robot.launch', 'gui:=false']
        with mock.patch.object(sys, 'argv', argv), mock.patch.object(sys, 'stdin', stdin), \
                mock.patch.object(module.os, 'execv', lambda path, arguments: calls.append((path, arguments))):
            module.main()
        self.assertEqual(len(calls), 1)
        return calls[0]

    def test_roslaunch_waits_for_the_session_master(self):
        path, arguments = self.launch()
        self.assertEqual(path, '/opt/ros/noetic/bin/roslaunch')
        self.assertEqual(arguments[:3], ['/opt/ros/noetic/bin/roslaunch', '--wait', 'robot.launch'])

    def test_frozen_robot_input_still_follows_the_launch_arguments(self):
        _, arguments = self.launch()
        self.assertIn('gui:=false', arguments)
        self.assertIn('ns:=ugv7', arguments)
        self.assertIn('target_id:=local', arguments)


if __name__ == '__main__':
    unittest.main()
