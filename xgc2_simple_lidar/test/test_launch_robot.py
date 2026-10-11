#!/usr/bin/env python3
"""Scout runtime lowering and reusable roster configuration without a ROS master."""
import copy
import importlib.machinery
import importlib.util
import io
import json
from contextlib import redirect_stderr
from pathlib import Path
import sys
import unittest
from unittest import mock

from xgc2_simple_lidar.configuration import robot_parameters, surface_parameters

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/launch_robot.py'
ROBOT = {'id': 'ugv7', 'kind': 'scout_mini', 'namespace': '/ugv7', 'runMode': 'simulation',
         'initialPose': {'x': 1, 'y': 2, 'z': .181, 'yaw': .3},
         'authoredSimulationSensors': {'simpleLidar': False},
         'scout': {'lidarSimulationEnabled': True, 'imageSimulationEnabled': False}}


def load():
    loader = importlib.machinery.SourceFileLoader('launch_robot_under_test', str(SCRIPT))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


class LaunchRobotTest(unittest.TestCase):
    def launch(self, *, value=None, extra=(), profile='scout', namespace='/ugv7', raw=None):
        module = load()
        value = {'robot': ROBOT} if value is None else value
        stdin = mock.Mock(buffer=io.BytesIO(json.dumps(value).encode() if raw is None else raw))
        argv = ['launch_robot.py', '--profile', profile, '--namespace', namespace,
                'robot.launch', 'gui:=false', *extra]
        with mock.patch.object(sys, 'argv', argv), mock.patch.object(sys, 'stdin', stdin), \
                mock.patch.object(module.os, 'execv') as execute:
            module.main()
        execute.assert_called_once()
        return execute.call_args.args

    def test_roslaunch_waits_for_the_session_master(self):
        path, arguments = self.launch()
        self.assertEqual(path, '/opt/ros/noetic/bin/roslaunch')
        self.assertEqual(arguments[:3], ['/opt/ros/noetic/bin/roslaunch', '--wait', 'robot.launch'])

    def test_only_robot_parameters_follow_the_launch_arguments(self):
        _, arguments = self.launch()
        self.assertIn('gui:=false', arguments)
        self.assertIn('ns:=ugv7', arguments)
        self.assertIn('model_name:=ugv7', arguments)
        self.assertIn('robot_description_param:=/ugv7/robot_description', arguments)
        self.assertIn('frame_prefix:=ugv7/', arguments)
        self.assertIn('z:=0.181', arguments)
        self.assertIn('enable_lidar:=true', arguments)
        self.assertIn('enable_camera:=false', arguments)
        self.assertFalse(any(arg.startswith(('target_id:=', 'simulation_service_ref_json:='))
                             for arg in arguments))

    def test_old_world_input_and_missing_robot_are_rejected(self):
        for value in ({}, {'robot': ROBOT, 'simulationServiceRef': {}}, {'robot': None}):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.launch(value=value)

    def test_mecanum_profile_target_option_and_unknown_options_are_rejected(self):
        for options in ({'profile': 'mecanum'}, {'extra': ('--target-id', 'local')},
                        {'extra': ('--unknown',)}, {'extra': ('plain-argument',)}):
            with self.subTest(options=options), redirect_stderr(io.StringIO()), \
                    self.assertRaises(SystemExit) as raised:
                self.launch(**options)
            self.assertEqual(raised.exception.code, 2)

    def test_namespace_and_frozen_parameter_overrides_are_rejected(self):
        for options in ({'namespace': '/ugv8'}, {'extra': ('ns:=other',)},
                        {'extra': ('enable_lidar:=false',)}, {'extra': ('model_name:=other',)}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                self.launch(**options)

    def test_oversized_or_malformed_robot_input_is_rejected(self):
        for raw in (b' ' * 65537, b'{'):
            with self.subTest(size=len(raw)), self.assertRaises(ValueError):
                self.launch(raw=raw)

    def test_scout_configuration_keeps_sensor_switches_names_and_pose(self):
        parameters = robot_parameters(ROBOT, 'scout', '/ugv7')
        self.assertEqual(parameters['tf_prefix'], 'ugv7')
        self.assertEqual(parameters['robot_state_publisher_ns'], 'ugv7')
        self.assertEqual(parameters['sensor_ns'], 'ugv7')
        self.assertEqual(parameters['run_mode'], 'simulation')
        self.assertEqual(parameters['yaw'], .3)
        robot = copy.deepcopy(ROBOT)
        robot['initialPose']['x'] = float('nan')
        with self.assertRaises(ValueError):
            robot_parameters(robot, 'scout', '/ugv7')

    def test_mecanum_and_surface_configuration_remain_importable(self):
        robot = copy.deepcopy(ROBOT)
        robot['kind'] = 'mecanum_ugv'
        parameters = robot_parameters(robot, 'mecanum', '/ugv7')
        self.assertEqual((parameters['ns'], parameters['model_name'], parameters['x']), ('ugv7', 'ugv7', 1))
        self.assertNotIn('robot_description_param', parameters)
        parameters = surface_parameters({'simpleLidar': {'enabled': True, 'acceleration': 'cpu',
                                                        'rateHz': 12, 'rangeMeters': 30}})
        self.assertTrue(parameters['enable_simple_lidar'])
        self.assertEqual(parameters['simple_lidar_acceleration'], 'cpu')
        self.assertEqual(parameters['simple_lidar_rate_hz'], 12)
        self.assertEqual(parameters['simple_lidar_range_meters'], 30)


if __name__ == '__main__':
    unittest.main()
