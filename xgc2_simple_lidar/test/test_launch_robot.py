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
ROBOT = {
    'slotId': 'slot-07', 'assetId': 'asset-07', 'assetCommitId': 'asset-commit-07',
    'name': 'Scout seven', 'kind': 'scout_mini', 'namespace': '/ugv7',
    'profile': {'id': 'profile-scout_mini', 'digest': 'digest-scout_mini',
                'providerDefinitionId': 'scout-provider', 'parameters': {}, 'channels': []},
    'binding': {'id': 'slot-07', 'ref': {'domain': 'robot', 'resourceId': 'asset-07', 'branch': 'main'},
                'namespace': '/ugv7', 'hybridSource': 'simulation', 'runtimeParameters': {},
                'initialPose': {'x': 1, 'y': 2, 'z': .181, 'yaw': .3},
                'simulationSensors': {'simpleLidar': False},
                'scout': {'lidarSimulationEnabled': True, 'imageSimulationEnabled': False}},
    'asset': {'name': 'Scout asset seven', 'kind': 'scout_mini', 'services': []},
}


def load():
    loader = importlib.machinery.SourceFileLoader('launch_robot_under_test', str(SCRIPT))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


class LaunchRobotTest(unittest.TestCase):
    def launch(self, *, value=None, extra=(), profile='scout', namespace='/ugv7', raw=None):
        module = load()
        value = {'robot': ROBOT, 'runMode': 'simulation'} if value is None else value
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
        self.assertIn('model_name:=slot-07', arguments)
        self.assertIn('robot_description_param:=/ugv7/robot_description', arguments)
        self.assertIn('frame_prefix:=ugv7/', arguments)
        self.assertIn('z:=0.181', arguments)
        self.assertIn('enable_lidar:=true', arguments)
        self.assertIn('enable_camera:=false', arguments)
        self.assertIn('run_mode:=simulation', arguments)
        self.assertFalse(any(arg.startswith(('target_id:=', 'simulation_service_ref_json:='))
                             for arg in arguments))

    def test_old_world_input_and_missing_robot_are_rejected(self):
        for value in ({}, {'robot': ROBOT}, {'runMode': 'simulation'},
                      {'robot': ROBOT, 'runMode': 'simulation', 'simulationServiceRef': {}},
                      {'robot': None, 'runMode': 'simulation'}):
            with self.subTest(value=value), redirect_stderr(io.StringIO()), \
                    self.assertRaises(SystemExit) as raised:
                self.launch(value=value)
            self.assertEqual(raised.exception.code, 2)

    def test_session_run_mode_is_passed_through_and_validated(self):
        for run_mode in ('simulation', 'physical', 'hybrid', 'custom-session-mode'):
            with self.subTest(run_mode=run_mode):
                _, arguments = self.launch(value={'robot': ROBOT, 'runMode': run_mode})
                self.assertIn('run_mode:=' + run_mode, arguments)
        for run_mode in ('', None, True, 1, {}):
            with self.subTest(run_mode=run_mode), redirect_stderr(io.StringIO()), \
                    self.assertRaises(SystemExit) as raised:
                self.launch(value={'robot': ROBOT, 'runMode': run_mode})
            self.assertEqual(raised.exception.code, 2)

    def test_missing_freezer_fields_are_usage_errors(self):
        for field in ('slotId', 'binding'):
            robot = copy.deepcopy(ROBOT)
            del robot[field]
            errors = io.StringIO()
            with self.subTest(field=field), redirect_stderr(errors), \
                    self.assertRaises(SystemExit) as raised:
                self.launch(value={'robot': robot, 'runMode': 'simulation'})
            self.assertEqual(raised.exception.code, 2)
            self.assertIn('robot.' + field, errors.getvalue())

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
            with self.subTest(options=options), redirect_stderr(io.StringIO()), \
                    self.assertRaises(SystemExit) as raised:
                self.launch(**options)
            self.assertEqual(raised.exception.code, 2)

    def test_oversized_or_malformed_robot_input_is_rejected(self):
        for raw in (b' ' * 65537, b'{'):
            with self.subTest(size=len(raw)), redirect_stderr(io.StringIO()), \
                    self.assertRaises(SystemExit) as raised:
                self.launch(raw=raw)
            self.assertEqual(raised.exception.code, 2)

    def test_scout_configuration_keeps_sensor_switches_names_and_pose(self):
        parameters = robot_parameters(ROBOT, 'scout', 'simulation')
        self.assertEqual(parameters['tf_prefix'], 'ugv7')
        self.assertEqual(parameters['robot_state_publisher_ns'], 'ugv7')
        self.assertEqual(parameters['sensor_ns'], 'ugv7')
        self.assertEqual(parameters['run_mode'], 'simulation')
        self.assertEqual(parameters['yaw'], .3)
        robot = copy.deepcopy(ROBOT)
        robot['binding']['initialPose']['x'] = float('nan')
        with self.assertRaises(ValueError):
            robot_parameters(robot, 'scout', 'simulation')

    def test_mecanum_and_surface_configuration_remain_importable(self):
        robot = copy.deepcopy(ROBOT)
        robot['kind'] = robot['asset']['kind'] = 'mecanum_ugv'
        del robot['binding']['scout']
        robot['binding']['mecanum'] = {}
        robot['profile'].update(id='profile-mecanum_ugv', digest='digest-mecanum_ugv',
                                providerDefinitionId='mecanum-provider')
        parameters = robot_parameters(robot, 'mecanum', 'simulation')
        self.assertEqual((parameters['ns'], parameters['model_name'], parameters['x']), ('ugv7', 'slot-07', 1))
        self.assertNotIn('robot_description_param', parameters)
        parameters = surface_parameters({'simpleLidar': {'enabled': True, 'acceleration': 'cpu',
                                                        'rateHz': 12, 'rangeMeters': 30}})
        self.assertTrue(parameters['enable_simple_lidar'])
        self.assertEqual(parameters['simple_lidar_acceleration'], 'cpu')
        self.assertEqual(parameters['simple_lidar_rate_hz'], 12)
        self.assertEqual(parameters['simple_lidar_range_meters'], 30)


if __name__ == '__main__':
    unittest.main()
