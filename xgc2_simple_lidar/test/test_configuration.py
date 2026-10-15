#!/usr/bin/env python3
"""Shared launch parameters derived from freezer rows without ROS."""
import copy
import math
import unittest
from unittest import mock
from xgc2_simple_lidar import configuration

from xgc2_simple_lidar.configuration import robot_parameters, surface_parameters

SCOUT_ROW = {
    'slotId': 'slot-07', 'assetId': 'asset-07', 'assetCommitId': 'asset-commit-07',
    'name': 'Scout seven', 'kind': 'scout_mini', 'namespace': '/ugv7',
    'profile': {'id': 'profile-scout_mini', 'digest': 'digest-scout_mini',
                'providerDefinitionId': 'scout-provider', 'parameters': {}, 'channels': []},
    'binding': {'id': 'slot-07', 'ref': {'domain': 'robot', 'resourceId': 'asset-07', 'branch': 'main'},
                'namespace': '/ugv7', 'hybridSource': 'simulation', 'runtimeParameters': {},
                'initialPose': {'x': 1, 'y': 2, 'z': .181, 'yaw': .3},
                'simulationSensors': {'simpleLidar': {
                    'enabled': True, 'acceleration': 'cpu', 'rateHz': 12, 'rangeMeters': 30,
                    'hFovDeg': 240, 'vFovDeg': 45, 'hRes': 120, 'vRes': 8}},
                'scout': {'lidarSimulationEnabled': True, 'imageSimulationEnabled': True}},
    'asset': {'name': 'Scout asset seven', 'kind': 'scout_mini', 'services': []},
}
MECANUM_ROW = copy.deepcopy(SCOUT_ROW)
MECANUM_ROW.update(name='Mecanum seven', kind='mecanum_ugv')
MECANUM_ROW['profile'].update(id='profile-mecanum_ugv', digest='digest-mecanum_ugv',
                              providerDefinitionId='mecanum-provider')
MECANUM_ROW['asset'].update(name='Mecanum asset seven', kind='mecanum_ugv')
del MECANUM_ROW['binding']['scout']
MECANUM_ROW['binding']['mecanum'] = {}

COMMON_PARAMETERS = {
    'ns': 'ugv7', 'model_name': 'slot-07', 'x': 1, 'y': 2, 'z': .181, 'yaw': .3,
    'enable_simple_lidar': True, 'simple_lidar_acceleration': 'cpu',
    'simple_lidar_rate_hz': 12, 'simple_lidar_range_meters': 30,
    'simple_lidar_hfov_deg': 240, 'simple_lidar_vfov_deg': 45,
    'simple_lidar_hres': 120, 'simple_lidar_vres': 8,
}


class ConfigurationTest(unittest.TestCase):
    def test_scout_parameters_keep_the_launch_contract(self):
        expected = dict(COMMON_PARAMETERS, robot_description_param='/ugv7/robot_description',
                        tf_prefix='ugv7', frame_prefix='ugv7/', sensor_ns='ugv7',
                        robot_state_publisher_ns='ugv7', run_mode='hybrid')
        self.assertEqual(robot_parameters(SCOUT_ROW, 'scout', 'hybrid'), expected)

    def test_mecanum_parameters_keep_the_launch_contract(self):
        self.assertEqual(robot_parameters(MECANUM_ROW, 'mecanum', 'simulation'), COMMON_PARAMETERS)

    def test_omitted_simulation_sensors_use_surface_defaults(self):
        defaults = {'enable_simple_lidar': False, 'simple_lidar_acceleration': 'gpu',
                    'simple_lidar_rate_hz': 10, 'simple_lidar_range_meters': 20,
                    'simple_lidar_hfov_deg': 360, 'simple_lidar_vfov_deg': 180/math.pi,
                    'simple_lidar_hres': 360, 'simple_lidar_vres': 16}
        for row, profile in ((SCOUT_ROW, 'scout'), (MECANUM_ROW, 'mecanum')):
            row = copy.deepcopy(row)
            del row['binding']['simulationSensors']
            with self.subTest(profile=profile):
                parameters = robot_parameters(row, profile, 'simulation')
                for key, value in defaults.items():
                    self.assertEqual(parameters[key], value)

    def test_scout_private_sensor_switches_do_not_enable_producers(self):
        for switches in ({}, {'lidarSimulationEnabled': True, 'imageSimulationEnabled': True}):
            row = copy.deepcopy(SCOUT_ROW)
            row['binding']['scout'] = switches
            parameters = robot_parameters(row, 'scout', 'simulation')
            self.assertNotIn('enable_lidar', parameters)
            self.assertNotIn('enable_camera', parameters)
            self.assertTrue(parameters['enable_simple_lidar'])

    def test_namespace_must_be_one_absolute_ros_segment(self):
        for namespace in (None, 7, '', 'ugv7', '/', '/ugv7/child', '/ugv-7'):
            row = copy.deepcopy(SCOUT_ROW)
            row['namespace'] = namespace
            with self.subTest(namespace=namespace), self.assertRaisesRegex(ValueError, 'robot.namespace'):
                robot_parameters(row, 'scout', 'simulation')

    def test_kind_must_match_the_native_profile(self):
        for row, profile in ((SCOUT_ROW, 'mecanum'), (MECANUM_ROW, 'scout')):
            with self.subTest(profile=profile), self.assertRaisesRegex(ValueError, 'kind differs'):
                robot_parameters(row, profile, 'simulation')

    def test_missing_freezer_fields_name_the_required_field(self):
        for field in ('slotId', 'binding'):
            row = copy.deepcopy(SCOUT_ROW)
            del row[field]
            with self.subTest(field=field), self.assertRaisesRegex(ValueError, 'robot.' + field):
                robot_parameters(row, 'scout', 'simulation')
        row = copy.deepcopy(SCOUT_ROW)
        del row['binding']['initialPose']
        with self.assertRaisesRegex(ValueError, 'binding.initialPose'):
            robot_parameters(row, 'scout', 'simulation')

    def test_pose_coordinates_must_be_present_and_finite_numbers(self):
        for field in ('x', 'y', 'z', 'yaw'):
            for value in (None, True, '1', [], float('nan'), float('inf'), -float('inf')):
                row = copy.deepcopy(SCOUT_ROW)
                row['binding']['initialPose'][field] = value
                with self.subTest(field=field, value=value), \
                        self.assertRaisesRegex(ValueError, 'binding.initialPose.' + field):
                    robot_parameters(row, 'scout', 'simulation')
            row = copy.deepcopy(SCOUT_ROW)
            del row['binding']['initialPose'][field]
            with self.subTest(missing=field), self.assertRaisesRegex(ValueError, 'binding.initialPose.' + field):
                robot_parameters(row, 'scout', 'simulation')

    def test_row_binding_pose_and_scout_must_be_objects(self):
        with self.assertRaisesRegex(ValueError, 'freezer row object'):
            robot_parameters(None, 'scout', 'simulation')
        for field in ('binding', 'initialPose', 'scout'):
            for value in (None, [], True):
                row = copy.deepcopy(SCOUT_ROW)
                target = row if field == 'binding' else row['binding']
                target[field] = value
                with self.subTest(field=field, value=value), self.assertRaisesRegex(ValueError, field):
                    robot_parameters(row, 'scout', 'simulation')

    def test_surface_parameters_remain_importable(self):
        parameters = surface_parameters({'simpleLidar': {'enabled': True, 'acceleration': 'cpu',
                                                        'rateHz': 12, 'rangeMeters': 30}})
        self.assertTrue(parameters['enable_simple_lidar'])
        self.assertEqual(parameters['simple_lidar_acceleration'], 'cpu')
        self.assertEqual(parameters['simple_lidar_rate_hz'], 12)
        self.assertEqual(parameters['simple_lidar_range_meters'], 30)

    def test_gazebo_rejects_unsupported_observation_requests(self):
        for field, value in (('mode', 'penetrating'), ('mode', 'depth_frustum'),
                             ('preset', 'bridge_equivalent'), ('preset', 'zju_cpu_crop'),
                             ('bodyReturns', True), ('publishBeams', True), ('bodyReturns', 'false')):
            with self.subTest(field=field, value=value), self.assertRaisesRegex(ValueError, field):
                surface_parameters({'simpleLidar': {'enabled': True, field: value}})
        native = {'enabled': True, 'acceleration': 'cpu'}
        self.assertEqual(surface_parameters({'simpleLidar': native}), surface_parameters({'simpleLidar': {
            **native, 'mode': 'raycast', 'preset': '', 'bodyReturns': False, 'publishBeams': False}}))


class AutoBackendTest(unittest.TestCase):
    def tearDown(self):
        configuration.gpu_available.cache_clear()

    def test_acceleration_rejects_non_strings_before_backend_probe(self):
        for enabled in (False, True):
            with mock.patch.object(configuration, 'gpu_available') as probe:
                for acceleration in (None, False, 0, [], {}, True, 1, 1.5, ['gpu'], {'backend': 'cpu'}, 'cuda'):
                    with self.subTest(enabled=enabled, acceleration=acceleration), \
                            self.assertRaisesRegex(ValueError, r'simpleLidar\.acceleration'):
                        surface_parameters({'simpleLidar': {'enabled': enabled, 'acceleration': acceleration}})
                probe.assert_not_called()

    def test_acceleration_preserves_optional_string_contract(self):
        for enabled in (False, True):
            for acceleration in (None, '', 'auto', 'cpu', 'gpu'):
                for available in (False, True):
                    lidar = {'enabled': enabled}
                    if acceleration is not None:
                        lidar['acceleration'] = acceleration
                    with self.subTest(enabled=enabled, acceleration=acceleration, available=available), \
                            mock.patch.object(configuration, 'gpu_available', return_value=available) as probe:
                        value = surface_parameters({'simpleLidar': lidar})
                        automatic = acceleration in (None, '', 'auto')
                        expected = ('gpu' if available or not enabled else 'cpu') if automatic else acceleration
                        self.assertEqual(value['simple_lidar_acceleration'], expected)
                        self.assertEqual(probe.call_count, int(enabled and automatic))

    def test_auto_probes_once_and_preserves_native_observation(self):
        for available in (True, False):
            configuration.gpu_available.cache_clear()
            with mock.patch.object(configuration.subprocess, 'run', return_value=mock.Mock(returncode=0 if available else 1)) as probe:
                for _ in range(3):
                    value = surface_parameters({'simpleLidar': {'enabled': True, 'acceleration': 'auto'}})
                    self.assertTrue(value['enable_simple_lidar'])
                    self.assertEqual(value['simple_lidar_acceleration'], 'gpu' if available else 'cpu')
                self.assertEqual(probe.call_count, 1)

    def test_probe_uses_gazebo_native_binary_and_isolated_master(self):
        with mock.patch.object(configuration.subprocess, 'run', return_value=mock.Mock(returncode=0)) as probe:
            self.assertTrue(configuration.gpu_available())
        arguments = probe.call_args
        self.assertEqual(arguments.args[0], ['xgc2-gazebo-ray-probe'])
        self.assertIn('GAZEBO_MASTER_URI', arguments.kwargs['env'])
        self.assertEqual(arguments.kwargs['timeout'], 10)

    def test_native_explicit_backend_preserves_scan(self):
        for backend in ('cpu', 'gpu'):
            with mock.patch.object(configuration.subprocess, 'run') as probe:
                value = surface_parameters({'simpleLidar': {
                    'enabled': True, 'mode': 'raycast', 'acceleration': backend,
                    'rateHz': 4, 'rangeMeters': 37, 'hFovDeg': 270,
                    'vFovDeg': 50, 'hRes': 720, 'vRes': 32}})
                for field, expected in (('rate_hz', 4), ('range_meters', 37),
                        ('hfov_deg', 270), ('vfov_deg', 50), ('hres', 720), ('vres', 32)):
                    self.assertEqual(value['simple_lidar_' + field], expected)
                self.assertEqual(value['simple_lidar_acceleration'], backend)
                probe.assert_not_called()

    def test_author_disabled_stays_disabled_without_an_auto_probe(self):
        with mock.patch.object(configuration.subprocess, 'run') as probe:
            value = surface_parameters({'simpleLidar': {'enabled': False,
                'mode': 'raycast', 'acceleration': 'auto'}})
            self.assertFalse(value['enable_simple_lidar'])
            probe.assert_not_called()


if __name__ == '__main__':
    unittest.main()
