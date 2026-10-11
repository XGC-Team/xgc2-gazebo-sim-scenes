#!/usr/bin/env python3
"""Shared launch parameters derived from freezer rows without ROS."""
import copy
import math
import unittest

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
                        robot_state_publisher_ns='ugv7', run_mode='hybrid',
                        enable_lidar=True, enable_camera=True)
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

    def test_omitted_scout_overrides_default_both_switches_to_false(self):
        row = copy.deepcopy(SCOUT_ROW)
        del row['binding']['scout']
        parameters = robot_parameters(row, 'scout', 'physical')
        self.assertFalse(parameters['enable_lidar'])
        self.assertFalse(parameters['enable_camera'])
        self.assertEqual(parameters['run_mode'], 'physical')

    def test_omitted_scout_switch_defaults_to_false(self):
        for source, target, other in (('lidarSimulationEnabled', 'enable_lidar', 'enable_camera'),
                                      ('imageSimulationEnabled', 'enable_camera', 'enable_lidar')):
            row = copy.deepcopy(SCOUT_ROW)
            del row['binding']['scout'][source]
            with self.subTest(source=source):
                parameters = robot_parameters(row, 'scout', 'simulation')
                self.assertFalse(parameters[target])
                self.assertTrue(parameters[other])

    def test_scout_switches_must_be_boolean(self):
        for source in ('lidarSimulationEnabled', 'imageSimulationEnabled'):
            for value in (None, 0, 1, 'true'):
                row = copy.deepcopy(SCOUT_ROW)
                row['binding']['scout'][source] = value
                with self.subTest(source=source, value=value), self.assertRaisesRegex(ValueError, 'boolean'):
                    robot_parameters(row, 'scout', 'simulation')

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


if __name__ == '__main__':
    unittest.main()
