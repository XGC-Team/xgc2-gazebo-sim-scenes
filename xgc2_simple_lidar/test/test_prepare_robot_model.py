"""Native artifact selection and source ownership without ROS I/O."""
import copy
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

from test_launch_robot import ROBOT

script = Path(__file__).resolve().parents[1] / 'scripts/prepare_robot_model.py'
spec = importlib.util.spec_from_file_location('prepare_robot_model', script)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class PrepareRobotTest(unittest.TestCase):
    def test_saved_cloud_disables_native_ray_publisher_before_rendering(self):
        calls = []
        native = types.SimpleNamespace(
            resolve_base_sdf=lambda value: '/installed/iris.sdf',
            prepare_frozen_model=lambda row, path:
                calls.append((row, path)) or {'format': 'sdf', 'content': '<sdf/>'})
        robot = copy.deepcopy(ROBOT)
        robot['kind'] = 'px4_multirotor'
        robot['binding']['simulationSensors'] = {'simpleLidar': {
            'enabled': True, 'mode': 'raycast', 'acceleration': 'gpu'}}
        with patch.dict(sys.modules, {'render_fs150_indoor_sdf': native}):
            self.assertEqual(module.prepare(robot, 'simulation', point_cloud=True)['format'], 'sdf')
        self.assertEqual(calls[0][0]['binding']['simulationSensors']['simpleLidar'], {'enabled': False})
        self.assertTrue(robot['binding']['simulationSensors']['simpleLidar']['enabled'])

    def test_native_models_keep_slot_identity_namespace_and_sensors(self):
        calls = []
        xacro = types.SimpleNamespace(process_file=lambda path, mappings:
            calls.append((path, mappings)) or types.SimpleNamespace(toxml=lambda: '<model/>'))
        with patch.dict(sys.modules, {'xacro': xacro}):
            for kind, path, fmt in [('scout_mini', 'gazebo_sim_scout/urdf/mini.xacro', 'urdf'),
                                    ('mecanum_ugv', 'gazebo_sim_mecanum/models/xgc2_mecanum_ugv/model.sdf.xacro', 'sdf')]:
                robot = copy.deepcopy(ROBOT)
                robot['kind'] = kind
                result = module.prepare(robot, 'simulation', Path('/installed'))
                self.assertEqual(result, {'format': fmt, 'content': '<model/>'})
                file, arguments = calls[-1]
                self.assertEqual(file, '/installed/' + path)
                self.assertEqual(arguments['chassis_robot_id'], ROBOT['slotId'])
                self.assertEqual(arguments['model_name'], ROBOT['slotId'])
                self.assertEqual(arguments['ns'], 'ugv7')
                self.assertEqual(arguments['enable_simple_lidar'], 'false')
                self.assertEqual(robot['binding']['initialPose'], ROBOT['binding']['initialPose'])

    def test_physical_member_and_unsupported_kind_do_not_render(self):
        for mode, source in [('physical', 'simulation'), ('hybrid', 'physical')]:
            robot = copy.deepcopy(ROBOT)
            robot['binding']['hybridSource'] = source
            with self.assertRaisesRegex(ValueError, 'only a simulated robot'):
                module.prepare(robot, mode)
        robot = copy.deepcopy(ROBOT)
        robot['kind'] = 'unknown'
        with patch.dict(sys.modules, {'xacro': types.SimpleNamespace()}):
            with self.assertRaisesRegex(ValueError, 'does not support'):
                module.prepare(robot, 'simulation')


if __name__ == '__main__':
    unittest.main()
