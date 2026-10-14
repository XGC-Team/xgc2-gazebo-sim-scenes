"""Native artifact selection and source ownership without ROS I/O."""
import copy
import importlib.util
import io
import json
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
    def test_px4_lowering_keeps_authored_sensor_and_only_needs_run_mode(self):
        calls = []
        native = types.SimpleNamespace(
            resolve_base_sdf=lambda value: '/installed/iris.sdf',
            prepare_frozen_model=lambda row, path:
                calls.append((row, path)) or {'format': 'sdf', 'content': '<sdf/>'})
        robot = copy.deepcopy(ROBOT)
        robot['kind'] = 'px4_multirotor'
        robot['binding']['simulationSensors'] = {'simpleLidar': {
            'enabled': True, 'mode': 'penetrating', 'acceleration': 'gpu'}}
        original = copy.deepcopy(robot)
        output = io.StringIO()
        with patch.dict(sys.modules, {'render_fs150_indoor_sdf': native}), \
                patch.object(sys, 'argv', [str(script), '--run-mode', 'simulation']), \
                patch.object(sys, 'stdin', types.SimpleNamespace(buffer=io.BytesIO(json.dumps(robot).encode()))), \
                patch.object(sys, 'stdout', output), patch('subprocess.check_output') as child:
            module.main()
            child.assert_not_called()
        self.assertEqual(json.loads(output.getvalue()), {'format': 'sdf', 'content': '<sdf/>'})
        self.assertEqual(calls[0][0], original)
        self.assertEqual(robot, original)

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

    def test_ground_robot_legacy_mode_renders_native_sensor_without_mutating_row(self):
        calls = []
        xacro = types.SimpleNamespace(process_file=lambda path, mappings:
            calls.append(mappings) or types.SimpleNamespace(toxml=lambda: '<model/>'))
        with patch.dict(sys.modules, {'xacro': xacro}):
            for kind in ('scout_mini', 'mecanum_ugv'):
                for backend in ('cpu', 'gpu'):
                    robot = copy.deepcopy(ROBOT)
                    robot['kind'] = kind
                    robot['binding']['simulationSensors'] = {'simpleLidar': {
                        'enabled': True, 'mode': 'penetrating', 'preset': 'bridge_equivalent',
                        'acceleration': backend, 'rateHz': 4, 'hRes': 720, 'vRes': 32}}
                    original = copy.deepcopy(robot)
                    module.prepare(robot, 'simulation', Path('/installed'))
                    self.assertEqual(calls[-1]['enable_simple_lidar'], 'true')
                    self.assertEqual(calls[-1]['simple_lidar_acceleration'], backend)
                    self.assertEqual(calls[-1]['simple_lidar_hres'], '720')
                    self.assertEqual(calls[-1]['simple_lidar_vres'], '32')
                    self.assertEqual(calls[-1]['simple_lidar_rate_hz'], '4')
                    self.assertEqual(robot, original)

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
