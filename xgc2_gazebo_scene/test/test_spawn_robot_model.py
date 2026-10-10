#!/usr/bin/env python3
"""The launch helper uses the real XRPC SDK; ROS supplies asset bytes only."""
import importlib.machinery
import importlib.util
import io
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from contextlib import redirect_stdout, redirect_stderr
from unittest.mock import patch

from xgc2_xrpc.http import Host
from xgc2_xrpc.runtime import Runtime
from xgc2_simple_lidar.configuration import robot_parameters

HELPER = Path(__file__).resolve().parents[1]/'scripts/spawn_robot_model'
loader = importlib.machinery.SourceFileLoader('spawn_robot_model', str(HELPER))
spec = importlib.util.spec_from_loader(loader.name, loader)
helper = importlib.util.module_from_spec(spec)
loader.exec_module(helper)
loader = importlib.machinery.SourceFileLoader('delete_robot_model', str(HELPER.with_name('delete_robot_model')))
spec = importlib.util.spec_from_loader(loader.name, loader)
delete_helper = importlib.util.module_from_spec(spec)
loader.exec_module(delete_helper)


class SpawnTest(unittest.TestCase):
    def test_native_robot_input_owns_names_pose_and_sensor_switches(self):
        robot = {'kind': 'scout_mini', 'namespace': '/ugv7', 'runMode': 'simulation',
                 'initialPose': {'x': 1, 'y': 2, 'z': .181, 'yaw': .3},
                 'authoredSimulationSensors': {'simpleLidar': False},
                 'scout': {'lidarSimulationEnabled': True, 'imageSimulationEnabled': False}}
        parameters = robot_parameters(robot, 'scout', '/ugv7')
        self.assertEqual(parameters['ns'], 'ugv7')
        self.assertEqual(parameters['model_name'], 'ugv7')
        self.assertEqual(parameters['robot_description_param'], '/ugv7/robot_description')
        self.assertEqual(parameters['frame_prefix'], 'ugv7/')
        self.assertEqual(parameters['z'], .181)
        self.assertTrue(parameters['enable_lidar'])
        self.assertFalse(parameters['enable_camera'])
        with self.assertRaises(ValueError):
            robot_parameters(robot, 'scout', '/ugv8')
        robot['initialPose']['x'] = float('nan')
        with self.assertRaises(ValueError):
            robot_parameters(robot, 'scout', '/ugv7')

    def test_stop_uses_observed_generation_and_native_terminal_completion(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Runtime(blocking_workers=1)
            path = str(Path(directory)/'world.sock')
            calls = []
            def read(context, value):
                calls.append('get')
                return {'entities': [{'ref': {'id': 'ugv7', 'generation': 73}}]}
            def delete(context, value):
                calls.append(('delete', value['generation']))
                return {'id': context.request_id, 'state': 'accepted'}
            def wait(context, value):
                calls.append('terminal')
                return {'id': 'stop-one', 'state': 'succeeded', 'result': {}}
            host = Host(path, {('GET', '/v1/entities/ugv7'): read,
                               ('DELETE', '/v1/entities/ugv7'): delete,
                               ('POST', '/v1/operations/stop-one/wait'): wait},
                        runtime=runtime, instance_id='world1')
            host.start()
            reference = {'target_id': 'fixture', 'service': 'xgc2.simulation', 'api_version': 'v1',
                         'profile': 'http.v1', 'instance_id': 'world1',
                         'endpoint': {'kind': 'unix', 'address': path}}
            try:
                argv = ['delete_robot_model', '--namespace', '/ugv7', '--target-id', 'fixture']
                launch_input = {'robot': {'kind': 'scout_mini', 'namespace': '/ugv7',
                                         'runMode': 'simulation',
                                         'initialPose': {'x': 0, 'y': 0, 'z': .181, 'yaw': 0},
                                         'authoredSimulationSensors': {'simpleLidar': False},
                                         'scout': {'lidarSimulationEnabled': True,
                                                   'imageSimulationEnabled': False}},
                                'simulationServiceRef': reference}
                stdin = types.SimpleNamespace(buffer=io.BytesIO(json.dumps(launch_input).encode()))
                with patch.object(sys, 'argv', argv), patch.object(sys, 'stdin', stdin), patch(
                        'xgc2_scene_runtime.simulation_client.uuid.uuid4',
                        return_value=types.SimpleNamespace(hex='stop-one')):
                    delete_helper.main()
                self.assertEqual(calls, ['get', ('delete', 73), 'terminal'])
            finally:
                host.close(); runtime.close()

    def test_one_native_creation_and_no_ros_control(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Runtime(blocking_workers=1)
            path = str(Path(directory)/'world.sock')
            requests = []
            def create(context, value):
                requests.append(value)
                return {'id': context.request_id, 'state': 'succeeded',
                        'result': {'entity': {'id': value['entity']['id'], 'generation': 1}}}
            host = Host(path, {('POST', '/v1/entities'): create}, runtime=runtime, instance_id='world1')
            host.start()
            reference = {'target_id': 'fixture', 'service': 'xgc2.simulation', 'api_version': 'v1',
                         'profile': 'http.v1', 'instance_id': 'world1',
                         'endpoint': {'kind': 'unix', 'address': path}}
            ros = types.SimpleNamespace(init_node=lambda *a, **k: None,
                get_param=lambda name: '<robot name="robot1"><link name="body"/></robot>',
                get_namespace=lambda: '/robot1/', signal_shutdown=lambda reason: None)
            output = io.StringIO()
            try:
                with patch.dict(sys.modules, {'rospy': ros}), redirect_stdout(output):
                    helper.main(['spawn_robot_model', '-urdf', '-param', 'robot_description',
                                 '-model', 'robot1', '-simulation_service_ref_json', json.dumps(reference),
                                 '-target_id', 'fixture', '-x', '1', '-Y', '0.5'])
                self.assertEqual(len(requests), 1)
                entity = requests[0]['entity']
                self.assertEqual(entity['asset']['realization']['media_type'], 'application/urdf+xml')
                self.assertEqual(entity['parameters'], {'ros_namespace': '/robot1/'})
                self.assertEqual(entity['pose']['position'], [1, 0, 0])
                self.assertIn('"generation": 1', output.getvalue())
            finally:
                host.close(); runtime.close()

    def test_missing_explicit_reference_and_bad_pose_are_rejected(self):
        with redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            helper.parse_arguments(['spawn_robot_model', '-sdf', '-param', 'description', '-model', 'one'])
        args = types.SimpleNamespace(x=float('nan'), y=0, z=0, R=0, P=0, Y=0, model='one', urdf=False)
        with self.assertRaises(ValueError):
            helper.entity(args, '<sdf/>', '/one/')

    def test_no_implicit_model_reuse_or_retry_after_provider_failure(self):
        with tempfile.TemporaryDirectory() as directory:
            runtime = Runtime(blocking_workers=1)
            count = []
            def create(context, value):
                count.append(value)
                return {'id': context.request_id, 'state': 'failed', 'error': {'code': 'conflict'}}
            path = str(Path(directory)/'world.sock')
            host = Host(path, {('POST', '/v1/entities'): create}, runtime=runtime, instance_id='world1')
            host.start()
            reference = {'target_id': 'fixture', 'service': 'xgc2.simulation', 'api_version': 'v1',
                         'profile': 'http.v1', 'instance_id': 'world1', 'endpoint': {'kind': 'unix', 'address': path}}
            ros = types.SimpleNamespace(init_node=lambda *a, **k: None, get_param=lambda name: '<sdf/>',
                                        get_namespace=lambda: '/', signal_shutdown=lambda reason: None)
            try:
                with patch.dict(sys.modules, {'rospy': ros}), redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as raised:
                    helper.main(['spawn_robot_model', '-sdf', '-param', 'description', '-model', 'one',
                                 '-simulation_service_ref_json', json.dumps(reference), '-target_id', 'fixture'])
                self.assertEqual(raised.exception.code, 1)
                self.assertEqual(len(count), 1)
            finally:
                host.close(); runtime.close()


if __name__ == '__main__':
    unittest.main()
