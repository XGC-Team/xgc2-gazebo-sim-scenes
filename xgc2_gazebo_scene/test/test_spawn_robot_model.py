#!/usr/bin/env python3
"""Finite placement CLI behavior without ROS, a master or Gazebo."""
import importlib.machinery
import importlib.util
import builtins
import io
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import types
import unittest
from contextlib import redirect_stdout, redirect_stderr
from unittest.mock import patch

from test_placement import FakeWorld

HELPER = Path(__file__).resolve().parents[1] / 'scripts/spawn_robot_model'
loader = importlib.machinery.SourceFileLoader('spawn_robot_model', str(HELPER))
spec = importlib.util.spec_from_loader(loader.name, loader)
helper = importlib.util.module_from_spec(spec)
loader.exec_module(helper)
MODEL = b'<robot name="robot1"><link name="body"/></robot>'


class SpawnTest(unittest.TestCase):
    def environment(self, *, modules=True):
        environment = {'PATH': '/usr/bin:/bin', 'PYTHONDONTWRITEBYTECODE': '1'}
        if modules:
            environment['PYTHONPATH'] = os.pathsep.join(sys.path)
        return environment

    def arguments(self, endpoint, *, file=None, timeout=2):
        args = ['--world-endpoint', endpoint, '--target-id', 'fixture', '--model-name', 'robot1',
                '--model-format', 'urdf', '--timeout', str(timeout)]
        return args + (['--model-file', str(file)] if file is not None else ['--model-stdin'])

    def run_cli(self, world, *, model=MODEL, file=False, extra=(), timeout=2):
        path = None
        if file:
            path = Path(world.directory.name) / 'model.xml'
            path.write_bytes(model)
        return subprocess.run([str(HELPER), *self.arguments(world.endpoint, file=path, timeout=timeout), *extra],
                              input=None if file else model, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              env=self.environment(), timeout=5)

    def assert_failure(self, completed, code, reason=None):
        self.assertEqual(completed.returncode, code, completed.stderr.decode())
        self.assertEqual(completed.stdout, b'')
        lines = completed.stderr.decode().splitlines()
        self.assertEqual(len(lines), 1, lines)
        self.assertTrue(lines[0].startswith('spawn_robot_model: '))
        self.assertNotIn('Traceback', lines[0])
        if reason is not None:
            self.assertIn(reason, lines[0])

    def test_stdin_and_file_preserve_entity_and_stdout_schema(self):
        for file, model_format in ((False, 'urdf'), (True, 'sdf')):
            with self.subTest(file=file), FakeWorld(accepted=True) as world:
                completed = self.run_cli(world, file=file, extra=(
                    '--model-format', model_format, '--robot-namespace', '/robot1',
                    '--x', '1', '--y', '2', '--z', '.181', '--yaw', '.5'))
                self.assertEqual(completed.returncode, 0, completed.stderr.decode())
                self.assertEqual(completed.stderr, b'')
                self.assertEqual(len(completed.stdout.decode().splitlines()), 1)
                output = json.loads(completed.stdout)
                self.assertEqual(output, {'schema': 'xgc2.placement.v1', 'kind': 'robot', 'id': 'robot1',
                                          'entity': {'entity': {'id': 'robot1', 'generation': 7}},
                                          'instance': 'world-one'})
                self.assertEqual([call[0] for call in world.calls], ['describe', 'create', 'wait'])
                entity = world.calls[1][1]['entity']
                self.assertEqual(entity['id'], 'robot1')
                self.assertEqual(entity['role'], 'robot')
                self.assertEqual(entity['asset'], {'id': 'robot1', 'realization': {
                    'media_type': 'application/' + model_format + '+xml', 'content': MODEL.decode()}})
                self.assertEqual(entity['parameters'], {'ros_namespace': '/robot1'})
                self.assertEqual(entity['pose']['position'], [1, 2, .181])
                for observed, expected in zip(entity['pose']['orientation'],
                                               [0, 0, math.sin(.25), math.cos(.25)]):
                    self.assertAlmostEqual(observed, expected)

    def test_rpy_quaternion_and_defaults(self):
        args = helper.parse_arguments(self.arguments('/tmp/world.sock'))
        self.assertEqual((args.robot_namespace, args.x, args.y, args.z, args.roll, args.pitch, args.yaw),
                         ('/', 0, 0, 0, 0, 0, 0))
        args.roll, args.yaw = math.pi, math.pi / 2
        value = helper.entity(args, MODEL.decode(), args.robot_namespace)
        for observed, expected in zip(value['pose']['orientation'], [math.sqrt(.5), math.sqrt(.5), 0, 0]):
            self.assertAlmostEqual(observed, expected)
        defaults = helper.parse_arguments(self.arguments('/tmp/world.sock')[:-3] + ['--model-stdin'])
        self.assertEqual(defaults.timeout, 50)

    def test_validation_errors_contact_no_world(self):
        cases = [('pose', ('--x', 'nan')), ('timeout', ('--timeout', '0')),
                 ('timeout', ('--timeout', '-1')), ('timeout', ('--timeout', '55.01')),
                 ('timeout', ('--timeout', 'nan')), ('timeout', ('--timeout', 'inf')),
                 ('endpoint', ('--world-endpoint', 'relative.sock')),
                 ('endpoint', ('--world-endpoint', '/tmp/../world.sock')),
                 ('target', ('--target-id', '')),
                 ('model name', ('--model-name', 'robot/one')),
                 ('namespace', ('--robot-namespace', 'relative'))]
        with FakeWorld() as world:
            for reason, extra in cases:
                with self.subTest(extra=extra):
                    self.assert_failure(self.run_cli(world, extra=extra), 2)
            self.assertEqual(world.calls, [])

    def test_malformed_empty_and_oversized_models_before_contact(self):
        with FakeWorld() as world:
            for file in (False, True):
                for model in (b'<robot>', b'', b'\xff', b' ' * (helper.MAX_MODEL_BYTES + 1)):
                    with self.subTest(file=file, size=len(model)):
                        self.assert_failure(self.run_cli(world, file=file, model=model), 2)
            missing = subprocess.run([str(HELPER), *self.arguments(world.endpoint, file='/missing/model.xml')],
                                     capture_output=True, env=self.environment(), timeout=2)
            self.assert_failure(missing, 2)
            self.assertEqual(world.calls, [])

    def test_exact_model_limit_is_accepted_and_imports_placement_lazily(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'model.xml'
            prefix, suffix = b'<sdf><!--', b'--></sdf>'
            model = prefix + b' ' * (helper.MAX_MODEL_BYTES - len(prefix) - len(suffix)) + suffix
            path.write_bytes(model)
            calls = []
            def place(endpoint, target, entity, timeout):
                calls.append((entity, timeout))
                return {'id': 'robot1', 'generation': 7}, 'world-one'
            module = types.SimpleNamespace(place_entity=place)
            output, errors = io.StringIO(), io.StringIO()
            with patch.dict(sys.modules, {'xgc2_gazebo_scene.placement': module, 'rospy': None}), \
                    patch.object(sys, 'path', []), redirect_stdout(output), redirect_stderr(errors):
                self.assertEqual(helper.main(self.arguments('/tmp/world.sock', file=path)), 0)
                self.assertIn('/opt/ros/noetic/lib/python3/dist-packages', sys.path)
            self.assertEqual(errors.getvalue(), '')
            self.assertEqual(len(calls), 1)
            self.assertEqual(len(calls[0][0]['asset']['realization']['content'].encode()), helper.MAX_MODEL_BYTES)
            self.assertGreater(calls[0][1], 0)
            self.assertLess(calls[0][1], 2)
            self.assertEqual(json.loads(output.getvalue())['entity'], {'id': 'robot1', 'generation': 7})

    def test_usage_errors_have_exit_two_and_one_line(self):
        arguments = self.arguments('/tmp/world.sock')
        cases = [[], arguments + ['--unknown'], arguments + ['--world', '/tmp/world.sock'],
                 arguments + ['--model-file', '/tmp/model.xml'], arguments + ['--model-format', 'other'],
                 arguments + ['--hold'], arguments + ['-urdf'], arguments + ['-param', 'robot_description'],
                 arguments + ['-simulation_service_ref_json', '{}']]
        for args in cases:
            with self.subTest(args=args):
                completed = subprocess.run([str(HELPER), *args], capture_output=True,
                                           env=self.environment(modules=False), timeout=2)
                self.assert_failure(completed, 2)

    def test_lazy_import_consumes_the_shared_budget(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'model.xml'
            path.write_bytes(MODEL)
            for import_seconds in (1, 3):
                with self.subTest(import_seconds=import_seconds):
                    clock = types.SimpleNamespace(now=100)
                    calls = []
                    def place(endpoint, target, entity, timeout):
                        calls.append(timeout)
                        return {'id': 'robot1'}, 'world-one'
                    module = types.SimpleNamespace(place_entity=place)
                    original_import = builtins.__import__
                    def importing(name, *args, **kwargs):
                        if name == 'xgc2_gazebo_scene.placement':
                            clock.now += import_seconds
                        return original_import(name, *args, **kwargs)
                    output, errors = io.StringIO(), io.StringIO()
                    with patch.dict(sys.modules, {'xgc2_gazebo_scene.placement': module}), patch.object(
                            helper, 'time', types.SimpleNamespace(monotonic=lambda: clock.now)), patch(
                            'builtins.__import__', importing), redirect_stdout(output), redirect_stderr(errors):
                        code = helper.main(self.arguments('/tmp/world.sock', file=path))
                    if import_seconds == 1:
                        self.assertEqual(code, 0)
                        self.assertEqual(calls, [1])
                        self.assertEqual(errors.getvalue(), '')
                    else:
                        self.assertEqual(code, 1)
                        self.assertEqual(calls, [])
                        self.assertEqual(output.getvalue(), '')
                        self.assertEqual(len(errors.getvalue().splitlines()), 1)
                        self.assertIn('deadline', errors.getvalue())

    def test_help_without_world_home_ros_or_placement_import(self):
        completed = subprocess.run([str(HELPER), '--help'], capture_output=True,
                                   env=self.environment(modules=False), timeout=2)
        self.assertEqual(completed.returncode, 0)
        self.assertEqual(completed.stderr, b'')
        self.assertIn(b'--model-stdin', completed.stdout)
        self.assertIn(b'--world-endpoint', completed.stdout)

    def test_world_validation_failures_are_exit_one_and_send_no_creation(self):
        for field, value in (('target_id', 'remote'), ('service', 'xgc2.other'), ('api_version', 'other'),
                             ('profile', 'grpc.v1')):
            with self.subTest(field=field), FakeWorld() as world:
                world.reference[field] = value
                self.assert_failure(self.run_cli(world), 1)
                self.assertEqual([call[0] for call in world.calls], ['describe'])
        with FakeWorld() as world:
            world.reference['endpoint'] = {'kind': 'https', 'address': 'https://example.com'}
            self.assert_failure(self.run_cli(world), 1)
            self.assertEqual([call[0] for call in world.calls], ['describe'])

    def test_conflict_names_entity_and_posts_exactly_once(self):
        with FakeWorld(conflict=True) as world:
            self.assert_failure(self.run_cli(world), 1, 'conflict')
            self.assertEqual([call[0] for call in world.calls], ['describe', 'create'])
            completed = self.run_cli(world)
            self.assert_failure(completed, 1, 'robot1')
            self.assertEqual(sum(call[0] == 'create' for call in world.calls), 2)

    def test_not_ready_and_missing_socket_finish_at_deadline(self):
        for start in (True, False):
            with self.subTest(start=start), FakeWorld(start=start, ready=False) as world:
                self.assert_failure(self.run_cli(world, timeout=.4), 1)
                self.assertEqual([call[0] for call in world.calls], ['describe'] if start else [])

    def test_uncompleted_creation_uses_budget_left_by_discovery(self):
        with FakeWorld(ready=False, ready_after=.15, accepted=True, incomplete=True) as world:
            started = time.monotonic()
            self.assert_failure(self.run_cli(world, timeout=.5), 1)
            self.assertLess(time.monotonic() - started, 1)
            self.assertEqual([call[0] for call in world.calls], ['describe', 'create', 'wait'])
            self.assertLess(world.calls[1][1]['operation_timeout_ms'], 350)
            self.assertLess(world.calls[2][2], .35)

    def test_unfinished_stdin_is_bounded_without_contact(self):
        with FakeWorld() as world:
            process = subprocess.Popen([str(HELPER), *self.arguments(world.endpoint, timeout=.15)],
                                       stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                       env=self.environment())
            try:
                process.wait(timeout=2)
                stdout, stderr = process.communicate(timeout=1)
                self.assert_failure(subprocess.CompletedProcess(process.args, process.returncode, stdout, stderr),
                                    2, 'stdin')
                self.assertEqual(world.calls, [])
            finally:
                if process.poll() is None:
                    process.kill()
                process.communicate()


if __name__ == '__main__':
    unittest.main()
