"""Simulation-v1 completion and error handling without ROS or Gazebo."""
from collections import deque
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from xgc2_xrpc.http import Fault, TransportError
from xgc2_xrpc.reference import ServiceRef
from xgc2_gazebo_scene.simulation_client import SimulationClient, SimulationError


REFERENCE = {'target_id': 'fixture', 'service': 'xgc2.simulation', 'api_version': 'v1',
             'profile': 'http.v1', 'instance_id': 'world-one',
             'endpoint': {'kind': 'unix', 'address': '/private/world.sock'}}


class FakeClient:
    def __init__(self, replies):
        self.replies = deque(replies)
        self.calls = []
        self.closed = False

    def json(self, route, value=None, **kwargs):
        self.calls.append((route, value, kwargs))
        result = self.replies.popleft()
        if isinstance(result, Exception):
            raise result
        return result

    def close(self):
        self.closed = True


class SimulationClientTest(unittest.TestCase):
    def client(self, replies):
        transport = FakeClient(replies)
        runtime = object()
        with patch('xgc2_gazebo_scene.simulation_client.Client.from_service',
                   return_value=transport) as factory:
            client = SimulationClient(REFERENCE, runtime=runtime, local_target='fixture', timeout=5)
        reference, = factory.call_args.args
        self.assertEqual(reference, ServiceRef.from_dict(REFERENCE))
        self.assertIs(factory.call_args.kwargs['runtime'], runtime)
        self.assertEqual(factory.call_args.kwargs['local_target'], 'fixture')
        limits = factory.call_args.kwargs['limits']
        self.assertEqual((limits.body_bytes, limits.response_bytes, limits.call_timeout),
                         (8*1024*1024, 8*1024*1024, 5))
        self.assertIs(client.client, transport)
        return client, transport

    def test_success_preserves_request_and_returns_native_result(self):
        client, transport = self.client([
            {'id': 'delete-one', 'state': 'succeeded', 'result': {'deleted': True}}])
        value = {'generation': 7}
        self.assertEqual(client.operation('/v1/entities/robot', value, method='DELETE',
                                          request_id='delete-one'), {'deleted': True})
        self.assertEqual(value, {'generation': 7})
        self.assertEqual(len(transport.calls), 1)
        route, payload, metadata = transport.calls[0]
        self.assertEqual(route, '/v1/entities/robot')
        self.assertEqual(payload, {'generation': 7, 'operation_timeout_ms': 5000})
        self.assertEqual((metadata['method'], metadata['request_id']), ('DELETE', 'delete-one'))
        self.assertGreater(metadata['timeout'], 0)
        self.assertLessEqual(metadata['timeout'], 5)
        client.close()
        self.assertTrue(transport.closed)

    def test_accepted_or_running_waits_once_with_remaining_budget(self):
        for state in ('accepted', 'running'):
            with self.subTest(state=state):
                client, transport = self.client([
                    {'id': 'create-one', 'state': state},
                    {'id': 'create-one', 'state': 'succeeded', 'result': {'generation': 9}}])
                with patch('xgc2_gazebo_scene.simulation_client.time.monotonic',
                           side_effect=[100, 101, 103]):
                    self.assertEqual(client.operation('/v1/entities', {}, request_id='create-one'),
                                     {'generation': 9})
                self.assertEqual([call[0] for call in transport.calls],
                                 ['/v1/entities', '/v1/operations/create-one/wait'])
                self.assertEqual([call[2]['timeout'] for call in transport.calls], [4, 2])
                self.assertEqual(transport.calls[1][1], {})
                self.assertNotIn('request_id', transport.calls[1][2])

    def test_mismatched_submission_identity_stops_before_wait(self):
        client, transport = self.client([{'id': 'other', 'state': 'accepted'}])
        with self.assertRaisesRegex(SimulationError, 'different operation identity'):
            client.operation('/v1/entities', {}, request_id='create-one')
        self.assertEqual(len(transport.calls), 1)

    def test_wrong_wait_identity_or_nonterminal_state_is_rejected(self):
        for terminal in ({'id': 'other', 'state': 'succeeded'},
                         {'id': 'create-one', 'state': 'running'}):
            with self.subTest(terminal=terminal):
                client, transport = self.client([{'id': 'create-one', 'state': 'accepted'}, terminal])
                with self.assertRaisesRegex(SimulationError, 'create-one did not complete'):
                    client.operation('/v1/entities', {}, request_id='create-one')
                self.assertEqual(len(transport.calls), 2)

    def test_failed_operation_reports_native_error_without_replay(self):
        client, transport = self.client([
            {'id': 'create-one', 'state': 'failed', 'error': {'code': 'conflict'}}])
        with self.assertRaisesRegex(SimulationError, 'conflict'):
            client.operation('/v1/entities', {}, request_id='create-one')
        self.assertEqual(len(transport.calls), 1)

    def test_transport_and_fault_preserve_cause_and_diagnostic_without_replay(self):
        for error in (TransportError('connection lost', 'outcome_unknown'),
                      Fault('unavailable', 'world stopped')):
            for accepted in (False, True):
                with self.subTest(error=type(error).__name__, accepted=accepted):
                    replies = [{'id': 'create-one', 'state': 'accepted'}] if accepted else []
                    client, transport = self.client(replies + [error])
                    with self.assertRaises(SimulationError) as raised:
                        client.operation('/v1/entities', {}, request_id='create-one')
                    self.assertIs(raised.exception.__cause__, error)
                    self.assertIn('create-one', str(raised.exception))
                    self.assertIn(str(error), str(raised.exception))
                    self.assertIn(error.disposition if isinstance(error, TransportError) else error.code,
                                  str(raised.exception))
                    self.assertEqual(len(transport.calls), 2 if accepted else 1)

    def test_apply_fences_revision_and_motion_keeps_native_route(self):
        result = {'epoch': 'epoch-one', 'revision': 7}
        client, transport = self.client([
            {'id': 'apply-one', 'state': 'succeeded', 'result': result},
            {'id': 'motion-one', 'state': 'succeeded'}])
        with patch('xgc2_gazebo_scene.simulation_client.uuid.uuid4',
                   side_effect=[SimpleNamespace(hex='apply-one'), SimpleNamespace(hex='motion-one')]):
            self.assertEqual(client.apply({'obstacles': []}, 'epoch-one', 7), result)
            self.assertEqual(client.motion('pause', 'epoch-one', 7), {})
        self.assertEqual([call[0] for call in transport.calls],
                         ['/v1/extensions/scene/apply', '/v1/extensions/scene/motion'])
        self.assertEqual(transport.calls[1][1],
                         {'operation': 'pause', 'epoch': 'epoch-one', 'revision': 7,
                          'operation_timeout_ms': 5000})
        for committed in ({'epoch': 'old', 'revision': 7}, {'epoch': 'epoch-one', 'revision': 6}):
            with self.subTest(committed=committed):
                client, _ = self.client([
                    {'id': 'apply-one', 'state': 'succeeded', 'result': committed}])
                with patch('xgc2_gazebo_scene.simulation_client.uuid.uuid4',
                           return_value=SimpleNamespace(hex='apply-one')):
                    with self.assertRaisesRegex(SimulationError, 'requested scene revision'):
                        client.apply({'obstacles': []}, 'epoch-one', 7)

    def test_reference_requires_simulation_version_and_bound_identity(self):
        for field, value in (('service', 'xgc2.scene.authoring'), ('api_version', 'other'),
                             ('instance_id', '')):
            with self.subTest(field=field), patch(
                    'xgc2_gazebo_scene.simulation_client.Client.from_service') as factory:
                with self.assertRaises(ValueError):
                    SimulationClient(dict(REFERENCE, **{field: value}), runtime=object(),
                                     local_target='fixture')
                factory.assert_not_called()


if __name__ == '__main__':
    unittest.main()
