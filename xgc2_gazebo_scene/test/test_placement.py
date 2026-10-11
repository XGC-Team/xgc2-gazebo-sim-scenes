"""Bounded discovery and placement with local SDK hosts and finite fakes."""
import asyncio
import tempfile
import threading
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from xgc2_xrpc import Host, Limits, Runtime
from xgc2_gazebo_scene.placement import discover_world, place_entity
from xgc2_gazebo_scene.simulation_client import SimulationError


class FakeWorld:
    def __init__(self, *, start=True, ready=True, ready_after=0, accepted=False,
                 incomplete=False, conflict=False):
        self.directory = tempfile.TemporaryDirectory()
        self.endpoint = str(Path(self.directory.name) / 'world.sock')
        self.reference = {'target_id': 'fixture', 'service': 'xgc2.simulation', 'api_version': 'v1',
                          'profile': 'http.v1', 'instance_id': 'world-one',
                          'endpoint': {'kind': 'unix', 'address': self.endpoint}}
        self.description = dict(service='xgc2.simulation', api_version='v1',
                                instance_id='world-one', facts={'service_ref': self.reference}, ready=ready)
        self.ready_after, self.accepted = ready_after, accepted
        self.incomplete, self.conflict = incomplete, conflict
        self.calls, self.headers = [], []
        self.runtime = Runtime(blocking_workers=1, log_level='error')
        self.host = Host(self.endpoint, {('GET', '/v1/describe'): self.describe,
                                       ('POST', '/v1/entities'): self.create},
                         runtime=self.runtime, instance_id='world-one',
                         discovery_routes=('/v1/describe',),
                         limits=Limits(body_bytes=20*1024*1024))
        original_headers = self.host._headers
        def headers(request):
            self.headers.append((request.method, request.path, request.query_string,
                                 request.headers.get('X-Xrpc-Instance-ID')))
            return original_headers(request)
        self.host._headers = headers
        self.start_immediately = start
        self.timer = None

    async def describe(self, context, value):
        self.calls.append(('describe', context.query))
        held = int(context.query.partition('=')[2]) / 1000
        if self.ready_after:
            await asyncio.sleep(min(self.ready_after, held))
            self.description['ready'] = self.ready_after < held
        elif not self.description['ready']:
            await asyncio.sleep(held)
        return self.description

    async def create(self, context, value):
        self.calls.append(('create', value, context.remaining()))
        self.operation_id = context.request_id
        if self.conflict:
            return {'id': context.request_id, 'state': 'failed',
                    'error': {'code': 'conflict'}}
        self.result = {'entity': {'id': value['entity']['id'], 'generation': 7}}
        if self.accepted:
            self.host.routes[('POST', '/v1/operations/' + context.request_id + '/wait')] = self.wait
            return {'id': context.request_id, 'state': 'accepted'}
        return {'id': context.request_id, 'state': 'succeeded', 'result': self.result}

    async def wait(self, context, value):
        self.calls.append(('wait', value, context.remaining()))
        if self.incomplete:
            await asyncio.sleep(context.remaining() * 2)
        return {'id': self.operation_id, 'state': 'succeeded', 'result': self.result}

    def appear_after(self, delay):
        self.timer = threading.Timer(delay, self.host.start)
        self.timer.start()

    def __enter__(self):
        if self.start_immediately:
            self.host.start()
        return self

    def __exit__(self, *_):
        if self.timer is not None:
            self.timer.cancel()
            self.timer.join()
        try:
            self.host.close()
        finally:
            self.runtime.close()
            self.directory.cleanup()


class PlacementTest(unittest.TestCase):
    def test_socket_appears_within_budget_then_one_unbound_describe(self):
        with FakeWorld(start=False) as world:
            self.assertFalse(Path(world.endpoint).exists())
            world.appear_after(.04)
            client = discover_world(world.endpoint, 'fixture', 2)
            owned_runtime = client._owned_runtime
            try:
                self.assertEqual(client.reference.instance_id, 'world-one')
                self.assertEqual(len(world.calls), 1)
                self.assertIsNone(world.headers[0][3])
                self.assertTrue(world.headers[0][2].startswith('wait_ready_ms='))
                self.assertLessEqual(int(world.headers[0][2].split('=')[1]), 2000)
            finally:
                client.close()
            self.assertTrue(owned_runtime.closed)

    def test_world_becomes_ready_in_the_same_held_call_and_instance_is_pinned(self):
        with FakeWorld(ready=False, ready_after=.04, accepted=True) as world:
            result, instance = place_entity(world.endpoint, 'fixture', {'id': 'robot'}, 2)
            self.assertEqual((result, instance), ({'entity': {'id': 'robot', 'generation': 7}}, 'world-one'))
            self.assertEqual([call[0] for call in world.calls], ['describe', 'create', 'wait'])
            self.assertIsNone(world.headers[0][3])
            self.assertEqual([header[3] for header in world.headers[1:]], ['world-one', 'world-one'])

    def test_wrong_world_identity_never_submits(self):
        for field, value, reason in (
                ('service', 'xgc2.other', 'service'), ('api_version', 'other', 'API version'),
                ('profile', 'grpc.v1', 'http.v1'), ('target_id', 'remote', 'target'),
                ('instance_id', 'other', 'instance IDs')):
            with self.subTest(field=field), FakeWorld() as world:
                world.reference[field] = value
                with self.assertRaisesRegex(SimulationError, reason):
                    place_entity(world.endpoint, 'fixture', {'id': 'robot'}, 1)
                self.assertEqual([call[0] for call in world.calls], ['describe'])
        with FakeWorld() as world:
            world.reference['endpoint'] = {'kind': 'https', 'address': 'https://example.com'}
            with self.assertRaisesRegex(SimulationError, 'Unix endpoint'):
                place_entity(world.endpoint, 'fixture', {'id': 'robot'}, 1)
            self.assertEqual(len(world.calls), 1)
        with FakeWorld() as world:
            world.reference['endpoint']['address'] = '/tmp/other-world.sock'
            with self.assertRaisesRegex(SimulationError, 'allocated endpoint'):
                place_entity(world.endpoint, 'fixture', {'id': 'robot'}, 1)
            self.assertEqual(len(world.calls), 1)

    def test_not_ready_at_deadline_never_rearms_discovery(self):
        with FakeWorld(ready=False) as world:
            with self.assertRaisesRegex(Exception, 'deadline|ready'):
                place_entity(world.endpoint, 'fixture', {'id': 'robot'}, .15)
            self.assertEqual([call[0] for call in world.calls], ['describe'])

    def test_conflict_and_uncompleted_operation_never_replay(self):
        for options in ({'conflict': True}, {'accepted': True, 'incomplete': True}):
            with self.subTest(options=options), FakeWorld(**options) as world:
                owners = []
                def runtime(**kwargs):
                    owner = Runtime(**kwargs)
                    owners.append(owner)
                    return owner
                with patch('xgc2_gazebo_scene.placement.Runtime', side_effect=runtime), \
                        self.assertRaisesRegex(SimulationError, 'conflict|deadline|closed'):
                    place_entity(world.endpoint, 'fixture', {'id': 'robot'}, .15)
                self.assertEqual(len(owners), 1)
                self.assertTrue(owners[0].closed)
                self.assertEqual(sum(call[0] == 'create' for call in world.calls), 1)
                self.assertEqual(sum(call[0] == 'wait' for call in world.calls), int(world.accepted))

    def test_invalid_arguments_allocate_no_runtime(self):
        for endpoint, target, timeout in (('relative.sock', 'fixture', 1),
                                          ('/tmp/../world.sock', 'fixture', 1),
                                          ('/tmp/world.sock', '', 1),
                                          ('/tmp/world.sock', 'fixture', 0),
                                          ('/tmp/world.sock', 'fixture', 56),
                                          ('/tmp/world.sock', 'fixture', float('nan'))):
            with self.subTest(endpoint=endpoint, target=target, timeout=timeout), patch(
                    'xgc2_gazebo_scene.placement.Runtime') as runtime:
                with self.assertRaises(ValueError):
                    discover_world(endpoint, target, timeout)
                runtime.assert_not_called()

    def test_shared_deadline_and_all_owners_close(self):
        clock = SimpleNamespace(now=100.0)
        time = SimpleNamespace(monotonic=lambda: clock.now)
        reference = dict(target_id='fixture', service='xgc2.simulation', api_version='v1',
                         profile='http.v1', instance_id='world-one',
                         endpoint={'kind': 'unix', 'address': '/tmp/world.sock'})
        calls = []
        discovery, bound, runtime = Mock(), Mock(), Mock()
        def socket(endpoint, deadline):
            clock.now += 1
        def describe(route, **kwargs):
            calls.append((route, kwargs['timeout']))
            clock.now += 2
            return dict(service='xgc2.simulation', api_version='v1', instance_id='world-one',
                        ready=True, facts={'service_ref': reference})
        def operation(route, payload, **kwargs):
            calls.append((route, kwargs['timeout']))
            if route == '/v1/entities':
                self.assertEqual(payload['operation_timeout_ms'], 2000)
                clock.now += 1
                return {'id': 'create-one', 'state': 'accepted'}
            clock.now += .5
            return {'id': 'create-one', 'state': 'succeeded', 'result': {'id': 'robot'}}
        discovery.json.side_effect = describe
        bound.json.side_effect = operation
        with patch('xgc2_gazebo_scene.placement.time', time), patch(
                'xgc2_gazebo_scene.simulation_client.time', time), patch(
                'xgc2_gazebo_scene.placement._wait_for_socket', socket), patch(
                'xgc2_gazebo_scene.placement.Runtime', return_value=runtime), patch(
                'xgc2_gazebo_scene.placement.Client', return_value=discovery), patch(
                'xgc2_gazebo_scene.simulation_client.Client.from_service', return_value=bound), patch(
                'xgc2_gazebo_scene.simulation_client.uuid.uuid4', return_value=SimpleNamespace(hex='create-one')):
            self.assertEqual(place_entity('/tmp/world.sock', 'fixture', {'id': 'robot'}, 5),
                             ({'id': 'robot'}, 'world-one'))
        self.assertEqual(calls, [('/v1/describe?wait_ready_ms=4000', 4),
                                 ('/v1/entities', 2), ('/v1/operations/create-one/wait', 1)])
        discovery.close.assert_called_once_with()
        bound.close.assert_called_once_with()
        runtime.close.assert_called_once_with()

    def test_held_describe_is_capped_and_failure_closes_discovery_runtime(self):
        reference = dict(target_id='fixture', service='xgc2.simulation', api_version='v1',
                         profile='http.v1', instance_id='world-one',
                         endpoint={'kind': 'unix', 'address': '/tmp/world.sock'})
        discovery, bound, runtime = Mock(), Mock(), Mock()
        discovery.json.return_value = dict(service='xgc2.simulation', api_version='v1',
                                           instance_id='world-one', ready=True, facts={'service_ref': reference})
        with patch('xgc2_gazebo_scene.placement._wait_for_socket'), patch(
                'xgc2_gazebo_scene.placement.Runtime', return_value=runtime), patch(
                'xgc2_gazebo_scene.placement.Client', return_value=discovery), patch(
                'xgc2_gazebo_scene.simulation_client.Client.from_service', return_value=bound):
            client = discover_world('/tmp/world.sock', 'fixture', 55)
            client.close()
        self.assertEqual(discovery.json.call_args.args, ('/v1/describe?wait_ready_ms=30000',))
        self.assertEqual(discovery.json.call_args.kwargs['method'], 'GET')
        self.assertLessEqual(discovery.json.call_args.kwargs['timeout'], 55)
        runtime.close.assert_called_once_with()
        discovery.reset_mock()
        runtime.reset_mock()
        discovery.json.side_effect = RuntimeError('discovery failed')
        with patch('xgc2_gazebo_scene.placement._wait_for_socket'), patch(
                'xgc2_gazebo_scene.placement.Runtime', return_value=runtime), patch(
                'xgc2_gazebo_scene.placement.Client', return_value=discovery), \
                self.assertRaisesRegex(RuntimeError, 'discovery failed'):
            discover_world('/tmp/world.sock', 'fixture', 2)
        discovery.close.assert_called_once_with()
        runtime.close.assert_called_once_with()


if __name__ == '__main__':
    unittest.main()
