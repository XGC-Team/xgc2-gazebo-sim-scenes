#!/usr/bin/env python3
"""Isolated adapter/Gazebo acceptance. Takes the algorithm-owned scene as input.

The collision observer independently reads actual collision shapes through the
planning geometry library and instances. No planner or control node is started.
All processes, ROS/Gazebo masters, files and save targets belong to this run.
"""

import argparse
from collections import Counter
import copy
import json
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import threading
import time
import uuid
import xmlrpc.client
import xml.etree.ElementTree as ET

import roslib.packages
import rospkg
import rospy
import yaml
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import String
from xgc2_geometry_msgs.msg import ConvexBodyArray, GeometryLibrary, SceneSnapshot, SceneState
from xgc2_xrpc.http import Client, Limits
from xgc2_xrpc.reference import ServiceRef
from xgc2_xrpc.runtime import Runtime


def wait_for(predicate, description, timeout=20):
    deadline = time.monotonic() + timeout
    last_error = None
    while time.monotonic() < deadline:
        try:
            result = predicate()
            if result:
                return result
        except (AssertionError, rospy.ROSException, OSError) as error:
            last_error = str(error)
        time.sleep(0.03)
    raise AssertionError('{} timed out: {}'.format(description, last_error))


def free_ports():
    sockets = [socket.socket(), socket.socket()]
    try:
        for sock in sockets:
            for _ in range(100):
                try:
                    sock.bind(('127.0.0.1', 20000 + uuid.uuid4().int % 20001))
                    break
                except OSError:
                    continue
            else:
                raise RuntimeError('No private master port available in 20000-40000')
        return [sock.getsockname()[1] for sock in sockets]
    finally:
        for sock in sockets:
            sock.close()


def model_name(oid):
    return 'xgc2_obstacle_scene_' + oid.encode('utf-8').hex()


def vector(value):
    return [value.x, value.y, value.z]


def close(left, right, tolerance=2e-6):
    assert len(left) == len(right)
    assert all(math.isclose(a, b, abs_tol=tolerance, rel_tol=0) for a, b in zip(left, right)), (left, right)


class Integration:
    def __init__(self, scene_file, evidence):
        self.evidence = evidence
        self.evidence.mkdir(parents=True, exist_ok=False)
        self.source = evidence / 'scene.yaml'
        shutil.copyfile(scene_file, self.source)
        self.processes = []
        self.logs = []
        self.samples = {}
        self.lock = threading.Lock()
        self.events = []
        self.ros_port, self.gazebo_port = free_ports()
        os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:{}'.format(self.ros_port)
        os.environ['GAZEBO_MASTER_URI'] = 'http://127.0.0.1:{}'.format(self.gazebo_port)
        os.environ['GAZEBO_MODEL_DATABASE_URI'] = ''
        os.environ['ROS_LOG_DIR'] = str(evidence / 'ros-log')
        os.environ['ROS_HOME'] = str(evidence / 'ros-home')
        os.environ['ROS_IP'] = '127.0.0.1'
        os.environ.pop('ROS_HOSTNAME', None)
        self.target_id = 'scene-integration'
        self.run_id = uuid.uuid4().hex
        self.world_socket = evidence / 'world.sock'
        self.adapter_socket = evidence / 'scene.sock'
        self.xrpc = self.scene_client = None
        self.packages = rospkg.RosPack()
        source_world = Path(self.packages.get_path('gazebo_sim_worlds')) / 'worlds/scene_editable/scene_editable.world'
        assert source_world.is_file(), 'Source the updated Gazebo scenes overlay before running: ' + str(source_world)
        try:
            nodes = roslib.packages.find_node('xgc_scene_ros1_adapter', 'xgc2_scene_adapter',
                                             rospack=self.packages)
        except rospkg.ResourceNotFound:
            nodes = []
        assert nodes, 'Install ros-noetic-xgc2-scene-adapter and source its overlay: xgc2_scene_adapter is absent'
        self.scene_adapter = Path(nodes[0])
        self.world = evidence / 'prepared-world.world'
        sdf = ET.parse(source_world)
        plugin = ET.SubElement(sdf.getroot().find('world'), 'plugin',
                               {'name': 'xgc2_simulation', 'filename': 'libxgc2_simulation_world.so'})
        for key, value in (('socket_path', self.world_socket), ('target_id', self.target_id),
                           ('resource_root', evidence), ('configuration_revision', 1)):
            ET.SubElement(plugin, key).text = str(value)
        sdf.write(self.world, encoding='utf-8', xml_declaration=True)

    def start(self, name, args):
        log = (self.evidence / (name + '.log')).open('wb')
        self.logs.append(log)
        process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.processes.append(process)
        self.events.append({'process': name, 'pid': process.pid, 'argv': [str(arg) for arg in args]})
        return process

    def stop(self, process):
        if process.poll() is not None:
            process.wait()
            return
        # A failed assertion may leave the simulator suspended by a fault test.
        os.killpg(process.pid, signal.SIGCONT)
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            os.killpg(process.pid, sig)
            try:
                process.wait(timeout=5)
                return
            except subprocess.TimeoutExpired:
                pass

    def start_gazebo(self):
        self.gzserver = self.start('gazebo', [
            'gzserver', '--verbose', '-s', 'libgazebo_ros_api_plugin.so',
            '-s', 'libxgc2_gazebo_scene_system.so', str(self.world)])
        wait_for(lambda: self.world_socket.exists(), 'native world socket', timeout=30)
        discovery = Client(str(self.world_socket), runtime=self.xrpc)
        try:
            description = discovery.json('/v1/describe?wait_ready_ms=30000', method='GET', timeout=35)
            self.simulation_reference = description['service_ref']
        finally:
            discovery.close()
        rospy.wait_for_service('/gazebo/get_model_state', timeout=30)
        self.get_model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    def start_adapter(self, source, suffix=''):
        self.adapter = self.start('adapter' + suffix, [
            str(self.scene_adapter), '--socket-path', str(self.adapter_socket),
            '--target-id', self.target_id, '--run-id', self.run_id,
            '--node-name', 'scene_adapter_' + self.run_id,
            '--ros-master-uri', os.environ['ROS_MASTER_URI'], '--ros-ip', '127.0.0.1',
            '__ns:=/xgc/scene', '_scene_file:=' + str(source),
            '_save_directory:=' + str(self.evidence), '_profile:=gazebo',
            '_target_id:=' + self.target_id, '_xrpc_socket:=' + str(self.adapter_socket),
            '_simulation_service_ref_json:=' + json.dumps(self.simulation_reference)])
        wait_for(lambda: self.adapter_socket.exists(), 'scene adapter socket', timeout=35)
        discovery = Client(str(self.adapter_socket), runtime=self.xrpc)
        try:
            description = discovery.json('/v1/describe?wait_ready_ms=30000', method='GET', timeout=35)
            assert description['ready'], description
            self.scene_client = Client.from_service(ServiceRef.from_dict(description['service_ref']),
                runtime=self.xrpc, local_target=self.target_id,
                limits=Limits(body_bytes=8*1024*1024, response_bytes=8*1024*1024+65536, call_timeout=60))
        finally:
            discovery.close()
        self.envelope = self.command('get')

    def sample(self, name, value):
        with self.lock:
            self.samples[name] = value

    def command(self, operation, expected=None, request_id=None, success=True, **fields):
        command = {'operation': operation}
        if operation != 'get':
            current = expected or self.envelope
            command.update(expectedEpoch=current['epoch'], expectedRevision=current['revision'])
        command.update(fields)
        started = time.monotonic()
        if operation == 'get':
            envelope = self.scene_client.json('/v1/scene', method='GET')
        else:
            envelope = self.scene_client.json('/v1/commands', command,
                                              request_id=request_id or uuid.uuid4().hex, timeout=60)
        succeeded = True if operation == 'get' else envelope['success']
        self.events.append({'operation': operation, 'success': succeeded,
                            'revision': envelope.get('revision'), 'seconds': time.monotonic() - started,
                            'error': envelope.get('error')})
        assert succeeded == success, {
            'operation': operation, 'success': succeeded, 'error': envelope.get('error'),
            'revision': envelope.get('revision'), 'consumers': envelope.get('consumers')}
        self.envelope = envelope
        return envelope

    def verify_collisions(self, document):
        """Use collision-derived planning messages, independently of apply's receipt."""
        with self.lock:
            library = self.samples.get('geometry_library')
            observed = self.samples.get('instances')
        assert library is not None and observed is not None, 'no collision observer snapshot'
        assert library.header.frame_id == observed.header.frame_id == 'world'
        templates = {item.type: item for item in library.templates}
        instances = {item.name: item for item in observed.instances}
        expected_names = {
            model_name(item['id'])[len('xgc2_obstacle_'):] + (
                '/body/part_' + part['id'].encode('utf-8').hex() if len(item['parts']) != 1 else '')
            for item in document['obstacles'] for part in item['parts']
        }
        assert set(instances) == expected_names
        shape_types = {'box': 'cube', 'sphere': 'sphere', 'cylinder': 'cylinder'}
        for item in document['obstacles']:
            state = self.get_model(model_name(item['id']), 'world')
            assert state.success, state.status_message
            close(vector(state.pose.position), item['pose']['position'])
            q = state.pose.orientation
            actual_q = [q.x, q.y, q.z, q.w]
            expected_q = item['pose']['orientation']
            assert abs(abs(sum(a*b for a, b in zip(actual_q, expected_q))) - 1) < 1e-6
            logical_name = model_name(item['id'])[len('xgc2_obstacle_'):]
            for expected in item['parts']:
                name = logical_name
                if len(item['parts']) != 1:
                    name += '/body/part_' + expected['id'].encode('utf-8').hex()
                actual = instances[name]
                local_pose = expected.get('pose', {'position': [0, 0, 0], 'orientation': [0, 0, 0, 1]})
                # Compose the model and collision poses to check the published world pose.
                x, y, z = local_pose['position']
                qx, qy, qz, qw = actual_q
                tx, ty, tz = 2*(qy*z-qz*y), 2*(qz*x-qx*z), 2*(qx*y-qy*x)
                offset = [x+qw*tx+qy*tz-qz*ty, y+qw*ty+qz*tx-qx*tz, z+qw*tz+qx*ty-qy*tx]
                close(vector(actual.pose.position), [a+b for a, b in zip(vector(state.pose.position), offset)])
                lx, ly, lz, lw = local_pose['orientation']
                rotation = [qw*lx+qx*lw+qy*lz-qz*ly, qw*ly-qx*lz+qy*lw+qz*lx,
                            qw*lz+qx*ly-qy*lx+qz*lw, qw*lw-qx*lx-qy*ly-qz*lz]
                orientation = actual.pose.orientation
                assert abs(abs(sum(a*b for a, b in zip(
                    [orientation.x, orientation.y, orientation.z, orientation.w], rotation))) - 1) < 1e-6
                geometry = expected['geometry']
                if geometry['type'] == 'convex':
                    assert actual.geometry_type.startswith('convex_mesh:')
                    assert actual.geometry_type in templates
                    close(vector(actual.scale), [1, 1, 1])
                    vertices = sorted(vector(vertex) for vertex in templates[actual.geometry_type].support_points)
                    expected_vertices = sorted(geometry['vertices'])
                    assert len(vertices) == len(expected_vertices)
                    for left, right in zip(vertices, expected_vertices):
                        close(left, right)
                else:
                    assert actual.geometry_type == shape_types[geometry['type']]
                    assert actual.geometry_type in templates
                    if geometry['type'] == 'box':
                        close(vector(actual.scale), geometry['size'])
                    elif geometry['type'] == 'sphere':
                        close(vector(actual.scale), [geometry['radius']] * 3)
                    else:
                        close(vector(actual.scale), [geometry['radius'], geometry['radius'], geometry['height']])
        return True

    def verify_current(self):
        document = copy.deepcopy(self.envelope['document'])
        wait_for(lambda: self.verify_collisions(document), 'physical scene convergence')
        current = self.command('get')
        assert current['synchronized'], current['consumers']

    def run(self):
        self.start('roscore', ['roscore', '-p', str(self.ros_port)])
        master = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        wait_for(lambda: master.getPid('/scene_integration')[0] == 1, 'private ROS master')
        rospy.init_node('scene_integration', anonymous=True, disable_signals=True)
        rospy.set_param('/use_sim_time', True)
        self.subscribers = [
            rospy.Subscriber('/xgc2/simulation/obstacles/geometry_library', GeometryLibrary,
                             lambda value: self.sample('geometry_library', value), queue_size=1),
            rospy.Subscriber('/xgc2/simulation/obstacles/instances', ConvexBodyArray,
                             lambda value: self.sample('instances', value), queue_size=1),
            rospy.Subscriber('/xgc/scene/state', SceneState, lambda value: self.sample('state', value), queue_size=1),
        ]
        self.xrpc = Runtime(blocking_workers=1, max_calls=4)
        self.start_gazebo()
        self.start_adapter(self.source)
        original = copy.deepcopy(self.envelope)
        counts = Counter(part['geometry']['type'] for body in original['document']['obstacles'] for part in body['parts'])
        assert counts == {'box': 4, 'sphere': 4, 'cylinder': 4, 'convex': 4}, counts
        self.verify_current()
        # Both late subscribers must receive the accepted initial scene immediately.
        late_document = json.loads(rospy.wait_for_message('/xgc/scene/document', String, timeout=3).data)
        for service in ('configure_motions', 'stop_motions'):
            assert master.lookupService('/scene_integration', '/xgc2/gazebo/obstacles/' + service)[0] != 1
        self.events.append({'motion_authority': 'collision observer exposes no motion ROS services'})

        late_snapshot = rospy.wait_for_message('/xgc/scene/snapshot', SceneSnapshot, timeout=3)
        assert late_document['epoch'] == late_snapshot.epoch == original['epoch']
        assert len(late_snapshot.obstacles) == 16

        box = {'id': 'online-box', 'name': 'Online box', 'pose': {'position': [25, -4, 1.5], 'orientation': [0, 0, 0, 1]},
               'parts': [{'id': 'body', 'geometry': {'type': 'box', 'size': [1, 2, 3]}}], 'motion': {'type': 'hold'}}
        add_id = uuid.uuid4().hex
        self.command('add', obstacle=box, request_id=add_id)
        added_revision = self.envelope['revision']
        self.command('add', obstacle=box, request_id=add_id, expected=original)
        assert self.envelope['revision'] == added_revision
        self.verify_current()
        box['pose'] = {'position': [26, -3, 2], 'orientation': [0, 0, math.sin(0.2), math.cos(0.2)]}
        self.command('update', obstacle=box)
        self.verify_current()
        box['parts'][0]['geometry']['size'] = [2.5, 1.25, 3.5]
        self.command('update', obstacle=box)
        self.verify_current()
        self.command('delete', id='online-box', expected=original, success=False)
        assert len(self.envelope['document']['obstacles']) == 17
        removed = original['document']['obstacles'][7]['id']
        self.command('delete', id=removed)
        self.verify_current()
        assert not self.get_model(model_name(removed), 'world').success
        self.command('undo')
        self.verify_current()
        self.command('redo')
        self.verify_current()
        self.command('undo')
        self.command('save', path='edited.yaml')
        assert not self.envelope['dirty']
        assert yaml.safe_load((self.evidence / 'edited.yaml').read_text()) == self.envelope['document']

        box['motion'] = {'type': 'constant_twist', 'linear': [0.5, 0, 0], 'angular': [0, 0, 0]}
        self.command('update', obstacle=box)
        self.command('play')
        initial_x = box['pose']['position'][0]
        wait_for(lambda: self.get_model(model_name('online-box'), 'world').pose.position.x > initial_x + 0.15,
                 'independent scene motion')
        self.command('pause')
        time.sleep(0.1)
        paused_x = self.get_model(model_name('online-box'), 'world').pose.position.x
        time.sleep(0.2)
        close([self.get_model(model_name('online-box'), 'world').pose.position.x], [paused_x])
        self.command('save', path='moving.yaml')
        saved = yaml.safe_load((self.evidence / 'moving.yaml').read_text())
        assert saved['obstacles'][-1]['pose']['position'][0] == initial_x
        assert paused_x > initial_x + 0.1
        self.command('reset')
        wait_for(lambda: abs(self.get_model(model_name('online-box'), 'world').pose.position.x - initial_x) < 1e-6,
                 'scene motion reset')

        previous_epoch = copy.deepcopy(self.envelope)
        self.scene_client.close()
        self.scene_client = None
        self.stop(self.adapter)
        wait_for(lambda: not self.adapter_socket.exists(), 'old scene adapter shutdown')
        self.start_adapter(self.evidence / 'moving.yaml', '-reload')
        assert self.envelope['epoch'] != previous_epoch['epoch']
        assert self.envelope['document'] == saved
        self.verify_current()
        self.command('clear', expected=previous_epoch, success=False)

        # An unresponsive simulator must not acknowledge an edit or erase the
        # document. Preserve its bound instance while testing the exact retry.
        os.killpg(self.gzserver.pid, signal.SIGSTOP)
        before_failure = copy.deepcopy(self.envelope)
        retry_id = uuid.uuid4().hex
        extra = copy.deepcopy(box)
        extra['id'] = 'after-recovery'
        extra['motion'] = {'type': 'hold'}
        self.command('add', obstacle=extra, request_id=retry_id, success=False)
        assert self.envelope['revision'] == before_failure['revision']
        assert self.envelope['document'] == before_failure['document']
        assert not self.envelope['synchronized']
        assert any(item['consumer'] == 'gazebo' and not item['applied'] for item in self.envelope['consumers'])
        os.killpg(self.gzserver.pid, signal.SIGCONT)
        wait_for(lambda: self.get_model('ground_plane', 'world').success, 'simulator recovery')
        self.command('add', obstacle=extra, request_id=retry_id, expected=before_failure)
        self.verify_current()
        assert self.get_model('ground_plane', 'world').success

        # Retry sync is also able to restore the accepted document without
        # replaying an unwanted failed edit or changing save/history semantics.
        os.killpg(self.gzserver.pid, signal.SIGSTOP)
        before_resync = copy.deepcopy(self.envelope)
        extra['parts'][0]['geometry']['size'][0] = 4.25
        self.command('update', obstacle=extra, success=False)
        os.killpg(self.gzserver.pid, signal.SIGCONT)
        wait_for(lambda: self.get_model('ground_plane', 'world').success, 'simulator resync recovery')
        self.command('resync')
        assert self.envelope['revision'] > before_resync['revision']
        assert self.envelope['document'] == before_resync['document']
        assert self.envelope['dirty'] == before_resync['dirty']
        self.verify_current()
        self.events.append({'result': 'passed', 'original_geometry_counts': dict(counts),
                            'final_revision': self.envelope['revision'], 'ros_port': self.ros_port,
                            'gazebo_port': self.gazebo_port})

    def close(self):
        if self.scene_client is not None:
            self.scene_client.close()
        if self.xrpc is not None:
            self.xrpc.close()
        for process in reversed(self.processes):
            self.stop(process)
        for log in self.logs:
            log.close()
        (self.evidence / 'result.json').write_text(json.dumps(self.events, indent=2) + '\n')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene-file', type=Path, required=True)
    parser.add_argument('--evidence-dir', type=Path, required=True, help='New directory for logs and temporary scene saves')
    args = parser.parse_args()
    runner = Integration(args.scene_file.resolve(), args.evidence_dir.resolve())
    try:
        runner.run()
        print('Original 16 obstacle adapter/Gazebo integration passed:', runner.evidence)
    except Exception as error:
        runner.events.append({'result': 'failed', 'error': str(error)})
        raise
    finally:
        runner.close()


if __name__ == '__main__':
    main()
