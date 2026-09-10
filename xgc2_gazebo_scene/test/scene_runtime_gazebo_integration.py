#!/usr/bin/env python3
"""Isolated runtime/Gazebo acceptance. Takes the algorithm-owned scene as input.

The legacy observer is loaded only to independently read actual collision shapes;
its motion services must reject scene-owned targets. No planner or control node is started.
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

import rospkg
import rospy
import yaml
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import String
from xgc2_gazebo_scene.msg import ConvexPart, ObstacleDefinitionArray, MotionSpec
from xgc2_gazebo_scene.srv import ConfigureMotions, StopMotions
from xgc2_geometry_msgs.msg import SceneSnapshot, SceneState
from xgc2_geometry_msgs.srv import SceneCommand


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
            sock.bind(('127.0.0.1', 0))
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
        self.packages = rospkg.RosPack()
        self.world = Path(self.packages.get_path('gazebo_sim_worlds')) / 'worlds/scene_editable/scene_editable.world'
        self.scene_node = Path(self.packages.get_path('xgc2_scene_runtime')) / 'scripts/scene_node'
        assert self.world.is_file(), 'Source the updated scene product overlay before running: ' + str(self.world)
        assert self.scene_node.is_file(), self.scene_node

    def start(self, name, args):
        log = (self.evidence / (name + '.log')).open('wb')
        self.logs.append(log)
        process = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.processes.append(process)
        self.events.append({'process': name, 'pid': process.pid, 'argv': [str(arg) for arg in args]})
        return process

    def stop(self, process):
        if process.poll() is not None:
            return
        for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
            os.killpg(process.pid, sig)
            try:
                process.wait(timeout=5)
                return
            except subprocess.TimeoutExpired:
                pass

    def start_gazebo(self, suffix=''):
        self.gzserver = self.start('gazebo' + suffix, [
            'gzserver', '--verbose', '-s', 'libgazebo_ros_api_plugin.so',
            '-s', 'libxgc2_gazebo_scene_system.so', str(self.world)])
        rospy.wait_for_service('/xgc/scene/gazebo/apply', timeout=30)
        rospy.wait_for_service('/gazebo/get_model_state', timeout=30)
        self.get_model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

    def start_runtime(self, source, suffix=''):
        self.runtime = self.start('runtime' + suffix, [
            '/usr/bin/python3', str(self.scene_node), '__ns:=/xgc/scene',
            '_scene_file:=' + str(source), '_save_directory:=' + str(self.evidence), '_gazebo:=true'])
        rospy.wait_for_service('/xgc/scene/command', timeout=35)
        self.command_service = rospy.ServiceProxy('/xgc/scene/command', SceneCommand)
        self.envelope = self.command('get')

    def sample(self, name, value):
        with self.lock:
            self.samples[name] = value

    def command(self, operation, expected=None, request_id=None, success=True, **fields):
        command = {'operation': operation}
        if operation != 'get':
            current = expected or self.envelope
            command.update(requestId=request_id or uuid.uuid4().hex,
                           expectedEpoch=current['epoch'], expectedRevision=current['revision'])
        command.update(fields)
        started = time.monotonic()
        result = self.command_service(json.dumps(command, allow_nan=False))
        envelope = json.loads(result.result_json)
        self.events.append({'operation': operation, 'success': result.success,
                            'revision': envelope.get('revision'), 'seconds': time.monotonic() - started,
                            'error': envelope.get('error')})
        assert result.success == envelope['success'] == success, {
            'operation': operation, 'success': result.success, 'error': envelope.get('error'),
            'revision': envelope.get('revision'), 'consumers': envelope.get('consumers')}
        self.envelope = envelope
        return envelope

    def verify_collisions(self, document):
        """Use collision-derived native messages, independently of apply's receipt."""
        with self.lock:
            observed = self.samples.get('geometry')
        assert observed is not None, 'no collision observer snapshot'
        bodies = {item.model_name: item for item in observed.obstacles}
        assert set(bodies) == {model_name(item['id']) for item in document['obstacles']}
        shape_codes = {'box': ConvexPart.SHAPE_BOX, 'sphere': ConvexPart.SHAPE_SPHERE,
                       'cylinder': ConvexPart.SHAPE_CYLINDER, 'convex': ConvexPart.SHAPE_CONVEX_MESH}
        for item in document['obstacles']:
            body = bodies[model_name(item['id'])]
            assert len(body.parts) == len(item['parts'])
            parts = {part.part_id.rsplit('/', 1)[-1]: part for part in body.parts}
            for expected in item['parts']:
                actual = parts['part_' + expected['id'].encode('utf-8').hex()]
                geometry = expected['geometry']
                assert actual.shape == shape_codes[geometry['type']]
                close(vector(actual.local_pose.position), expected['pose']['position'])
                if geometry['type'] == 'box':
                    close(vector(actual.size), geometry['size'])
                elif geometry['type'] in ('sphere', 'cylinder'):
                    close([actual.radius], [geometry['radius']])
                    if geometry['type'] == 'cylinder':
                        close([actual.length], [geometry['height']])
                else:
                    close(vector(actual.mesh_scale), [1, 1, 1])
                    vertices = sorted(vector(vertex) for vertex in actual.conservative_vertices)
                    expected_vertices = sorted(geometry['vertices'])
                    assert len(vertices) == len(expected_vertices)
                    for left, right in zip(vertices, expected_vertices):
                        close(left, right)
            state = self.get_model(model_name(item['id']), 'world')
            assert state.success, state.status_message
            close(vector(state.pose.position), item['pose']['position'])
            q = state.pose.orientation
            actual_q = [q.x, q.y, q.z, q.w]
            expected_q = item['pose']['orientation']
            assert abs(abs(sum(a*b for a, b in zip(actual_q, expected_q))) - 1) < 1e-6
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
            rospy.Subscriber('/xgc2/simulation/obstacles/geometry', ObstacleDefinitionArray,
                             lambda value: self.sample('geometry', value), queue_size=1),
            rospy.Subscriber('/xgc/scene/state', SceneState, lambda value: self.sample('state', value), queue_size=1),
        ]
        self.start_gazebo()
        self.start_runtime(self.source)
        original = copy.deepcopy(self.envelope)
        counts = Counter(part['geometry']['type'] for body in original['document']['obstacles'] for part in body['parts'])
        assert counts == {'box': 4, 'sphere': 4, 'cylinder': 4, 'convex': 4}, counts
        self.verify_current()
        # Both late subscribers must receive the accepted initial scene immediately.
        late_document = json.loads(rospy.wait_for_message('/xgc/scene/document', String, timeout=3).data)
        rospy.wait_for_service('/xgc2/gazebo/obstacles/configure_motions', timeout=5)
        configure = rospy.ServiceProxy('/xgc2/gazebo/obstacles/configure_motions', ConfigureMotions)
        stop = rospy.ServiceProxy('/xgc2/gazebo/obstacles/stop_motions', StopMotions)
        logical_name = model_name(original['document']['obstacles'][0]['id'])[len('xgc2_obstacle_'):]
        motion = MotionSpec()
        motion.name = logical_name
        denied = configure(command_id=uuid.uuid4().hex, expected_scene_revision=0, motions=[motion])
        assert not denied.success and 'scene runtime' in denied.message, denied
        denied = stop(command_id=uuid.uuid4().hex, expected_scene_revision=0, names=[logical_name])
        assert not denied.success and 'scene runtime' in denied.message, denied
        stopped = stop(command_id=uuid.uuid4().hex, expected_scene_revision=0, names=[])
        assert stopped.success and not stopped.motion_revisions, stopped
        self.events.append({'old_motion_authority': 'explicit configure/stop rejected; global stop owns no scene model'})

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
        self.stop(self.runtime)
        wait_for(lambda: not master.lookupService('/scene_integration', '/xgc/scene/command')[0] == 1,
                 'old scene runtime shutdown')
        self.start_runtime(self.evidence / 'moving.yaml', '-reload')
        assert self.envelope['epoch'] != previous_epoch['epoch']
        assert self.envelope['document'] == saved
        self.verify_current()
        self.command('clear', expected=previous_epoch, success=False)

        # A dead simulator must not acknowledge an accepted edit or erase the
        # current authoring document. Restart and retry the exact command.
        self.stop(self.gzserver)
        before_failure = copy.deepcopy(self.envelope)
        retry_id = uuid.uuid4().hex
        extra = copy.deepcopy(box)
        extra['id'] = 'after-recovery'
        extra['motion'] = {'type': 'hold'}
        self.command('add', obstacle=extra, request_id=retry_id, success=False)
        assert self.envelope['revision'] == before_failure['revision']
        assert self.envelope['document'] == before_failure['document']
        assert not self.envelope['synchronized']
        assert any(item['consumer'] == 'gazebo' and not item['success'] for item in self.envelope['consumers'])
        self.start_gazebo('-recovered')
        self.command('add', obstacle=extra, request_id=retry_id, expected=before_failure)
        self.verify_current()
        assert self.get_model('ground_plane', 'world').success

        # Retry sync is also able to restore the accepted document without
        # replaying an unwanted failed edit or changing save/history semantics.
        self.stop(self.gzserver)
        before_resync = copy.deepcopy(self.envelope)
        extra['parts'][0]['geometry']['size'][0] = 4.25
        self.command('update', obstacle=extra, success=False)
        self.start_gazebo('-resync')
        self.command('resync')
        assert self.envelope['revision'] > before_resync['revision']
        assert self.envelope['document'] == before_resync['document']
        assert self.envelope['dirty'] == before_resync['dirty']
        self.verify_current()
        self.events.append({'result': 'passed', 'original_geometry_counts': dict(counts),
                            'final_revision': self.envelope['revision'], 'ros_port': self.ros_port,
                            'gazebo_port': self.gazebo_port})

    def close(self):
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
        print('Original 16 obstacle runtime/Gazebo integration passed:', runner.evidence)
    except Exception as error:
        runner.events.append({'result': 'failed', 'error': str(error)})
        raise
    finally:
        runner.close()


if __name__ == '__main__':
    main()
