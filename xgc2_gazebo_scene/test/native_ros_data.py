#!/usr/bin/env python3
"""Actual ROS user data from an explicitly started, isolated Classic world."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import random
import socket
import signal
import subprocess
import tempfile
import threading
import time


def free_port():
    for _ in range(100):
        port = random.randrange(20000, 40001)
        with socket.socket() as reserved:
            try:
                reserved.bind(('127.0.0.1', port))
                return port
            except OSError:
                pass
    raise RuntimeError('No private fixture port available')


def stop(process):
    if process is None or process.poll() is not None:
        return True
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()
        return False
    return process.returncode == 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--plugins', required=True)
    parser.add_argument('--gzserver', required=True)
    args = parser.parse_args()
    gzserver = Path(args.gzserver).resolve(strict=True)
    scene = Path(__file__).resolve().parents[1]
    entry = scene.parent/'gazebo_sim_worlds/scripts/native_world_start'
    import roslib.packages
    import rospkg
    try:
        tools = roslib.packages.find_node('xgc_scene_ros1_adapter', 'xgc2-scene-prepare',
                                         rospack=rospkg.RosPack())
    except rospkg.ResourceNotFound:
        tools = []
    if not tools:
        raise SystemExit('Install ros-noetic-xgc2-scene-adapter (>= 0.7.0-1) and source its overlay: '
                         'xgc2-scene-prepare is absent from xgc_scene_ros1_adapter')
    with tempfile.TemporaryDirectory(prefix='xgc2-native-data-') as directory:
        root = Path(directory)
        root.chmod(0o700)
        asset = root/'Shared/Scenes/user-data'
        asset.mkdir(parents=True)
        source = asset/'source.world'
        source.write_text('<sdf version="1.6"><world name="user_data"><gravity>0 0 0</gravity>'
                          '<physics type="ode"><max_step_size>0.001</max_step_size>'
                          '<real_time_update_rate>1000</real_time_update_rate></physics></world></sdf>')
        output = root/'prepared'
        output.mkdir()
        endpoint = root/'world.sock'
        (asset/'manifest.json').write_text(json.dumps({
            'schemaVersion': 1, 'kind': 'xgc.scene-replay-asset.v1',
            'simulators': {'gazebo': {'geometry': 'native'}},
            'gazeboWorld': {'file': source.name, 'sha256': hashlib.sha256(source.read_bytes()).hexdigest()}}))
        request = {'openingRunId': 'data-fixture', 'scene': {
            'asset': 'user-data', 'simulator': 'gazebo', 'parameters': {'clock_rate_hz': 0}}}
        prepared = json.loads(subprocess.check_output([
            tools[0], '--context-json', json.dumps(request), '--resource-root', str(root),
            '--workspace-root', str(output), '--prepare-world', '--socket-path', str(endpoint),
            '--target-id', 'data-fixture', '--configuration-revision', '13'], text=True, timeout=30))
        os.environ.pop('DISPLAY', None)
        os.environ.pop('WAYLAND_DISPLAY', None)
        os.environ.pop('ROS_HOSTNAME', None)
        os.environ.update(ROS_MASTER_URI='http://127.0.0.1:'+str(free_port()), ROS_IP='127.0.0.1',
                          ROS_HOME=str(root/'ros'), ROS_LOG_DIR=str(root/'logs'))
        # Import ROS only after installing this fixture's private master identity.
        import rosgraph
        import rospy
        from gazebo_msgs.msg import ModelStates
        from rosgraph_msgs.msg import Clock
        from xgc2_xrpc.http import Client
        from xgc2_xrpc.runtime import Runtime
        environment = dict(os.environ)
        environment.update(HOME=str(root), GAZEBO_MASTER_URI='http://127.0.0.1:'+str(free_port()),
            GAZEBO_MODEL_DATABASE_URI='', GAZEBO_PLUGIN_PATH=args.plugins)
        command = [str(entry), '--world', prepared['worldFile'], '--paused', 'true', '--gui', 'false',
                   '--ros-data', 'true', '--gazebo-bin', str(gzserver)]
        master = world = None
        runtime = discovery = client = None
        subscriber = clock_subscriber = None
        with (root/'master.log').open('w+') as master_log, (root/'world.log').open('w+') as world_log:
            try:
                master = subprocess.Popen(['rosmaster', '--core', '-p', os.environ['ROS_MASTER_URI'].rsplit(':', 1)[1]],
                                          env=environment, stdout=master_log, stderr=subprocess.STDOUT)
                deadline = time.monotonic()+10
                while True:
                    if master.poll() is not None:
                        raise AssertionError('private ROS master exited')
                    try:
                        rosgraph.Master('/native_data_fixture').getPid()
                        break
                    except (OSError, rosgraph.MasterException):
                        if time.monotonic() >= deadline:
                            raise AssertionError('private ROS master did not start')
                        time.sleep(.02)
                rospy.init_node('native_data_fixture', disable_signals=True)
                lock = threading.Condition()
                clocks, frames = [], []

                def clock_callback(message):
                    with lock:
                        clocks.append(message.clock.to_nsec())
                        lock.notify_all()

                def model_callback(message):
                    with lock:
                        frames.append(message)
                        lock.notify_all()

                def await_data(predicate):
                    deadline = time.monotonic()+5
                    with lock:
                        while not predicate():
                            if world.poll() is not None or time.monotonic() >= deadline:
                                raise AssertionError('native ROS data was not observed')
                            lock.wait(min(.1, deadline-time.monotonic()))

                clock_subscriber = rospy.Subscriber('/clock', Clock, clock_callback, queue_size=100)
                world = subprocess.Popen(command, env=environment, stdout=world_log, stderr=subprocess.STDOUT)
                deadline = time.monotonic()+15
                while not endpoint.exists():
                    if world.poll() is not None or time.monotonic() >= deadline:
                        raise AssertionError('explicit native world did not start')
                    time.sleep(.02)
                runtime = Runtime(blocking_workers=1, max_calls=8)
                discovery = Client(str(endpoint), runtime=runtime)
                description = json.loads(discovery.call('/v1/describe', method='GET').body)
                assert description['facts']['simulation']['configuration']['revision'] == '13', description
                assert description['facts']['simulation']['sensor_kinds'] == ['camera', 'imu', 'lidar'], description
                ref = description['facts']['service_ref']
                client = Client(str(endpoint), runtime=runtime, instance_id=ref['instance_id'])
                counter = 0

                def query(route):
                    return json.loads(client.call(route, method='GET', timeout=5).body)

                def mutate(route, body):
                    nonlocal counter
                    counter += 1
                    accepted = json.loads(client.call(route, dict(body, operation_timeout_ms=5000),
                        timeout=5, request_id='data-'+str(counter)).body)
                    result = json.loads(client.call('/v1/operations/'+accepted['id']+'/wait', {}, timeout=10).body)
                    assert result['state'] == 'succeeded', result
                    return result['result']

                pose = {'position': [1, 2, 3], 'orientation': [0, 0, 0, 1]}
                artifact = ('<sdf version="1.6"><model name="original"><link name="body"><inertial><mass>1</mass>'
                    '<inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial></link></model></sdf>')
                created = mutate('/v1/entities', {'entity': {'id': 'robot-1', 'role': 'robot', 'pose': pose,
                    'asset': {'id': 'fixture', 'realization': {'media_type': 'application/sdf+xml', 'content': artifact}}}})
                generation = created['entities'][0]['ref']['generation']
                subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, model_callback, queue_size=100)
                await_data(lambda: any('robot-1' in message.name for message in frames))
                current = next(message for message in reversed(frames) if 'robot-1' in message.name)
                index = current.name.index('robot-1')
                assert current.pose[index].position.x == 1 and current.pose[index].position.y == 2
                assert not any(name.startswith('xgc2_native_') for name in current.name), current.name
                assert query('/v1/world')['time']['nanoseconds'] == '0', 'paused data advanced time'
                moved_pose = {'position': [4, 5, 6], 'orientation': [0, 0, 0, 1]}
                twist = {'linear': [.4, -.2, .1], 'angular': [.05, .1, -.1]}
                mutate('/v1/entities/robot-1/state', {'generation': generation, 'state': {'pose': moved_pose, 'twist': twist}})
                # Subscriber demand triggers one current snapshot even while paused.
                subscriber.unregister()
                with lock:
                    frames.clear()
                subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, model_callback, queue_size=100)
                await_data(lambda: any('robot-1' in message.name and message.pose[message.name.index('robot-1')].position.x == 4 for message in frames))
                current = next(message for message in reversed(frames) if 'robot-1' in message.name)
                actual = current.twist[current.name.index('robot-1')]
                for field, expected in ((actual.linear, twist['linear']), (actual.angular, twist['angular'])):
                    assert all(abs(a-b) < 1e-12 for a, b in zip((field.x, field.y, field.z), expected)), (field, expected)
                stepped = mutate('/v1/world/step', {'steps': 7})
                stamp = int(stepped['time']['nanoseconds'])
                assert stamp == 7000000
                await_data(lambda: stamp in clocks)
                count = len(clocks)
                time.sleep(.05)
                assert len(clocks) == count, 'paused source invented clock ticks'
                clock_subscriber.unregister()
                with lock:
                    clocks.clear()
                clock_subscriber = rospy.Subscriber('/clock', Clock, clock_callback, queue_size=1)
                await_data(lambda: clocks and clocks[-1] == stamp)
                assert query('/v1/world')['time']['nanoseconds'] == str(stamp), 'clock reconnect advanced time'
                mutate('/v1/world/reset', {'scope': 'all_entities', 'reset_time': True})
                await_data(lambda: clocks and clocks[-1] == 0)
                services = rosgraph.Master('/native_data_fixture').getSystemState()[2]
                retired = {'/gazebo/spawn_sdf_model', '/gazebo/delete_model', '/gazebo/set_model_state',
                           '/gazebo/pause_physics', '/gazebo/unpause_physics'}
                assert not retired.intersection(name for name, _ in services), services
                pid = str(world.pid)
                executable = os.readlink('/proc/'+pid+'/exe')
                assert Path(executable) == gzserver, executable
                version = subprocess.run([str(gzserver), '--version'], env=environment, text=True,
                                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=5)
                assert 'version 11.' in version.stdout, version.stdout
                print(json.dumps({'pid': world.pid, 'exe': executable, 'rootfs': os.readlink('/proc/'+pid+'/root'),
                    'version': version.stdout.splitlines()[0], 'version_exit': version.returncode,
                    'command': command, 'gzclient': False, 'display': None,
                    'checks': ['configuration-revision', 'logical-model-name', 'paused-exact-pose-and-world-twist',
                               'exact-step-clock', 'paused-clock', 'clock-rewind', 'no-platform-ros-services']}))
            except BaseException:
                world_log.flush()
                world_log.seek(0)
                print(world_log.read())
                raise
            finally:
                if subscriber is not None: subscriber.unregister()
                if clock_subscriber is not None: clock_subscriber.unregister()
                if client is not None: client.close()
                if discovery is not None: discovery.close()
                if runtime is not None: runtime.close()
                drained = stop(world)
                rospy.signal_shutdown('explicit fixture shutdown')
                stop(master)
                if world is not None:
                    assert drained and world.returncode == 0, 'native world SIGINT drain failed'
                    assert not endpoint.exists(), 'native world left its owned UDS after SIGINT drain'
                    print(json.dumps({'shutdown_pid': world.pid, 'signal': 'SIGINT', 'exit_code': world.returncode,
                                      'owned_socket_removed': True}))


if __name__ == '__main__':
    main()
