#!/usr/bin/env python3

import argparse
import copy
import json
import os
import signal
import socket
import subprocess
import time
import xml.etree.ElementTree as ET
import xmlrpc.client
from pathlib import Path


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=(
            'Isolated Gazebo native world-frame scan check; '
            'GPU requires a hardware GL display.'
        )
    )
    parser.add_argument('--ros-port', type=int, required=True)
    parser.add_argument('--gazebo-port', type=int, required=True)
    parser.add_argument('--plugin-dir', type=Path, required=True)
    parser.add_argument('--output-dir', type=Path, required=True)
    parser.add_argument('--acceleration', choices=('cpu', 'gpu'), default='gpu')
    parser.add_argument('--rebuild-cycles', type=int, default=1)
    parser.add_argument(
        '--benchmark-sensors',
        type=int,
        default=0,
        help='Also report one and N active sensors, in the same isolated world',
    )
    return parser.parse_args(argv)


def reserve_ports(args):
    for port in (args.ros_port, args.gazebo_port):
        with socket.socket() as reservation:
            reservation.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            reservation.bind(('127.0.0.1', port))
    if args.ros_port == args.gazebo_port:
        raise ValueError('ROS and Gazebo ports must differ')


def prepare_output(args):
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    results_path = output_dir / 'results.jsonl'
    results_path.write_text('', encoding='utf-8')
    return output_dir, results_path


def make_environment(args):
    environment = os.environ.copy()
    environment.update(
        ROS_MASTER_URI='http://127.0.0.1:' + str(args.ros_port),
        ROS_IP='127.0.0.1',
        GAZEBO_MASTER_URI='http://127.0.0.1:' + str(args.gazebo_port),
        GAZEBO_IP='127.0.0.1',
        GAZEBO_MODEL_DATABASE_URI='',
    )
    environment['GAZEBO_PLUGIN_PATH'] = (
        str(args.plugin_dir.resolve())
        + ':'
        + environment.get('GAZEBO_PLUGIN_PATH', '')
    )
    os.environ.update(environment)
    return environment


def prepare_world(args, output_dir):
    tree = ET.parse(Path(__file__).resolve().with_name('enclosure.world'))
    if args.acceleration == 'cpu':
        sensor = tree.getroot().find("world/model[@name='scanner']/link/sensor")
        sensor.set('type', 'ray')
        sensor.find('plugin').set('filename', 'libxgc2_simple_lidar_cpu.so')
    path = output_dir / 'enclosure.world'
    tree.write(path, encoding='unicode')
    return path


def start_processes(output_dir, environment, ros_port, processes, world):
    commands = [
        ('roscore', ['roscore', '-p', str(ros_port)]),
        (
            'gazebo',
            [
                'gzserver',
                '--verbose',
                '-s',
                'libgazebo_ros_api_plugin.so',
                str(world),
            ],
        ),
    ]
    for name, command in commands:
        log_file = (output_dir / (name + '.log')).open('w')
        process = subprocess.Popen(
            command,
            env=environment,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        processes.append((process, log_file))
        print(name, process.pid, flush=True)
        if name == 'roscore':
            wait_for_roscore(environment['ROS_MASTER_URI'])


def wait_for_roscore(master_uri):
    for _ in range(100):
        try:
            xmlrpc.client.ServerProxy(master_uri).getPid('/lidar_test')
            break
        except OSError:
            time.sleep(0.1)


def write_phase(results_path, phase):
    encoded = json.dumps(phase)
    with results_path.open('a', encoding='utf-8') as results_file:
        results_file.write(encoded + '\n')
    print(encoded, flush=True)


def report_cloud(label, messages, point_cloud2, results_path):
    planes = [(0, 5), (0, -6), (1, 4), (1, -3), (2, 0), (2, 6)]
    errors = []
    for message in messages:
        if message.header.frame_id != 'world' or message.point_step != 12:
            raise RuntimeError('invalid cloud contract')
        points = list(
            point_cloud2.read_points(
                message,
                field_names=('x', 'y', 'z'),
                skip_nans=False,
            )
        )
        for point in points:
            error = min(abs(point[axis] - value) for axis, value in planes)
            errors.append(error)
    errors.sort()
    result = {
        'phase': label,
        'frames': len(messages),
        'points': len(errors),
        'max_wall_error_m': max(errors),
        'p99_wall_error_m': errors[int(0.99 * (len(errors) - 1))],
        'first_stamp': messages[0].header.stamp.to_sec(),
        'last_stamp': messages[-1].header.stamp.to_sec(),
    }
    write_phase(results_path, result)
    if max(errors) > 0.03:
        raise RuntimeError('world cloud drift or projection error')


def process_usage(process):
    fields = Path('/proc/' + str(process.pid) + '/stat').read_text().split()
    ticks_per_second = os.sysconf('SC_CLK_TCK')
    page_size = os.sysconf('SC_PAGE_SIZE')
    cpu_seconds = (int(fields[13]) + int(fields[14])) / ticks_per_second
    rss_bytes = int(fields[23]) * page_size
    return cpu_seconds, rss_bytes


def record_sensor_message(metrics, name, message):
    entry = metrics[name]
    entry['count'] += 1
    entry['stamp'] = message.header.stamp.to_sec()
    entry['bytes'] = len(message.data)


def subscribe_sensor(rospy, point_cloud_type, metrics, subscribers, name, topic):
    metrics[name] = {'count': 0, 'stamp': 0.0, 'bytes': 0}
    subscriber = rospy.Subscriber(
        topic,
        point_cloud_type,
        lambda message, sensor_name=name: record_sensor_message(
            metrics, sensor_name, message
        ),
        queue_size=1,
    )
    subscribers.append(subscriber)


def benchmark_sensor_load(metrics, gazebo_process, results_path, expected_rate):
    time.sleep(2)
    before = copy.deepcopy(metrics)
    cpu_start, _ = process_usage(gazebo_process)
    wall_start = time.monotonic()
    time.sleep(5)
    elapsed = time.monotonic() - wall_start
    cpu_end, rss_bytes = process_usage(gazebo_process)
    observed = {
        name: {
            'wall_hz': (value['count'] - before[name]['count']) / elapsed,
            'real_time_factor': (value['stamp'] - before[name]['stamp']) / elapsed,
            'simulation_hz': (value['count'] - before[name]['count'])
            / (value['stamp'] - before[name]['stamp']),
            'bytes_per_frame': value['bytes'],
        }
        for name, value in metrics.items()
    }
    result = {
        'phase': 'sensor_load',
        'sensors': len(metrics),
        'cpu_cores': (cpu_end - cpu_start) / elapsed,
        'rss_mib': rss_bytes / (1024 * 1024),
        'observed': observed,
    }
    write_phase(results_path, result)
    if any(item['wall_hz'] <= 0 for item in observed.values()):
        raise RuntimeError('one or more sensors stopped publishing')
    if any(
        not expected_rate * 0.85 <= item['simulation_hz'] <= expected_rate * 1.15
        for item in observed.values()
    ):
        raise RuntimeError('native scan rate differs from the configured simulation-time rate')


def wait_for_sensor_frames(metrics, names, baseline, minimum_frames, timeout, error):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if all(
            metrics[name]['count'] - baseline[name]['count'] >= minimum_frames
            for name in names
        ):
            return
        time.sleep(0.05)
    raise RuntimeError(error)


def make_sensor_instance(base_sdf, name):
    instance = copy.deepcopy(base_sdf)
    instance.find('model').set('name', name)
    instance.find('model/link/sensor').set('name', name + '_simple_lidar')
    instance.find('model/link/sensor/plugin/robotNamespace').text = '/' + name
    return instance


def spawn_pose(pose_type):
    pose = pose_type()
    pose.position.z = 1
    pose.orientation.w = 1
    return pose


def check_middle_sensor_rebuild(
    sensor_count,
    sensor_sdfs,
    metrics,
    pose_type,
    spawn_model,
    delete_model,
    results_path,
):
    middle_name = 'scanner_' + str(sensor_count // 2)
    surviving_names = [name for name in metrics if name != middle_name]
    start_time = time.monotonic()

    deleted = delete_model(middle_name)
    if not deleted.success:
        raise RuntimeError('could not delete benchmark sensor: ' + middle_name + ': ' + deleted.status_message)
    time.sleep(0.3)

    during_delete_baseline = copy.deepcopy(metrics)
    wait_for_sensor_frames(
        metrics,
        surviving_names,
        during_delete_baseline,
        3,
        5,
        'a surviving benchmark sensor stopped while the middle sensor was deleted',
    )
    after_delete = copy.deepcopy(metrics)

    restored = spawn_model(
        middle_name,
        sensor_sdfs[middle_name],
        '',
        spawn_pose(pose_type),
        'world',
    )
    if not restored.success:
        raise RuntimeError(restored.status_message)

    after_spawn = copy.deepcopy(metrics)
    names_to_check = [middle_name] + surviving_names
    wait_for_sensor_frames(
        metrics,
        names_to_check,
        after_spawn,
        3,
        5,
        'a benchmark sensor did not resume during middle-sensor rebuild',
    )
    result = {
        'phase': 'middle_sensor_rebuild',
        'sensors': sensor_count,
        'sensor': middle_name,
        'namespace': '/' + middle_name,
        'recovery_frames': metrics[middle_name]['count'] - after_spawn[middle_name]['count'],
        'survivor_frames_during_delete': {
            name: after_delete[name]['count'] - during_delete_baseline[name]['count']
            for name in surviving_names
        },
        'survivor_frames_during_rebuild': {
            name: metrics[name]['count'] - after_spawn[name]['count']
            for name in surviving_names
        },
        'rebuild_wall_s': time.monotonic() - start_time,
    }
    write_phase(results_path, result)


def stop_processes(processes):
    for process, log_file in reversed(processes):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        log_file.close()


def run_check(args):
    reserve_ports(args)
    output_dir, results_path = prepare_output(args)
    environment = make_environment(args)
    if args.acceleration == 'gpu':
        renderer = subprocess.check_output(['glxinfo','-B'],env=environment,text=True,stderr=subprocess.STDOUT)
        (output_dir / 'renderer.txt').write_text(renderer,encoding='utf-8')
        if any(software in renderer.lower() for software in ('llvmpipe','softpipe','software rasterizer','accelerated: no')):
            raise RuntimeError('GPU acceptance requires a hardware renderer; see renderer.txt')

    world = prepare_world(args, output_dir)
    processes = []
    try:
        start_processes(output_dir, environment, args.ros_port, processes, world)
        write_phase(results_path, {'phase': 'configuration', 'acceleration': args.acceleration})

        import rospy
        from gazebo_msgs.msg import ModelState
        from gazebo_msgs.srv import DeleteModel, GetModelState, SetModelState, SpawnModel
        from geometry_msgs.msg import Pose
        from sensor_msgs import point_cloud2
        from sensor_msgs.msg import PointCloud2

        rospy.init_node('simple_lidar_check', disable_signals=True)
        rospy.wait_for_service('/gazebo/set_model_state', timeout=30)
        frames = []
        subscriber = rospy.Subscriber(
            '/test_robot/simple_lidar/points',
            PointCloud2,
            lambda message: frames.append(message),
            queue_size=1,
        )
        deadline = time.monotonic() + 20
        while len(frames) < 5 and time.monotonic() < deadline:
            time.sleep(0.05)
        if len(frames) < 5:
            raise RuntimeError('no native pointcloud frames')
        report_cloud('static_mount', frames[-4:], point_cloud2, results_path)

        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        state = ModelState()
        state.model_name = 'scanner'
        state.reference_frame = 'world'
        state.pose.position.z = 1
        state.pose.orientation.w = 1
        state.twist.linear.x = 0.3
        state.twist.angular.z = 0.6
        response = set_state(state)
        if not response.success:
            raise RuntimeError(response.status_message)
        start = len(frames)
        time.sleep(3)
        report_cloud(
            'translation_rotation_mount',
            frames[start + 2:],
            point_cloud2,
            results_path,
        )

        actual = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)(
            'scanner', 'world'
        )
        if (
            not actual.success
            or actual.pose.position.x < 0.5
            or abs(actual.pose.orientation.z) < 0.2
        ):
            raise RuntimeError('moving test did not actually move and rotate the sensor')

        # Exercise subscriber lifecycle, then delete and respawn a live sensor.
        for _ in range(5):
            subscriber.unregister()
            time.sleep(0.05)
            subscriber = rospy.Subscriber(
                '/test_robot/simple_lidar/points',
                PointCloud2,
                lambda message: frames.append(message),
                queue_size=1,
            )
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        deleted = delete_model('scanner')
        if not deleted.success:
            raise RuntimeError('could not delete the live sensor: ' + deleted.status_message)
        time.sleep(0.3)
        start = len(frames)

        model = ET.parse(world).getroot().find("world/model[@name='scanner']")
        model.find('pose').text = '0 0 0 0 0 0'
        sdf = ET.Element('sdf', version='1.6')
        sdf.append(model)
        result = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)(
            'scanner',
            ET.tostring(sdf, encoding='unicode'),
            '',
            spawn_pose(Pose),
            'world',
        )
        if not result.success:
            raise RuntimeError(result.status_message)
        deadline = time.monotonic() + 5
        while len(frames) < start + 3 and time.monotonic() < deadline:
            time.sleep(0.05)
        if len(frames) < start + 3:
            raise RuntimeError('respawn did not resume observations')
        report_cloud('respawn_mount', frames[start:], point_cloud2, results_path)

        # Two solid wall segments leave an opening; another wall is behind it.
        # Rays through the opening must see the rear wall. Rays aimed into a
        # solid segment must never report the wall behind it.
        spawn_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        spawn_box(spawn_model, Pose, 'opening_left', 2.05, 1.25, 0.1, 1.5)
        spawn_box(spawn_model, Pose, 'opening_right', 2.05, -1.25, 0.1, 1.5)
        spawn_box(spawn_model, Pose, 'rear_wall', 3.05, 0, 0.1, 4)

        start = len(frames)
        deadline = time.monotonic() + 5
        while len(frames) < start + 5 and time.monotonic() < deadline:
            time.sleep(0.05)
        if len(frames) < start + 5:
            raise RuntimeError('no cloud after obstacle insertion')

        through = 0
        blocked = 0
        leaked = 0
        for message in frames[start + 2:]:
            for x, y, z in point_cloud2.read_points(
                message,
                field_names=('x', 'y', 'z'),
            ):
                if x <= 0.31:
                    continue
                # The sensor's world position after respawn is (.3,-.2,1.4).
                t = (2 - 0.3) / (x - 0.3)
                hit_y = -0.2 + t * (y + 0.2)
                hit_z = 1.4 + t * (z - 1.4)
                if not 0.2 < hit_z < 3.8:
                    continue
                if abs(hit_y) < 0.35 and abs(x - 3) < 0.03:
                    through += 1
                if 0.65 < abs(hit_y) < 1.8:
                    if abs(x - 2) < 0.03:
                        blocked += 1
                    if x > 2.13:
                        leaked += 1

        opening_result = {
            'phase': 'opening_occlusion',
            'through_opening': through,
            'solid_returns': blocked,
            'penetrating_returns': leaked,
        }
        write_phase(results_path, opening_result)
        if not through or not blocked or leaked:
            raise RuntimeError('opening or occlusion failed')

        # A thin foreground object must appear at its new position after a
        # scene edit, and must stop occluding its former position.
        spawn_box(spawn_model, Pose, 'thin_pole', 1, 1, 0.08, 0.08)
        pole = ModelState()
        pole.model_name = 'thin_pole'
        pole.reference_frame = 'world'
        pole.pose.position.x = 1
        pole.pose.position.z = 2
        pole.pose.orientation.w = 1
        for pole_y in (1, -1):
            pole.pose.position.y = pole_y
            response = set_state(pole)
            if not response.success:
                raise RuntimeError(response.status_message)
            start = len(frames)
            deadline = time.monotonic() + 5
            while len(frames) < start + 5 and time.monotonic() < deadline:
                time.sleep(0.05)
            hits = old_hits = 0
            for message in frames[start + 2:]:
                for x, y, z in point_cloud2.read_points(message, field_names=('x', 'y', 'z')):
                    if abs(x - 1) < 0.05 and 0.2 < z < 3.8:
                        hits += abs(y - pole_y) < 0.05
                        old_hits += abs(y + pole_y) < 0.05
            write_phase(results_path, {
                'phase': 'thin_obstacle_update', 'position_y': pole_y,
                'surface_returns': hits, 'obsolete_returns': old_hits,
            })
            if not hits or old_hits:
                raise RuntimeError('thin obstacle observation did not follow scene updates')
        if not delete_model('thin_pole').success:
            raise RuntimeError('could not remove the thin obstacle')
        subscriber.unregister()

        if args.benchmark_sensors:
            metrics = {}
            subscribers = []

            def subscribe(name, topic):
                subscribe_sensor(
                    rospy,
                    PointCloud2,
                    metrics,
                    subscribers,
                    name,
                    topic,
                )

            subscribe('scanner', '/test_robot/simple_lidar/points')
            expected_rate = float(sdf.find('model/link/sensor/update_rate').text)
            benchmark_sensor_load(metrics, processes[-1][0], results_path, expected_rate)

            sensor_sdfs = {}
            for index in range(1, args.benchmark_sensors):
                name = 'scanner_' + str(index)
                instance = make_sensor_instance(sdf, name)
                instance_xml = ET.tostring(instance, encoding='unicode')
                sensor_sdfs[name] = instance_xml
                pose = spawn_pose(Pose)
                result = spawn_model(name, instance_xml, '', pose, 'world')
                if not result.success:
                    raise RuntimeError(result.status_message)
                subscribe(name, '/' + name + '/simple_lidar/points')

            if args.benchmark_sensors > 1:
                benchmark_sensor_load(metrics, processes[-1][0], results_path, expected_rate)
                for _ in range(args.rebuild_cycles):
                    check_middle_sensor_rebuild(
                        args.benchmark_sensors,
                        sensor_sdfs,
                        metrics,
                        Pose,
                        spawn_model,
                        rospy.ServiceProxy('/gazebo/delete_model', DeleteModel),
                        results_path,
                    )

            for item in subscribers:
                item.unregister()
        print('PASS', flush=True)
    finally:
        stop_processes(processes)


def spawn_box(spawn_model, pose_type, name, x, y, size_x, size_y):
    model_xml = (
        '<sdf version="1.6"><model name="{name}"><static>true</static>'
        '<link name="body"><collision name="c"><geometry><box><size>'
        '{size_x} {size_y} 4</size></box></geometry></collision>'
        '<visual name="v"><geometry><box><size>'
        '{size_x} {size_y} 4</size></box></geometry></visual></link>'
        '</model></sdf>'
    ).format(name=name, size_x=size_x, size_y=size_y)
    pose = pose_type()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 2
    pose.orientation.w = 1
    result = spawn_model(name, model_xml, '', pose, 'world')
    if not result.success:
        raise RuntimeError(result.status_message)


def main(argv=None):
    args = parse_args(argv)
    run_check(args)


if __name__ == '__main__':
    main()
