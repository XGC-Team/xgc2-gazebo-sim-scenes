"""Interpret authored observations at the sensor's native boundary."""
import math
import re
import functools
import subprocess
import os
import socket
import tempfile



@functools.lru_cache(maxsize=1)
def gpu_available():
    """Probe the installed native renderer, never infer GPU support from a label."""
    try:
        with tempfile.TemporaryDirectory(prefix='gazebo-ray-probe-') as directory, socket.socket() as port:
            port.bind(('127.0.0.1', 0))
            master_port = port.getsockname()[1]
            port.close()
            environment = dict(os.environ, HOME=directory,
                               GAZEBO_MASTER_URI=f'http://127.0.0.1:{master_port}',
                               GAZEBO_MODEL_DATABASE_URI='')
            return subprocess.run(['xgc2-gazebo-ray-probe'], stdout=subprocess.DEVNULL,
                                  stderr=subprocess.DEVNULL, timeout=10, env=environment).returncode == 0
    except (OSError, subprocess.TimeoutExpired):
        return False

def robot_parameters(row, profile, run_mode):
    """Lower one frozen robot at the native launch boundary."""
    if not isinstance(row, dict):
        raise ValueError('robot must be a freezer row object')
    namespace = row.get('namespace')
    if not isinstance(namespace, str) or not re.fullmatch(r'/[A-Za-z][A-Za-z0-9_]{0,127}', namespace):
        raise ValueError('robot.namespace requires one absolute ROS namespace segment')
    identity = row.get('slotId')
    if not isinstance(identity, str) or not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,63}', identity):
        raise ValueError('robot.slotId requires an explicit chassis identity')
    name = namespace[1:]
    if row.get('kind') != {'scout': 'scout_mini', 'mecanum': 'mecanum_ugv'}[profile]:
        raise ValueError('robot kind differs from the native launch profile')
    binding = row.get('binding')
    if not isinstance(binding, dict):
        raise ValueError('robot.binding must be an object')
    pose = binding.get('initialPose')
    if not isinstance(pose, dict):
        raise ValueError('robot.binding.initialPose must be an object')
    parameters = surface_parameters(binding.get('simulationSensors') or {})
    parameters.update(ns=name, model_name=identity)
    for key in ('x', 'y', 'z', 'yaw'):
        value = pose.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('robot.binding.initialPose.' + key + ' must be finite')
        parameters[key] = value
    if profile == 'scout':
        scout = binding.get('scout', {})
        if not isinstance(scout, dict):
            raise ValueError('robot.binding.scout must be an object')
        parameters.update(robot_description_param=namespace+'/robot_description',
                          tf_prefix=name, frame_prefix=name+'/', sensor_ns=name,
                          robot_state_publisher_ns=name, run_mode=run_mode)
    return parameters


def surface_parameters(sensors):
    if not isinstance(sensors, dict):
        raise ValueError('simulation sensors must be an object')
    if sensors.keys() - {'simpleLidar'}:
        raise ValueError('unknown simulation sensor')
    authored = sensors.get('simpleLidar', False)
    lidar = {'enabled': authored} if isinstance(authored, bool) else authored
    if not isinstance(lidar, dict) or not isinstance(lidar.get('enabled', False), bool):
        raise ValueError('simpleLidar must be a boolean or sensor configuration')
    if lidar.keys() - {'enabled', 'bodyReturns', 'mode', 'preset', 'acceleration', 'rateHz', 'rangeMeters',
                       'hFovDeg', 'vFovDeg', 'hRes', 'vRes', 'publishBeams'}:
        raise ValueError('unknown simpleLidar field')
    if lidar.get('mode', '') not in ('', 'raycast'):
        raise ValueError('Gazebo simpleLidar.mode supports raycast only')
    if lidar.get('preset', '') != '':
        raise ValueError('Gazebo simpleLidar.preset must be empty')
    for field in ('publishBeams', 'bodyReturns'):
        if not isinstance(lidar.get(field, False), bool) or lidar.get(field, False):
            raise ValueError('Gazebo simpleLidar.' + field + ' is unsupported')
    acceleration = lidar.get('acceleration', '')
    if not isinstance(acceleration, str) or acceleration not in ('', 'auto', 'cpu', 'gpu'):
        raise ValueError('simpleLidar.acceleration must be a string: empty, auto, cpu or gpu')
    acceleration = acceleration or 'auto'
    if acceleration == 'auto':
        acceleration = ('gpu' if gpu_available() else 'cpu') if lidar.get('enabled', False) else 'gpu'
    result = {'enable_simple_lidar': lidar.get('enabled', False),
              'simple_lidar_acceleration': acceleration}
    for field, target, default, maximum, integral in (
            ('rateHz', 'rate_hz', 10, 100, False),
            ('rangeMeters', 'range_meters', 20, 200, False),
            ('hFovDeg', 'hfov_deg', 360, 360, False),
            ('vFovDeg', 'vfov_deg', 180/math.pi, 180, False),
            ('hRes', 'hres', 360, 4096, True),
            ('vRes', 'vres', 16, 4096, True)):
        value = lidar.get(field, 0)
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or not 0 <= value <= maximum
                or (integral and (not isinstance(value, int) or value == 1))):
            raise ValueError('invalid simpleLidar.' + field)
        result['simple_lidar_' + target] = value or default
    return result
