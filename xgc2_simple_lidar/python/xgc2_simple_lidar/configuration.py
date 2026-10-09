"""Interpret authored observations at the sensor's native boundary."""
import math
import re


def robot_parameters(robot, profile, namespace):
    """Lower one frozen robot at the native launch boundary."""
    if not isinstance(robot, dict) or robot.get('namespace') != namespace:
        raise ValueError('robot namespace differs from the claimed namespace')
    if not re.fullmatch(r'/[A-Za-z][A-Za-z0-9_]{0,127}', namespace):
        raise ValueError('robot requires one absolute ROS namespace segment')
    name = namespace[1:]
    if robot.get('kind') != {'scout': 'scout_mini', 'mecanum': 'mecanum_ugv'}[profile]:
        raise ValueError('robot kind differs from the native launch profile')
    parameters = surface_parameters(robot['authoredSimulationSensors'])
    parameters.update(ns=name, model_name=name)
    for key in ('x', 'y', 'z', 'yaw'):
        value = robot['initialPose'][key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('robot initial pose must be finite')
        parameters[key] = value
    if profile == 'scout':
        scout = robot['scout']
        parameters.update(robot_description_param=namespace+'/robot_description',
                          tf_prefix=name, frame_prefix=name+'/', sensor_ns=name,
                          robot_state_publisher_ns=name, run_mode=robot['runMode'])
        for source, target in (('lidarSimulationEnabled', 'enable_lidar'),
                               ('imageSimulationEnabled', 'enable_camera')):
            value = scout[source]
            if not isinstance(value, bool):
                raise ValueError('Scout sensor switches must be boolean')
            parameters[target] = value
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
    if lidar.keys() - {'enabled', 'mode', 'preset', 'acceleration', 'rateHz', 'rangeMeters',
                       'hFovDeg', 'vFovDeg', 'hRes', 'vRes', 'publishBeams'}:
        raise ValueError('unknown simpleLidar field')
    if not isinstance(lidar.get('publishBeams', False), bool):
        raise ValueError('simpleLidar.publishBeams must be boolean')
    mode, preset = lidar.get('mode', ''), lidar.get('preset', '')
    if mode not in ('', 'raycast', 'penetrating', 'depth_frustum'):
        raise ValueError('invalid simpleLidar mode')
    if preset not in ('', 'bridge_equivalent', 'zju_cpu_crop', 'zju_cpu_crop_ego_v2'):
        raise ValueError('invalid simpleLidar preset')
    sampled = (mode in ('penetrating', 'depth_frustum') or lidar.get('publishBeams', False)
               or (preset and mode != 'raycast'))
    result = {'enable_simple_lidar': lidar.get('enabled', False) and not sampled,
              'simple_lidar_acceleration': lidar.get('acceleration') or 'gpu'}
    if result['simple_lidar_acceleration'] not in ('cpu', 'gpu'):
        raise ValueError('invalid simpleLidar acceleration')
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
                or (integral and (not isinstance(value, int) or (value == 1 and mode != 'penetrating')))):
            raise ValueError('invalid simpleLidar.' + field)
        result['simple_lidar_' + target] = value or default
    return result
