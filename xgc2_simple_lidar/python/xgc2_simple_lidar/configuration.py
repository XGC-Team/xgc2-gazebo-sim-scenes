"""Interpret authored observations at the sensor's native boundary."""
import math


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
