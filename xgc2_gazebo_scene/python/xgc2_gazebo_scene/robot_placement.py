"""Frozen placement input shared by product-owned native model renderers."""
import json
import math
import re
import sys
import xml.etree.ElementTree as ET


def read_placement(stream=None):
    raw = (stream or sys.stdin.buffer).read(65537)
    if len(raw) > 65536:
        raise ValueError('placement input exceeds 64 KiB')
    value = json.loads(raw)
    if not isinstance(value, dict) or set(value) != {'robot', 'runMode'}:
        raise ValueError('placement input requires {robot, runMode}')
    row, mode = value['robot'], value['runMode']
    if not isinstance(row, dict) or mode not in ('simulation', 'hybrid'):
        raise ValueError('placement requires a simulated robot')
    if mode == 'hybrid' and row.get('binding', {}).get('hybridSource') != 'simulation':
        raise ValueError('physical hybrid members cannot be placed')
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,63}', row.get('slotId', '')):
        raise ValueError('invalid frozen robot identity')
    if not re.fullmatch(r'/[A-Za-z][A-Za-z0-9_]*(?:/[A-Za-z][A-Za-z0-9_]*)*', row.get('namespace', '')):
        raise ValueError('invalid frozen robot namespace')
    for axis in ('x', 'y', 'z', 'yaw'):
        coordinate = row.get('binding', {}).get('initialPose', {}).get(axis)
        if isinstance(coordinate, bool) or not isinstance(coordinate, (int, float)) or not math.isfinite(coordinate):
            raise ValueError('initialPose.' + axis + ' must be finite')
    return row, mode


def ground_model(row, mode, kind, model, model_format):
    if row.get('kind') != kind:
        raise ValueError('robot kind differs from placement product')
    # Sensor lowering stays with the sensor product; model ownership stays here.
    from xgc2_simple_lidar.configuration import surface_parameters
    import xacro
    namespace = row['namespace'].lstrip('/')
    parameters = surface_parameters(row['binding'].get('simulationSensors') or {})
    parameters.update(ns=namespace, model_name=row['slotId'], chassis_robot_id=row['slotId'],
                      robot_namespace=namespace, robot_description_param=row['namespace']+'/robot_description',
                      tf_prefix=namespace, frame_prefix=namespace+'/', sensor_ns=namespace,
                      robot_state_publisher_ns=namespace, run_mode=mode,
                      **row['binding']['initialPose'])
    if kind == 'scout_mini':
        scout = row['binding'].get('scout', {})
        parameters.update(urdf_extras=str(model.parent/'scout_dynamics_plugins.xacro'), enable_ros_control=False,
                          enable_lidar=scout.get('lidarSimulationEnabled', False),
                          enable_camera=scout.get('imageSimulationEnabled', False))
    mappings = {key: str(value).lower() if isinstance(value, bool) else str(value)
                for key, value in parameters.items()}
    document = ET.fromstring(xacro.process_file(str(model), mappings=mappings).toxml())
    entity = document if model_format == 'urdf' else document.find('model')
    if entity is None:
        raise ValueError('product renderer did not produce a native model')
    entity.set('name', row['slotId'])
    return {'format': model_format, 'content': ET.tostring(document, encoding='unicode')}
