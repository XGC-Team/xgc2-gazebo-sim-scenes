#!/usr/bin/env python3
"""Render an installed native model from one frozen robot row, without ROS I/O."""
import argparse
import copy
import subprocess
import json
import os
from pathlib import Path
import sys

sys.path.append('/opt/ros/noetic/lib/python3/dist-packages')

from xgc2_simple_lidar.configuration import robot_parameters


def prepare(row, run_mode, share=Path('/opt/ros/noetic/share'), point_cloud=False):
    os.environ.setdefault('ROS_PACKAGE_PATH', str(share))
    if run_mode not in ('simulation', 'hybrid') or (run_mode == 'hybrid' and
            row.get('binding', {}).get('hybridSource') != 'simulation'):
        raise ValueError('only a simulated robot can be prepared for Gazebo')

    if point_cloud:
        row = copy.deepcopy(row)
        sensors = row.setdefault('binding', {}).setdefault('simulationSensors', {})
        sensors['simpleLidar'] = {'enabled': False}
    if row.get('kind') == 'px4_multirotor':
        sys.path.append('/opt/ros/noetic/lib/gazebo_sim_fs150_sitl')
        from render_fs150_indoor_sdf import prepare_frozen_model, resolve_base_sdf
        return prepare_frozen_model(row, resolve_base_sdf(None))

    import xacro

    profile = {'scout_mini': 'scout', 'mecanum_ugv': 'mecanum'}.get(row.get('kind'))
    if profile is None:
        raise ValueError('native model preparation does not support robot kind ' + str(row.get('kind')))
    parameters = robot_parameters(row, profile, run_mode)
    parameters['chassis_robot_id'] = row['slotId']
    if profile == 'scout':
        model = share / 'gazebo_sim_scout/urdf/mini.xacro'
        parameters.update(urdf_extras=str(model.parent / 'scout_dynamics_plugins.xacro'),
                          enable_ros_control=False)
        model_format = 'urdf'
    else:
        model = share / 'gazebo_sim_mecanum/models/xgc2_mecanum_ugv/model.sdf.xacro'
        parameters.update(robot_namespace=row['namespace'].lstrip('/'))
        model_format = 'sdf'
    mappings = {key: str(value).lower() if isinstance(value, bool) else str(value)
                for key, value in parameters.items()}
    return {'format': model_format, 'content': xacro.process_file(str(model), mappings=mappings).toxml()}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run-mode', required=True)
    parser.add_argument('--scene-json', required=True)
    parser.add_argument('--resource-root', required=True)
    parser.add_argument('--workspace-root', required=True)
    args = parser.parse_args()
    try:
        raw = sys.stdin.buffer.read(65537)
        if len(raw) > 65536:
            raise ValueError('robot input exceeds 64 KiB')
        scene = json.loads(args.scene_json)
        point_cloud = False
        if scene.get('asset'):
            selected = json.loads(subprocess.check_output([
                '/opt/ros/noetic/lib/xgc_scene_ros1_adapter/xgc2-scene-prepare',
                '--context-json', json.dumps({'scene': scene}),
                '--resource-root', args.resource_root,
                '--workspace-root', args.workspace_root], text=True))
            point_cloud = 'pointCloud' in selected
        print(json.dumps(prepare(json.loads(raw), args.run_mode, point_cloud=point_cloud), separators=(',', ':')))
    except Exception as error:
        parser.exit(1, 'prepare_robot_model: ' + ' '.join(str(error).splitlines()) + '\n')


if __name__ == '__main__':
    main()
