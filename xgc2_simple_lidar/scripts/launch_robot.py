#!/usr/bin/env python3
"""Consume authored sensor input, then replace this process with ROS launch."""
import json
import os
import sys
import argparse
sys.dont_write_bytecode = True
from xgc2_simple_lidar.configuration import robot_parameters


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=('scout', 'mecanum'), required=True)
    parser.add_argument('--namespace', required=True)
    parser.add_argument('--target-id', required=True)
    parser.add_argument('launch_file')
    args, launch_args = parser.parse_known_args()
    raw = sys.stdin.buffer.read(65537)
    if len(raw) > 65536:
        raise ValueError('robot input exceeds 64 KiB')
    launch_input = json.loads(raw)
    if not isinstance(launch_input, dict) or set(launch_input) != {'robot', 'simulationServiceRef'}:
        raise ValueError('launch input requires robot and simulationServiceRef')
    service_ref = launch_input['simulationServiceRef']
    if not isinstance(service_ref, dict) or not service_ref:
        raise ValueError('simulationServiceRef must be a nonempty object')
    parameters = robot_parameters(launch_input['robot'], args.profile, args.namespace)
    parameters.update(simulation_service_ref_json=json.dumps(service_ref, separators=(',', ':')),
                      target_id=args.target_id)
    if any(arg.partition(':=')[0] in parameters for arg in launch_args):
        raise ValueError('launch arguments override frozen robot input')
    arguments = ['/opt/ros/noetic/bin/roslaunch', args.launch_file, *launch_args]
    arguments.extend(key + ':=' + (str(value).lower() if isinstance(value, bool) else str(value))
                     for key, value in parameters.items())
    os.execv('/opt/ros/noetic/bin/roslaunch', arguments)


if __name__ == '__main__':
    main()
