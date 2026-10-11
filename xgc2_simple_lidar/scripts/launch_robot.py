#!/usr/bin/env python3
"""Lower one Scout robot row, then replace this process with its runtime launch."""
import json
import os
import sys
import argparse
sys.dont_write_bytecode = True
from xgc2_simple_lidar.configuration import robot_parameters


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--profile', choices=('scout',), required=True)
    parser.add_argument('--namespace', required=True)
    parser.add_argument('launch_file')
    args, launch_args = parser.parse_known_args()
    if any(':=' not in arg or arg.startswith('-') for arg in launch_args):
        parser.error('additional arguments must be launch name:=value assignments')
    raw = sys.stdin.buffer.read(65537)
    if len(raw) > 65536:
        raise ValueError('robot input exceeds 64 KiB')
    launch_input = json.loads(raw)
    if not isinstance(launch_input, dict) or set(launch_input) != {'robot'}:
        raise ValueError('launch input requires only robot')
    parameters = robot_parameters(launch_input['robot'], args.profile, args.namespace)
    if any(arg.partition(':=')[0] in parameters for arg in launch_args):
        raise ValueError('launch arguments override frozen robot input')
    # --wait: with parallel starts the session master may not be up yet, and roslaunch would start its own.
    arguments = ['/opt/ros/noetic/bin/roslaunch', '--wait', args.launch_file, *launch_args]
    arguments.extend(key + ':=' + (str(value).lower() if isinstance(value, bool) else str(value))
                     for key, value in parameters.items())
    os.execv('/opt/ros/noetic/bin/roslaunch', arguments)


if __name__ == '__main__':
    main()
