#!/usr/bin/env python3
"""Consume authored sensor input, then replace this process with ROS launch."""
import json
import os
import sys
sys.dont_write_bytecode = True
from xgc2_simple_lidar.configuration import surface_parameters


def main():
    if len(sys.argv) < 2:
        raise ValueError('a robot launch file is required')
    raw = sys.stdin.buffer.read(65537)
    if len(raw) > 65536:
        raise ValueError('sensor input exceeds 64 KiB')
    parameters = surface_parameters(json.loads(raw))
    arguments = ['/opt/ros/noetic/bin/roslaunch', *sys.argv[1:]]
    arguments.extend(key + ':=' + (str(value).lower() if isinstance(value, bool) else str(value))
                     for key, value in parameters.items())
    os.execv('/opt/ros/noetic/bin/roslaunch', arguments)


if __name__ == '__main__':
    main()
