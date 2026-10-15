#!/usr/bin/env python3
"""Execute the selected Gazebo GUI with transport scoped to its master."""
import ipaddress
import os
import sys
from urllib.parse import urlsplit


def compose(arguments, environment):
    env = dict(environment)
    try:
        host = urlsplit(env.get('GAZEBO_MASTER_URI', 'http://localhost:11345')).hostname
        local = host == 'localhost'
        if host and not local:
            try:
                local = ipaddress.ip_address(host).is_loopback
            except ValueError:
                local = False
    except ValueError:
        local = False
    if local:
        env['IGN_IP'] = '127.0.0.1'
    return ['gzclient'] + list(arguments), env


def main(arguments=None):
    argv, environment = compose(sys.argv[1:] if arguments is None else arguments, os.environ)
    try:
        os.execvpe(argv[0], argv, environment)
    except OSError as error:
        print('native_gazebo_client: ' + str(error), file=sys.stderr)
        return 2


if __name__ == '__main__':
    sys.exit(main())
