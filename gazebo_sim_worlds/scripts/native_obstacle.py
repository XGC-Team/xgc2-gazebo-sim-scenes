#!/usr/bin/env python3
"""Run a bounded obstacle plan through the current native world ServiceRef."""
import argparse
import json
import math
import re
import sys
import time
import uuid
from pathlib import Path
from xgc2_xrpc.http import Client, Limits
from xgc2_xrpc.reference import ServiceRef
from xgc2_xrpc.runtime import Runtime

PREFIX = 'xgc2_obstacle_'


def identity(name):
    if not isinstance(name, str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]{0,63}', name):
        raise ValueError('Obstacle name must be bounded ASCII')
    return PREFIX+name


def native_pose(value):
    x,y,z,roll,pitch,yaw = [float(value[k]) for k in ('x','y','z','roll','pitch','yaw')]
    if not all(math.isfinite(v) for v in (x,y,z,roll,pitch,yaw)):
        raise ValueError('Obstacle pose must be finite')
    sr,cr,sp,cp,sy,cy = math.sin(roll/2),math.cos(roll/2),math.sin(pitch/2),math.cos(pitch/2),math.sin(yaw/2),math.cos(yaw/2)
    return {'position':[x,y,z], 'orientation':[sr*cp*cy-cr*sp*sy,cr*sp*cy+sr*cp*sy,cr*cp*sy-sr*sp*cy,cr*cp*cy+sr*sp*sy]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--service-ref-json', required=True)
    parser.add_argument('--plan-json', required=True)
    args = parser.parse_args()
    ref = ServiceRef.from_dict(json.loads(args.service_ref_json)).validate()
    if ref.service != 'xgc2.simulation' or ref.api_version != 'v1':
        raise ValueError('Actual simulation ServiceRef required')
    if len(args.plan_json.encode()) > 65536:
        raise ValueError('Obstacle plan exceeds 64 KiB')
    plan = json.loads(args.plan_json)
    if not isinstance(plan, list) or not 0 < len(plan) <= 32:
        raise ValueError('Obstacle plan requires 1 to 32 operations')
    runtime = Runtime(blocking_workers=1, max_connections=1)
    client = Client.from_service(ref, runtime=runtime, local_target=ref.target_id,
                                 limits=Limits(call_timeout=35, body_bytes=1048576, response_bytes=1048576, connections=1))
    deadline = time.monotonic()+120
    count = 0
    def call(path, payload=None, method='POST'):
        timeout = min(35, deadline-time.monotonic())
        if timeout <= 0: raise TimeoutError('Obstacle plan budget exhausted')
        return client.json(path, payload, method=method, timeout=timeout, request_id=uuid.uuid4().hex)
    def apply(path, payload, method='POST'):
        accepted = call(path, payload, method)
        terminal = call('/v1/operations/'+accepted['id']+'/wait', {})
        if terminal['state'] != 'succeeded':
            raise RuntimeError('Native obstacle operation ended '+terminal['state']+': '+json.dumps(terminal.get('error', {})))
    def entity(name):
        result = call('/v1/entities/'+identity(name), method='GET')['entities']
        if len(result) != 1 or result[0]['ref']['id'] != identity(name):
            raise ValueError('World returned a different obstacle EntityRef')
        return result[0]['ref']
    try:
        for item in plan:
            op = item['operation']
            if op == 'clear':
                for row in call('/v1/entities', method='GET')['entities']:
                    actual = row['ref']
                    if row['role'] == 'obstacle' and actual['id'].startswith(PREFIX):
                        apply('/v1/entities/'+actual['id'], {'generation':actual['generation'], 'operation_timeout_ms':30000}, 'DELETE')
            elif op == 'spawn':
                model = item['model']
                if not isinstance(model, str) or not re.fullmatch(r'xgc2_geom_[a-z0-9_]{1,48}', model):
                    raise ValueError('A packaged obstacle model is required')
                path = Path('/opt/ros/noetic/share/gazebo_sim_worlds/models')/model/'model.sdf'
                if path.stat().st_size > 1048576: raise ValueError('Obstacle asset exceeds 1 MiB')
                apply('/v1/entities', {'entity':{'id':identity(item['name']), 'role':'obstacle',
                      'asset':{'id':model,'realization':{'media_type':'application/sdf+xml','content':path.read_text()}},
                      'pose':native_pose(item['pose'])}, 'operation_timeout_ms':30000})
            elif op == 'move':
                actual = entity(item['name'])
                apply('/v1/entities/'+actual['id']+'/state', {'generation':actual['generation'],
                      'state':{'pose':native_pose(item['pose'])}, 'operation_timeout_ms':30000})
            elif op == 'motion':
                apply('/v1/extensions/entities/motion', {'entities':[{'ref':entity(item['name']), 'motion':item['motion']}],
                      'operation_timeout_ms':30000})
            else:
                raise ValueError('Unsupported native obstacle plan operation')
            count += 1
        print(json.dumps({'completedOperations':count, 'service':ref.instance_id}, separators=(',', ':')))
    finally:
        client.close()
        runtime.close()


if __name__ == '__main__':
    try: main()
    except Exception as error:
        print('native_obstacle: '+str(error), file=sys.stderr)
        sys.exit(1)
