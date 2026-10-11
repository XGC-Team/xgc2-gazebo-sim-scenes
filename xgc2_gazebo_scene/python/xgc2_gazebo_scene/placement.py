"""Finite entity placement through one discovered simulation-v1 world."""

import ctypes
import math
import os
import select
import time

from xgc2_xrpc import Client, Limits, Runtime, ServiceRef

from .simulation_client import SimulationClient, SimulationError


def _remaining(deadline):
    remaining = deadline - time.monotonic()
    if remaining <= 0:
        raise SimulationError('World placement deadline exceeded')
    return remaining


def _wait_for_socket(endpoint, deadline):
    if os.path.exists(endpoint):
        return
    libc = ctypes.CDLL(None, use_errno=True)
    libc.inotify_init1.argtypes = [ctypes.c_int]
    libc.inotify_add_watch.argtypes = [ctypes.c_int, ctypes.c_char_p, ctypes.c_uint32]
    events = libc.inotify_init1(os.O_NONBLOCK | os.O_CLOEXEC)
    if events < 0:
        raise OSError(ctypes.get_errno(), 'World socket watch failed')
    try:
        # Core creates the rendezvous directory before starting the world.
        if libc.inotify_add_watch(events, os.fsencode(os.path.dirname(endpoint)),
                                  0x100 | 0x80) < 0:  # IN_CREATE | IN_MOVED_TO
            raise OSError(ctypes.get_errno(), 'World socket directory watch failed')
        # Check after arming the watch so an appearance cannot be missed.
        while not os.path.exists(endpoint):
            readable, _, _ = select.select([events], [], [], _remaining(deadline))
            if not readable:
                raise SimulationError('World socket did not appear before the placement deadline')
            os.read(events, 65536)
    finally:
        os.close(events)


def discover_world(endpoint, target_id, timeout):
    """Return a bound client owning its runtime; the caller must close it.

    Socket appearance and one unbound held describe share the timeout budget.
    Readiness must be returned by that call; discovery is never re-armed.
    """
    if (not isinstance(endpoint, str) or not os.path.isabs(endpoint)
            or os.path.normpath(endpoint) != endpoint or '\0' in endpoint
            or len(os.fsencode(endpoint)) > 107):
        raise ValueError('World endpoint must be a canonical absolute Unix socket path')
    if (not isinstance(target_id, str) or not target_id or target_id.strip() != target_id
            or any(ord(character) < 32 for character in target_id)):
        raise ValueError('Target ID must be a nonempty canonical string')
    if (isinstance(timeout, bool) or not isinstance(timeout, (int, float))
            or not math.isfinite(timeout) or not 0 < timeout <= 55):
        raise ValueError('Placement timeout must be in (0, 55] seconds')
    deadline = time.monotonic() + timeout
    _wait_for_socket(endpoint, deadline)
    runtime = Runtime(blocking_workers=1, max_calls=2, max_connections=2,
                      max_sessions=2, shutdown_timeout=1, log_level='error')
    try:
        discovery = Client(endpoint, runtime=runtime,
                           limits=Limits(call_timeout=_remaining(deadline), shutdown_timeout=1))
        try:
            remaining = _remaining(deadline)
            description = discovery.json('/v1/describe?wait_ready_ms={}'.format(
                min(30000, max(1, int(remaining * 1000)))), method='GET', timeout=remaining)
            reference = ServiceRef.from_dict(description.get('facts', {}).get(
                'service_ref', description.get('service_ref'))).validate()
            if (description.get('service') != 'xgc2.simulation'
                    or reference.service != 'xgc2.simulation'):
                raise SimulationError('World service must be xgc2.simulation')
            if description.get('api_version') != 'v1' or reference.api_version != 'v1':
                raise SimulationError('World API version must be v1')
            if reference.profile != 'http.v1' or reference.endpoint.kind != 'unix':
                raise SimulationError('World requires an http.v1 Unix endpoint')
            if reference.target_id != target_id:
                raise SimulationError('World target does not match target ID ' + target_id)
            if reference.endpoint.address != endpoint:
                raise SimulationError('World endpoint differs from the allocated endpoint')
            if description.get('instance_id') != reference.instance_id:
                raise SimulationError('World description and reference instance IDs differ')
            if description.get('ready') is not True:
                raise SimulationError('World did not become ready within the held describe deadline')
        finally:
            discovery.close()
        return SimulationClient(reference, runtime=runtime, local_target=target_id,
                                timeout=_remaining(deadline), owns_runtime=True)
    except BaseException:
        runtime.close()
        raise


def place_entity(endpoint, target_id, entity, timeout):
    """Return (native result dict, world instance ID), closing all resources.

    Discovery, one submission and at most one completion wait share a deadline.
    """
    deadline = time.monotonic() + timeout
    client = discover_world(endpoint, target_id, timeout)
    try:
        client.timeout = _remaining(deadline)
        result = client.operation('/v1/entities', {'entity': entity}, deadline=deadline)
        if not isinstance(result, dict):
            raise SimulationError('World creation result must be an object')
        return result, client.reference.instance_id
    finally:
        client.close()
