"""Explicitly bound simulation-v1 client; completion belongs to the world."""

import time
import uuid

from xgc2_xrpc.http import Client, Fault, Limits, TransportError
from xgc2_xrpc.reference import ServiceRef


class SimulationError(ValueError):
    """A simulation operation failed or did not commit the requested result."""


class SimulationClient:
    def __init__(self, reference, *, runtime, local_target, timeout=30.0, owns_runtime=False):
        if not isinstance(reference, ServiceRef):
            reference = ServiceRef.from_dict(reference)
        if reference.service != 'xgc2.simulation' or reference.api_version != 'v1':
            raise SimulationError('An explicit simulation-v1 ServiceRef is required')
        reference.validate()
        self.reference = reference
        self.timeout = timeout
        self._owned_runtime = runtime if owns_runtime else None
        self.client = Client.from_service(reference, runtime=runtime, local_target=local_target,
                                          limits=Limits(body_bytes=8*1024*1024,
                                                        response_bytes=8*1024*1024,
                                                        call_timeout=timeout,
                                                        shutdown_timeout=1 if owns_runtime else 5))

    def operation(self, route, value, *, method='POST', request_id=None, deadline=None):
        """Submit once, then one notification-driven wait. Never replay writes."""
        request_id = request_id or uuid.uuid4().hex
        if deadline is None:
            deadline = time.monotonic() + self.timeout
        payload = dict(value, operation_timeout_ms=max(1, int(self.timeout*1000)))
        try:
            remaining = deadline-time.monotonic()
            if remaining <= 0:
                raise SimulationError('Simulation operation {} deadline exceeded'.format(request_id))
            result = self.client.json(route, payload, method=method, request_id=request_id,
                                      timeout=remaining)
            if result.get('id') != request_id:
                raise SimulationError('Simulation returned a different operation identity')
            if result.get('state') in ('accepted', 'running'):
                remaining = deadline-time.monotonic()
                if remaining <= 0:
                    raise SimulationError('Simulation operation {} deadline exceeded'.format(request_id))
                result = self.client.json('/v1/operations/{}/wait'.format(request_id), {},
                                          timeout=remaining)
            if result.get('id') != request_id or result.get('state') != 'succeeded':
                raise SimulationError('Simulation operation {} did not complete: {}'.format(
                    request_id, result.get('error') or result.get('state')))
            return result.get('result', {})
        except TransportError as error:
            raise SimulationError('Simulation operation {}: {} ({})'.format(
                request_id, error, error.disposition)) from error
        except Fault as error:
            raise SimulationError('Simulation operation {}: {} ({})'.format(
                request_id, error, error.code)) from error

    def apply(self, document, epoch, revision):
        result = self.operation('/v1/extensions/scene/apply',
                                {'document': document, 'epoch': epoch, 'revision': revision})
        if result.get('epoch') != epoch or result.get('revision') != revision:
            raise SimulationError('Simulation did not commit the requested scene revision')
        return result

    def motion(self, operation, epoch, revision):
        return self.operation('/v1/extensions/scene/motion',
                              {'operation': operation, 'epoch': epoch, 'revision': revision})

    def close(self):
        try:
            self.client.close()
        finally:
            if self._owned_runtime is not None:
                self._owned_runtime.close()
                self._owned_runtime = None
