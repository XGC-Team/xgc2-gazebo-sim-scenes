#!/usr/bin/env python3
"""Real paused Gazebo factories with independently withheld native ACKs."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
from xml.sax.saxutils import escape

from xgc2_xrpc.http import Client
from xgc2_xrpc.runtime import Runtime


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--plugins", type=Path, required=True)
    parser.add_argument("--gzserver", required=True)
    parser.add_argument("--evidence", type=Path, required=True)
    args = parser.parse_args()
    artifacts = {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                 for path in args.plugins.rglob("libxgc2*.so")}
    checks = []
    with tempfile.TemporaryDirectory(prefix="create-ack-world-") as directory:
        root = Path(directory)
        root.chmod(0o700)
        endpoint = root / "world.sock"
        with socket.socket() as port:
            port.bind(("127.0.0.1", 0))
            master_port = port.getsockname()[1]
        world = root / "fixture.world"
        world.write_text(f'''<sdf version="1.6"><world name="isolated">
          <gravity>0 0 0</gravity>
          <plugin name="simulation" filename="libxgc2_simulation_world.so">
            <socket_path>{escape(str(endpoint))}</socket_path><target_id>create-fixture</target_id>
            <resource_root>{escape(str(root))}</resource_root>
          </plugin></world></sdf>''')
        environment = dict(os.environ)
        environment.pop("DISPLAY", None)
        environment.pop("WAYLAND_DISPLAY", None)
        environment.update(HOME=str(root), GAZEBO_MASTER_URI=f"http://127.0.0.1:{master_port}",
            GAZEBO_PLUGIN_PATH=str(args.plugins / "create_ack_fixture") + ":" + str(args.plugins),
            GAZEBO_MODEL_DATABASE_URI="", XGC2_CREATE_ACK_FIXTURE_ROOT=str(root))
        environment["LD_LIBRARY_PATH"] = str(args.plugins) + ":/opt/ros/noetic/lib:" + environment.get("LD_LIBRARY_PATH", "")
        runtime = Runtime(blocking_workers=1, max_calls=8)
        discovery = Client(str(endpoint), runtime=runtime)
        client = None
        log = args.evidence.with_suffix(".log").open("w")
        process = subprocess.Popen([args.gzserver, "--verbose", "--pause", str(world)],
            env=environment, stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 15
            while not endpoint.exists():
                assert process.poll() is None and time.monotonic() < deadline, "isolated world startup failed"
                time.sleep(.025)
            ref = json.loads(discovery.call("/v1/describe", method="GET").body)["facts"]["service_ref"]
            client = Client(str(endpoint), runtime=runtime, instance_id=ref["instance_id"])

            def query(route):
                return json.loads(client.call(route, method="GET", timeout=5).body)

            sequence = 0

            def mutate(route, body, *, method="POST"):
                nonlocal sequence
                sequence += 1
                operation = json.loads(client.call(route, {**body, "operation_timeout_ms": 5000},
                    method=method, request_id=f"create-check-{sequence}", timeout=5).body)
                return json.loads(client.call(f'/v1/operations/{operation["id"]}/wait', {}, timeout=5).body)

            pose = {"position": [1, 2, 3], "orientation": [0, 0, 0, 1]}
            artifact = '<sdf version="1.6"><model name="input"><link name="body"><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial></link></model></sdf>'
            entity = {"role": "robot", "pose": pose, "asset": {"id": "test-model", "realization": {
                "media_type": "application/sdf+xml", "content": artifact}}}

            def submit_held(name, timeout=5000):
                submitted = json.loads(client.call("/v1/entities", {
                    "entity": {**entity, "id": name}, "operation_timeout_ms": timeout},
                    request_id=name, timeout=5).body)
                assert submitted["state"] in ("accepted", "running"), submitted
                deadline = time.monotonic() + 5
                while not (root / (name + ".factory")).exists():
                    assert time.monotonic() < deadline, "held model never reached the native factory"
                    time.sleep(.01)
                assert query("/v1/operations/" + name)["state"] == "running"

            started = time.monotonic()
            submit_held("create-held", timeout=500)
            expired = json.loads(client.call("/v1/operations/create-held/wait", {}, timeout=2).body)
            assert expired["state"] == "failed" and expired["error"]["code"] == "deadline_exceeded", expired
            assert time.monotonic() - started < 2, "missing factory ACK did not settle within budget"
            submit_held("create-cancel-held")
            started = time.monotonic()
            client.call("/v1/operations/create-cancel-held/cancel", {})
            cancelled = json.loads(client.call("/v1/operations/create-cancel-held/wait", {}, timeout=2).body)
            assert cancelled["state"] == "cancelled", cancelled
            assert time.monotonic() - started < 2, "cancel waited for missing factory ACK"
            checks.append("missing-factory-ack-bounded-expiry-and-cancel")
            following_submission = json.loads(client.call("/v1/entities", {
                "entity": {**entity, "id": "create-following"}, "operation_timeout_ms": 5000},
                request_id="create-following", timeout=5).body)
            deadline = time.monotonic() + 5
            while not (root / "create-following.factory").exists():
                assert time.monotonic() < deadline, "following factory remained queued behind withheld ACK"
                time.sleep(.01)
            following = json.loads(client.call(
                f'/v1/operations/{following_submission["id"]}/wait', {}, timeout=5).body)
            assert following["state"] == "succeeded", following
            assert query("/v1/operations/create-held") == expired
            checks.append("following-create-completes-before-first-ack")
            duplicate = mutate("/v1/entities", {"entity": {**entity, "id": "create-held"}})
            assert duplicate["error"]["code"] == "conflict", duplicate
            assert all(item["ref"]["id"] != "create-held" for item in query("/v1/entities")["entities"])
            checks.append("pending-id-reserved-without-fabricated-ready")
            scene = {"schema": "xgc2.scene.v1", "id": "concurrent", "frame": "world",
                "obstacles": [{"id": "box", "name": "Box", "pose": pose, "motion": {"type": "hold"},
                    "parts": [{"id": "body", "pose": {"position": [0, 0, 0], "orientation": [0, 0, 0, 1]},
                        "color": [1, .5, .1, 1], "geometry": {"type": "box", "size": [1, 1, 1]}}]}]}
            scene_create = mutate("/v1/extensions/scene/apply", {
                "epoch": "concurrent-epoch", "revision": 1, "document": scene})
            assert scene_create["state"] == "succeeded", scene_create
            assert query("/v1/operations/create-held") == expired
            scene['obstacles'][0]['id'] = 'held'
            started = time.monotonic()
            admitted = json.loads(client.call("/v1/extensions/scene/apply", {
                "epoch": "concurrent-epoch", "revision": 2, "document": scene, "operation_timeout_ms": 200},
                request_id="scene-deadline", timeout=2).body)
            scene_expired = json.loads(client.call('/v1/operations/'+admitted['id']+'/wait', {}, timeout=2).body)
            assert (root / 'scene-held.factory').exists(), "scene factory did not reach withheld ACK"
            assert scene_expired['state'] == 'failed' and scene_expired['error']['code'] == 'deadline_exceeded', scene_expired
            assert time.monotonic() - started < 2, "missing scene factory ACK stalled worker"
            query('/v1/entities')
            checks.append('scene-factory-missing-ack-bounded-deadline')
            scene['obstacles'][0]['id'] = 'cancel'
            admitted = json.loads(client.call("/v1/extensions/scene/apply", {
                "epoch": "concurrent-epoch", "revision": 3, "document": scene, "operation_timeout_ms": 5000},
                request_id="scene-cancel", timeout=2).body)
            deadline = time.monotonic()+2
            while not (root/'scene-cancel.factory').exists():
                assert time.monotonic()<deadline, "cancelled scene never reached native factory"
                time.sleep(.01)
            started = time.monotonic()
            client.call('/v1/operations/'+admitted['id']+'/cancel', {})
            scene_cancelled = json.loads(client.call('/v1/operations/'+admitted['id']+'/wait', {}, timeout=2).body)
            assert scene_cancelled['state']=='cancelled', scene_cancelled
            assert time.monotonic()-started<2, "scene cancellation waited for missing ACK"
            checks.append('scene-factory-missing-ack-cancellable')
            released = mutate("/v1/entities", {"entity": {**entity, "id": "create-release"}})
            assert released["state"] == "succeeded", released
            assert query("/v1/operations/create-held") == expired
            assert query("/v1/operations/create-cancel-held") == cancelled
            deadline = time.monotonic() + 2
            while True:
                listed = query("/v1/entities")["entities"]
                if any(e["ref"]["id"] == "create-held" for e in listed):
                    break
                assert time.monotonic() < deadline, "late ACK never reconciled reserved native model"
                time.sleep(.01)
            checks.extend(["late-ack-preserves-terminal-receipts", "scene-batch-ack-isolation"])
            failed_sdf = artifact.replace("</model>", '<plugin name="unbound-chassis" '
                'filename="libxgc2_simulation_entity_ack.so"><chassisRobotId>create-failed</chassisRobotId>'
                '</plugin></model>')
            failed_entity = {**entity, "id": "create-failed", "asset": {"id": "missing-chassis",
                "realization": {"media_type": "application/sdf+xml", "content": failed_sdf}}}
            submitted = json.loads(client.call("/v1/entities", {
                "entity": failed_entity, "operation_timeout_ms": 500},
                request_id="missing-chassis", timeout=5).body)
            failed = json.loads(client.call(f'/v1/operations/{submitted["id"]}/wait', {}, timeout=2).body)
            assert failed["state"] == "failed" and failed["error"]["code"] == "deadline_exceeded", failed
            assert failed["effects"]["started"] and failed["effects"]["completion"] == "partial_or_unknown"
            assert query("/v1/operations/create-held") == expired
            assert query("/v1/entities/create-following")["entities"][0]["lifecycle"] == "ready"
            checks.append("failed-create-does-not-alter-other-receipts")
            submit_held("create-stop-held")
            assert query("/v1/health")["pending_commands"] == 1
        finally:
            pending_error = sys.exc_info()[1]
            cleanup_errors = []
            for owner in (client, discovery, runtime):
                if owner:
                    try:
                        owner.close()
                    except Exception as error:
                        cleanup_errors.append(str(error))
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
                cleanup_errors.append("native pending factory did not drain")
            log.close()
            assert process.returncode == 0 and not endpoint.exists() and not cleanup_errors, cleanup_errors
            unchanged = artifacts == {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
                                      for path in args.plugins.rglob("libxgc2*.so")}
            assert unchanged, "fixture artifacts changed during probe"
            args.evidence.write_text(json.dumps({"checks": checks, "test_error": None if pending_error is None else str(pending_error),
                "cleanup_errors": cleanup_errors, "exit_code": process.returncode,
                "endpoint_retained": endpoint.exists(), "artifacts_sha256": artifacts}, indent=2) + "\n")
        checks.append("pending-create-shutdown-endpoint-cleanup")
        result = json.loads(args.evidence.read_text())
        result["checks"] = checks
        args.evidence.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result))


if __name__ == "__main__":
    main()
