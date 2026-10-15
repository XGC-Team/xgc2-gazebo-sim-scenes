#!/usr/bin/env python3
"""Real paused Gazebo Create with a withheld first sensor Init ACK."""
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
            GAZEBO_PLUGIN_PATH=str(args.plugins / "sensor_ack_fixture") + ":" + str(args.plugins / "create_ack_fixture") + ":" + str(args.plugins),
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

            def sensed(name):
                return artifact.replace('</link>', '<sensor name="'+name+'" type="imu"><always_on>true</always_on><update_rate>10</update_rate></sensor></link>')

            def with_sensor(name):
                return {**entity, "asset": {"id": "sensor-model", "realization": {
                    "media_type": "application/sdf+xml", "content": sensed(name)}}}

            first = json.loads(client.call("/v1/entities", {
                "entity": {**with_sensor("held_sensor"), "id": "sensor-held"}, "operation_timeout_ms": 500},
                request_id="sensor-held", timeout=5).body)
            deadline = time.monotonic()+5
            while not ((root/"held_sensor.init").exists() and (root/"sensor-held.factory").exists()):
                assert time.monotonic()<deadline, "first sensor Init never reached fixture"
                time.sleep(.01)
            assert (root/"sensor-held.factory").exists(), "parent factory did not acknowledge"
            started = time.monotonic()
            expired = json.loads(client.call("/v1/operations/sensor-held/wait", {}, timeout=2).body)
            assert expired["state"] == "failed" and expired["error"]["code"] == "deadline_exceeded", expired
            assert time.monotonic() - started < 2, "missing sensor ACK did not settle within budget"
            following_submission = json.loads(client.call("/v1/entities", {
                "entity": {**entity, "id": "sensor-following"}, "operation_timeout_ms": 5000},
                request_id="sensor-following", timeout=5).body)
            deadline = time.monotonic()+1
            while not (root/"sensor-following.factory").exists() and time.monotonic()<deadline:
                time.sleep(.01)
            assert (root/"sensor-following.factory").exists(), "following factory blocked behind held sensor ACK"
            following = json.loads(client.call('/v1/operations/'+following_submission["id"]+'/wait', {}, timeout=5).body)
            assert following["state"] == "succeeded", following
            assert query("/v1/operations/sensor-held") == expired
            checks.append("following-create-completes-before-first-sensor-ack")
            duplicate = mutate("/v1/entities", {"entity": {**entity, "id": "sensor-held"}})
            assert duplicate["error"]["code"] == "conflict", duplicate
            assert all(item["ref"]["id"] != "sensor-held" for item in query("/v1/entities")["entities"])
            checks.append("pending-parent-reserved-without-fabricated-ready")
            released = mutate("/v1/entities", {"entity": {**with_sensor("release_sensor"), "id": "sensor-release"}})
            assert released["state"] == "succeeded", released
            assert query("/v1/operations/sensor-held") == expired
            deadline = time.monotonic() + 2
            while True:
                listed = query("/v1/entities")["entities"]
                ready = next((e for e in listed if e["ref"]["id"] == "sensor-held"), None)
                if ready:
                    break
                assert time.monotonic() < deadline, "late sensor ACK never reconciled reserved parent"
                time.sleep(.01)
            generation = ready["ref"]["generation"]
            managed = query("/v1/sensors")["sensors"]
            assert all(s["native_initialized"] for s in managed), managed
            assert any(s["parent"]["id"] == "sensor-held" and s["parent"]["generation"] == generation for s in managed)
            checks.append("late-sensor-ack-preserves-deadline-receipt")
            removed = mutate("/v1/entities/sensor-held", {"generation": generation}, method="DELETE")
            assert removed["state"] == "succeeded", removed
            replacement = mutate("/v1/entities", {"entity": {**entity, "id": "sensor-held"}})
            assert replacement["state"] == "succeeded", replacement
            assert replacement["result"]["entities"][0]["ref"]["generation"] != generation
            assert all(s["parent"]["id"] != "sensor-held" for s in query("/v1/sensors")["sensors"])
            assert query("/v1/operations/sensor-following") == following
            checks.append("sensor-parent-generation-isolation-and-native-release")
            parent_generation = following["result"]["entities"][0]["ref"]["generation"]
            started = time.monotonic()
            admitted = json.loads(client.call("/v1/sensors", {"sensor": {
                "id": "deadline_sensor", "parent": {"id": "sensor-following", "generation": parent_generation, "link": "body"},
                "realization": {"media_type": "application/sdf+xml", "content":
                    '<sensor name="input" type="imu"><always_on>true</always_on><update_rate>10</update_rate></sensor>'}},
                "operation_timeout_ms": 200}, request_id="sensor-deadline", timeout=2).body)
            bounded = json.loads(client.call('/v1/operations/'+admitted["id"]+'/wait', {}, timeout=2).body)
            assert bounded["state"] == "failed" and bounded["error"]["code"] == "deadline_exceeded", bounded
            assert time.monotonic() - started < 2, "controller Wait ignored the sensor deadline"
            assert query("/v1/entities/sensor-following")["entities"], "deadline blocked following commands"
            checks.append("native-sensor-wait-bounded-missing-ack")
            admitted = json.loads(client.call("/v1/sensors", {"sensor": {
                "id": "cancel_sensor", "parent": {"id": "sensor-following", "generation": parent_generation, "link": "body"},
                "realization": {"media_type": "application/sdf+xml", "content":
                    '<sensor name="input" type="imu"><always_on>true</always_on><update_rate>10</update_rate></sensor>'}},
                "operation_timeout_ms": 5000}, request_id="sensor-cancel", timeout=2).body)
            deadline = time.monotonic() + 2
            while not (root / "cancel_sensor.init").exists():
                assert time.monotonic() < deadline, "cancel sensor never reached native initialization"
                time.sleep(.01)
            started = time.monotonic()
            client.call('/v1/operations/'+admitted['id']+'/cancel', {})
            cancelled = json.loads(client.call('/v1/operations/'+admitted['id']+'/wait', {}, timeout=2).body)
            assert cancelled['state'] == 'cancelled', cancelled
            assert time.monotonic()-started < 2, "sensor cancellation waited for native ACK or original budget"
            checks.append("native-sensor-wait-cancellable-missing-ack")
            parent = mutate('/v1/entities', {'entity': {**with_sensor('held_release_sensor'), 'id': 'release-parent'}})
            parent_generation = parent['result']['entities'][0]['ref']['generation']
            started = time.monotonic()
            admitted = json.loads(client.call('/v1/entities/release-parent', {
                'generation': parent_generation, 'operation_timeout_ms': 200}, method='DELETE',
                request_id='release-deadline', timeout=2).body)
            release_expired = json.loads(client.call('/v1/operations/'+admitted['id']+'/wait', {}, timeout=2).body)
            assert release_expired['state']=='failed' and release_expired['error']['code']=='deadline_exceeded', release_expired
            assert time.monotonic()-started<2, 'parent removal renewed or ignored its sensor-release budget'
            parent = mutate('/v1/entities', {'entity': {**with_sensor('cancel_release_sensor'), 'id': 'cancel-release-parent'}})
            parent_generation = parent['result']['entities'][0]['ref']['generation']
            admitted = json.loads(client.call('/v1/entities/cancel-release-parent', {
                'generation': parent_generation, 'operation_timeout_ms': 5000}, method='DELETE',
                request_id='release-cancel', timeout=2).body)
            deadline = time.monotonic()+2
            while not (root/'cancel_release_sensor.released').exists():
                assert time.monotonic()<deadline, 'native sensor release never reached fixture'
                time.sleep(.01)
            started = time.monotonic()
            client.call('/v1/operations/'+admitted['id']+'/cancel', {})
            release_cancelled = json.loads(client.call('/v1/operations/'+admitted['id']+'/wait', {}, timeout=2).body)
            assert release_cancelled['state']=='cancelled', release_cancelled
            assert time.monotonic()-started<2, 'parent removal cancellation waited for missing sensor release ACK'
            checks.append('parent-sensor-release-missing-ack-bounded-and-cancellable')
            stopped = json.loads(client.call("/v1/entities", {
                "entity": {**with_sensor("stop_sensor"), "id": "sensor-stop"}, "operation_timeout_ms": 5000},
                request_id="sensor-stop", timeout=5).body)
            deadline=time.monotonic()+5
            while not (root/"stop_sensor.init").exists():
                assert time.monotonic()<deadline, "shutdown sensor Init never reached fixture"
                time.sleep(.01)
            assert query("/v1/operations/sensor-stop")["state"] == "running"
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
        checks.append("pending-sensor-create-shutdown-endpoint-cleanup")
        result = json.loads(args.evidence.read_text())
        result["checks"] = checks
        args.evidence.write_text(json.dumps(result, indent=2) + "\n")
        print(json.dumps(result))


if __name__ == "__main__":
    main()
