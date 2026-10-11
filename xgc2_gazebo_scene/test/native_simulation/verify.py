#!/usr/bin/env python3
"""Real isolated gzserver + installed XRPC SDK management conformance."""
import argparse
import concurrent.futures
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import socket
import signal
import subprocess
import sys
import tempfile
import time
from xml.sax.saxutils import escape
from xgc2_xrpc.http import Client, Fault, TransportError
from xgc2_xrpc.runtime import Runtime


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--plugins", required=True)
    parser.add_argument("--gzserver", required=True)
    parser.add_argument("--evidence", type=Path)
    args = parser.parse_args()
    sdk_root = Path(importlib.util.find_spec("xgc2_xrpc").submodule_search_locations[0])
    fixture_files = list(sdk_root.rglob("*.py")) + list(sdk_root.rglob("*.json"))
    fixture_files += list(Path(args.plugins).glob("libxgc2*.so"))

    def source_hashes():
        return {str(path.resolve()): hashlib.sha256(path.read_bytes()).hexdigest()
                for path in sorted(set(fixture_files))}

    admitted_hashes = source_hashes()
    with tempfile.TemporaryDirectory(prefix="sol6-world-") as directory:
        root = Path(directory)
        root.chmod(0o700)
        endpoint = root / "world.sock"
        with socket.socket() as port:
            port.bind(("127.0.0.1", 0))
            master_port = port.getsockname()[1]
        world = root / "fixture.world"
        world.write_text(f'''<sdf version="1.6"><world name="isolated">
          <physics type="ode"><max_step_size>0.001</max_step_size><real_time_update_rate>1000</real_time_update_rate></physics>
          <gravity>0 0 0</gravity>
          <plugin name="simulation" filename="libxgc2_simulation_world.so">
            <socket_path>{escape(str(endpoint))}</socket_path><target_id>test-world</target_id>
            <resource_root>{escape(str(root))}</resource_root>
            <configuration_revision>fixture-configuration-1</configuration_revision>
          </plugin></world></sdf>''')
        environment = dict(os.environ)
        environment.pop("DISPLAY", None)
        environment.pop("WAYLAND_DISPLAY", None)
        environment.update(HOME=str(root), GAZEBO_MASTER_URI=f"http://127.0.0.1:{master_port}",
                           GAZEBO_PLUGIN_PATH=args.plugins, GAZEBO_MODEL_DATABASE_URI="")
        environment["LD_LIBRARY_PATH"] = "/opt/ros/noetic/lib:" + environment.get("LD_LIBRARY_PATH", "")
        log = open(root / "gzserver.log", "w+")
        command = [args.gzserver, "--verbose", "--pause", str(world)]
        process = subprocess.Popen(command, env=environment,
                                   stdout=log, stderr=subprocess.STDOUT)
        runtime = Runtime(blocking_workers=1, max_calls=8)
        discovery = Client(str(endpoint), runtime=runtime)
        client = None
        try:
            deadline = time.monotonic() + 15
            while not endpoint.exists():
                if process.poll() is not None or time.monotonic() >= deadline:
                    log.seek(0)
                    raise AssertionError("world startup failed: " + log.read())
                time.sleep(0.025)
            description = json.loads(discovery.call("/v1/describe", method="GET").body)
            assert source_hashes() == admitted_hashes, "SDK source or native artifacts changed during startup"
            native_maps = Path(f"/proc/{process.pid}/maps").read_text().splitlines()
            fixture_files += [Path(line.split(maxsplit=5)[5]) for line in native_maps
                              if "libxgc2_xrpc_" in line and len(line.split(maxsplit=5)) == 6]
            admitted_hashes = source_hashes()
            ref = description["service_ref"]
            assert ref["target_id"] == "test-world" and ref["api_version"] == "v1"
            assert description["configuration"]["revision"] == "fixture-configuration-1"
            assert description["sensor_kinds"] == ["camera", "imu", "lidar"]
            client = Client(str(endpoint), runtime=runtime, instance_id=ref["instance_id"])

            def query(route):
                return json.loads(client.call(route, method="GET", timeout=5).body)

            def hold(method, body):
                return json.loads(client.call("/v1/call/xgc2.chassis.hold/" + method, body, timeout=5).body)

            assert description["facts"]["capabilities"] == [{"name": "xgc2.chassis.hold", "entities": []}]
            hold_description = hold("Describe", {})
            assert hold_description["instance"] == ref["instance_id"], "HOLD shares the instance of the transport"
            assert hold_description["robots"] == []
            health = query("/v1/health")
            assert health["world_initialized"] and health["lifecycle"] == "ready", health
            immediate = json.loads(client.call("/v1/health/observe", {"after_revision": 0}, timeout=2).body)
            assert immediate["revision"] == health["revision"]
            for _ in range(20):
                observed_at = time.monotonic()
                try:
                    client.call("/v1/health/observe", {"after_revision": health["revision"]}, timeout=0.05)
                except Fault as error:
                    assert error.code == "deadline_exceeded", error
                except TransportError as error:
                    assert process.poll() is None, "native world exited during observation"
                    assert time.monotonic() - observed_at >= 0.04, error
                else:
                    raise AssertionError("unchanged health returned a fabricated event")
            # More queries than host admission slots catches retained HttpReply
            # copies leaking completed per-call leases in the domain queue.
            for _ in range(160):
                assert query("/v1/world")["paused"]

            sequence = 0

            def mutate(route, body, *, method="POST", request_id=None):
                nonlocal sequence
                sequence += 1
                body = {**body, "operation_timeout_ms": 5000}
                response = client.call(route, body, method=method, timeout=5,
                                       request_id=request_id or f"mutation-{sequence}")
                op = json.loads(response.body)
                assert response.status == 202 or op["state"] in ("succeeded", "failed", "cancelled")
                final = json.loads(client.call(f'/v1/operations/{op["id"]}/wait', {}, timeout=10).body)
                return final

            pose = {"position": [1, 2, 3], "orientation": [0, 0, 0, 1]}
            artifact = '<sdf version="1.6"><model name="input"><link name="body"><inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz></inertia></inertial></link></model></sdf>'
            entity = {"id": "robot-1", "role": "robot", "pose": pose,
                      "asset": {"id": "test-model", "realization": {
                          "media_type": "application/sdf+xml", "content": artifact}}}
            before = query("/v1/world")["time"]
            created = mutate("/v1/entities", {"entity": entity}, request_id="create-1")
            assert created["state"] == "succeeded", created
            native = created["result"]["entities"][0]
            generation = native["ref"]["generation"]
            assert native["state"]["pose"] == pose, native
            assert query("/v1/world")["time"] == before, "paused CRUD advanced time"
            repeat = mutate("/v1/entities", {"entity": entity}, request_id="create-1")
            assert repeat["result"] == created["result"], "admitted request replayed"
            conflict = mutate("/v1/entities", {"entity": entity})
            assert conflict["error"]["code"] == "conflict"
            moved_pose = {"position": [4, 5, 6], "orientation": [0, 0, 0, 1]}
            moved = mutate("/v1/entities/robot-1/state", {
                "generation": generation, "state": {"pose": moved_pose, "twist": {
                    "linear": [0, 0, 0], "angular": [0, 0, 0]}}})
            assert moved["result"]["entities"][0]["state"]["pose"] == moved_pose
            reset = mutate("/v1/entities/robot-1/reset", {"generation": generation})
            assert reset["result"]["entities"][0]["state"]["pose"] == pose
            stepped = mutate("/v1/world/step", {"steps": 7})
            assert int(stepped["result"]["time"]["nanoseconds"]) - int(before["nanoseconds"]) == 7000000
            scoped = mutate("/v1/world/reset", {"scope": "entities", "entities": [
                {"id": "robot-1", "generation": generation}], "reset_time": False})
            assert scoped["result"]["time"]["epoch"] == before["epoch"]
            rewound = mutate("/v1/world/reset", {"scope": "all_entities", "reset_time": True})
            assert rewound["result"]["time"] == {"epoch": before["epoch"] + 1, "nanoseconds": "0"}
            removed = mutate("/v1/entities/robot-1", {"generation": generation}, method="DELETE")
            assert removed["state"] == "succeeded"
            replacement = mutate("/v1/entities", {"entity": entity})
            new_generation = replacement["result"]["entities"][0]["ref"]["generation"]
            assert new_generation > generation
            stale = mutate("/v1/entities/robot-1/state", {"generation": generation, "state": {"pose": moved_pose}})
            assert stale["error"]["code"] == "conflict"
            assert query("/v1/entities/robot-1")["entities"][0]["state"]["pose"] == pose
            for n in (1, 2):
                drive_sdf = artifact.replace('</model>', f'<plugin name="drive" filename="libxgc2_chassis_fixture.so"><robot_id>drive-{n}</robot_id></plugin></model>')
                drive = {"id": f"drive-{n}", "role": "robot", "pose": pose,
                         "asset": {"id": "native-drive", "realization": {"media_type": "application/sdf+xml", "content": drive_sdf}}}
                added = mutate("/v1/entities", {"entity": drive})
                assert added["state"] == "succeeded", added
                assert query(f"/v1/entities/drive-{n}")["entities"][0]["state"]["twist"]["linear"] == [1, 0, 0]
            def twist(name):
                return query(f"/v1/entities/{name}")["entities"][0]["state"]["twist"]["linear"]

            assert query("/v1/world")["paused"]
            assert query("/v1/describe")["facts"]["capabilities"] == [
                {"name": "xgc2.chassis.hold", "entities": ["drive-1", "drive-2"]}]
            engaged = hold("Engage", {"robot_ids": ["drive-1", "drive-2"]})["robots"]
            assert [(r["outcome"], r["held"]) for r in engaged] == [("engaged", True)] * 2, engaged
            for n in (1, 2):
                # The native tick writes zero while the world is paused.
                deadline = time.monotonic() + 5
                while hold("State", {"robot_ids": [f"drive-{n}"]})["robots"][0]["stage"] not in ("zero_written", "stopped"):
                    assert time.monotonic() < deadline, "zero was not written while paused"
                    time.sleep(0.01)
            assert twist("drive-1") == [0, 0, 0] and twist("drive-2") == [0, 0, 0]
            released = hold("Release", {"expected_instance": ref["instance_id"], "changes": [
                {"robot_id": r["robot_id"], "expected_revision": r["revision"]} for r in engaged]})["robots"]
            assert [r["outcome"] for r in released] == ["released"] * 2, released
            mutate("/v1/world/step", {"steps": 2})
            assert twist("drive-1") == [0, 0, 0], "release replayed cached command"
            urdf = '<robot name="camera"><link name="body"><inertial><mass value="1"/><inertia ixx="1" ixy="0" ixz="0" iyy="1" iyz="0" izz="1"/></inertial></link></robot>'
            converted = mutate("/v1/entities", {"entity": {"id": "urdf-1", "role": "sensor", "pose": pose,
                "asset": {"id": "camera-urdf", "realization": {"media_type": "application/urdf+xml", "content": urdf}}}})
            assert converted["state"] == "succeeded", converted
            # The same world host realizes authoring geometry and advances it
            # natively, preserving EntityRef identity for unchanged geometry.
            scene_pose = {"position": [2, 0, 0], "orientation": [0, 0, 0, 1]}
            document = {"schema": "xgc2.scene.v1", "id": "fixture-scene", "frame": "world", "obstacles": [
                {"id": "box", "name": "Box", "pose": scene_pose,
                 "parts": [{"id": "body", "pose": {"position": [0, 0, 0], "orientation": [0, 0, 0, 1]},
                            "color": [1, .5, .1, 1],
                            "geometry": {"type": "box", "size": [1, 2, 3]}}],
                 "motion": {"type": "constant_twist", "linear": [1, 0, 0], "angular": [0, 0, 0]}}]}
            scene = mutate("/v1/extensions/scene/apply", {"epoch": "scene-epoch", "revision": 1, "document": document})
            assert scene["state"] == "succeeded", scene
            snapshot = query("/v1/extensions/scene")
            with concurrent.futures.ThreadPoolExecutor(max_workers=1) as observer:
                observed = observer.submit(client.call, "/v1/extensions/scene/observe",
                    {"after_serial": snapshot["serial"]}, timeout=5)
                unchanged = mutate("/v1/extensions/scene/apply", {"epoch": "scene-epoch", "revision": 2, "document": document})
                changed_snapshot = json.loads(observed.result(timeout=6).body)
            assert changed_snapshot["serial"] != snapshot["serial"]
            assert changed_snapshot["revision"] == 2
            assert changed_snapshot["simulation_time"]["epoch"] == query("/v1/world")["time"]["epoch"]
            members = query("/v1/entities")["entities"]
            scene_ref = next(item["ref"] for item in members if item["ref"]["id"] == "scene:box")
            assert unchanged["state"] == "succeeded", unchanged
            assert query("/v1/entities/scene:box")["entities"][0]["ref"] == scene_ref
            played = mutate("/v1/extensions/scene/motion", {"epoch": "scene-epoch", "revision": 2, "operation": "play"})
            assert played["state"] == "succeeded", played
            mutate("/v1/world/step", {"steps": 10})
            actual = query("/v1/entities/scene:box")["entities"][0]["state"]["pose"]["position"][0]
            assert abs(actual - 2.01) < 0.002, actual
            reset_scene = mutate("/v1/extensions/scene/motion", {"epoch": "scene-epoch", "revision": 2, "operation": "reset"})
            assert reset_scene["state"] == "succeeded"
            document["obstacles"] = []
            cleared = mutate("/v1/extensions/scene/apply", {"epoch": "scene-epoch", "revision": 3, "document": document})
            assert cleared["state"] == "succeeded", cleared
            assert all(item["ref"]["id"] != "scene:box" for item in query("/v1/entities")["entities"])
            for body in ({"steps": 1.0}, {"steps": True}, {"steps": 1, "unused": 2}):
                try:
                    mutate("/v1/world/step", body)
                except Fault as error:
                    assert error.code == "invalid_argument"
                else:
                    raise AssertionError("malformed request was accepted")
            try:
                discovery.call("/v1/world", method="GET")
            except Fault as error:
                assert error.code == "conflict"
            else:
                raise AssertionError("unbound business route dispatched")
            evidence = {"native_engine": "Gazebo Classic 11", "service_ref": ref,
                              "process": {"pid": process.pid, "executable": str(Path(f"/proc/{process.pid}/exe").resolve()),
                                          "command": command, "rootfs": "native host",
                                          "display": environment.get("DISPLAY"), "wayland_display": environment.get("WAYLAND_DISPLAY"),
                                          "gzclient": False},
                              "limits": ["native semantics; no Focal ABI or deployment claim"],
                              "checks": ["paused-native-create", "native-pose", "reset", "exact-step",
                                         "time-epoch", "removal", "replacement-fence", "deduplication",
                                         "strict-input", "instance-binding", "native-scene-geometry",
                                         "scene-identity", "native-scene-motion", "native-scene-clear",
                                         "shared-hold-instance", "native-multi-hold-zero",
                                         "hold-release-no-replay", "native-urdf-create", "configuration-revision",
                                         "native-health", "health-observe-deadline-reclaim", "scene-held-observe",
                                         "completed-query-leases-released"]}
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
                cleanup_errors.append("native world did not drain before shutdown deadline")
            log.seek(0)
            output = log.read()
            log.close()
            completed_hashes = source_hashes()
            if completed_hashes != admitted_hashes:
                cleanup_errors.append("SDK source or native artifacts changed during the fixture")
            if args.evidence:
                args.evidence.with_suffix(".log").write_text(output)
                args.evidence.with_suffix(".cleanup.json").write_text(json.dumps({
                    "pid": process.pid, "exit_code": process.returncode,
                    "endpoint_retained": endpoint.exists(), "cleanup_errors": cleanup_errors,
                    "test_error": None if pending_error is None else str(pending_error),
                    "admitted_sha256": admitted_hashes, "completed_sha256": completed_hashes,
                }, indent=2) + "\n")
            if process.returncode != 0:
                raise AssertionError(f"native world shutdown exited {process.returncode}") from pending_error
            if endpoint.exists() and sys.exc_info()[0] is None:
                raise AssertionError(f"world shutdown retained its SDK endpoint lease (exit {process.returncode})")
            if cleanup_errors and pending_error is None:
                raise AssertionError("SDK cleanup failed: " + "; ".join(cleanup_errors))
        evidence["checks"].append("native-shutdown-endpoint-cleanup")
        evidence["source_sha256"] = admitted_hashes
        if args.evidence:
            args.evidence.write_text(json.dumps(evidence, indent=2) + "\n")
        print(json.dumps(evidence, sort_keys=True))


if __name__ == "__main__":
    main()
