#!/usr/bin/env python3
"""Deferred native chassis enrollment retains the original Create receipt."""
import argparse
import ctypes
import hashlib
import json
import os
from pathlib import Path
import select
import signal
import socket
import subprocess
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
    checks = []
    artifacts = {p.name: hashlib.sha256(p.read_bytes()).hexdigest()
                 for p in args.plugins.glob("libxgc2*.so")}
    with tempfile.TemporaryDirectory(prefix="chassis-enrollment-") as directory:
        root = Path(directory)
        root.chmod(0o700)
        endpoint = root / "world.sock"
        gates = {name: socket.socketpair() for name in ("delayed", "expired", "cancelled", "stopped")}
        for parent, child in gates.values():
            parent.settimeout(5)
        with socket.socket() as port:
            port.bind(("127.0.0.1", 0))
            master_port = port.getsockname()[1]
        world = root / "fixture.world"
        world.write_text(f'''<sdf version="1.6"><world name="isolated"><gravity>0 0 0</gravity>
          <plugin name="simulation" filename="libxgc2_simulation_world.so">
            <socket_path>{escape(str(endpoint))}</socket_path><target_id>enrollment-fixture</target_id>
            <resource_root>{escape(str(root))}</resource_root>
          </plugin></world></sdf>''')
        environment = dict(os.environ)
        environment.pop("DISPLAY", None)
        environment.pop("WAYLAND_DISPLAY", None)
        environment.update(HOME=str(root), GAZEBO_MASTER_URI=f"http://127.0.0.1:{master_port}",
                           GAZEBO_PLUGIN_PATH=str(args.plugins), GAZEBO_MODEL_DATABASE_URI="")
        environment["LD_LIBRARY_PATH"] = str(args.plugins) + ":/opt/ros/noetic/lib:" + environment.get("LD_LIBRARY_PATH", "")
        runtime = Runtime(blocking_workers=1, max_calls=8)
        discovery = Client(str(endpoint), runtime=runtime)
        client = None
        log = args.evidence.with_suffix(".log").open("w")
        native = ctypes.CDLL(None, use_errno=True)
        watcher = native.inotify_init1(os.O_NONBLOCK | os.O_CLOEXEC)
        assert watcher >= 0 and native.inotify_add_watch(watcher, os.fsencode(root), 0x100) >= 0
        process = subprocess.Popen([args.gzserver, "--verbose", "--pause", str(world)], env=environment,
            pass_fds=tuple(child.fileno() for parent, child in gates.values()),
            stdout=log, stderr=subprocess.STDOUT)
        try:
            deadline = time.monotonic() + 15
            # Startup and enrollment both wait on events rather than an elapsed-time assumption.
            while not endpoint.exists():
                assert process.poll() is None and time.monotonic() < deadline, "world startup failed"
                assert select.select([watcher], [], [], max(0, deadline - time.monotonic()))[0], "world startup failed"
                os.read(watcher, 65536)
            ref = json.loads(discovery.call("/v1/describe", method="GET").body)["facts"]["service_ref"]
            client = Client(str(endpoint), runtime=runtime, instance_id=ref["instance_id"])

            def query(route):
                return json.loads(client.call(route, method="GET", timeout=5).body)

            def wait(name):
                return json.loads(client.call(f"/v1/operations/{name}/wait", {}, timeout=5).body)

            pose = {"position": [0, 0, 1], "orientation": [0, 0, 0, 1]}

            def create(name, *, gated=True, timeout=5000):
                plugin = ""
                if gated:
                    plugin = ('<plugin name="chassis" filename="libxgc2_chassis_fixture.so">'
                              f'<robot_id>{name}</robot_id><chassisRobotId>{name}</chassisRobotId>'
                              f'<enrollment_fd>{gates[name][1].fileno()}</enrollment_fd></plugin>')
                artifact = ('<sdf version="1.6"><model name="input"><link name="body">'
                            '<inertial><mass>1</mass><inertia><ixx>1</ixx><iyy>1</iyy><izz>1</izz>'
                            '</inertia></inertial></link>' + plugin + '</model></sdf>')
                result = json.loads(client.call("/v1/entities", {"entity": {
                    "id": name, "role": "robot", "pose": pose, "asset": {"id": "fixture",
                    "realization": {"media_type": "application/sdf+xml", "content": artifact}}},
                    "operation_timeout_ms": timeout}, request_id=name, timeout=5).body)
                assert result["id"] == name, result
                if gated:
                    assert gates[name][0].recv(1) == b"L", "native plant did not load"
                return result

            def release(name):
                gates[name][0].sendall(b"E")
                assert gates[name][0].recv(1) == b"E", "native plant did not enroll"

            create("delayed")
            # Gazebo's following factory and its final ACK complete before enrollment is released.
            # This proves the first model ACK was handled without using an elapsed-time assumption.
            create("following", gated=False)
            assert wait("following")["state"] == "succeeded"
            pending = query("/v1/operations/delayed")
            assert pending["state"] == "running", pending
            assert all(e["ref"]["id"] != "delayed" for e in query("/v1/entities")["entities"])
            release("delayed")  # No World mutation wakes the scene worker after this signal.
            completed = wait("delayed")
            assert completed["state"] == "succeeded", completed
            assert query("/v1/entities/delayed")["entities"][0]["lifecycle"] == "ready"
            checks.append("same-create-receipt-completes-on-delayed-enrollment")

            create("expired", timeout=500)
            expired = wait("expired")
            assert expired["state"] == "failed" and expired["error"]["code"] == "deadline_exceeded", expired
            release("expired")
            assert query("/v1/operations/expired") == expired
            checks.append("missing-enrollment-preserves-original-deadline-and-late-terminal-receipt")

            create("cancelled")
            create("cancel-barrier", gated=False)
            assert wait("cancel-barrier")["state"] == "succeeded"
            assert query("/v1/operations/cancelled")["state"] == "running"
            client.call("/v1/operations/cancelled/cancel", {}, timeout=5)
            cancelled = wait("cancelled")
            assert cancelled["state"] == "cancelled", cancelled
            release("cancelled")
            assert query("/v1/operations/cancelled") == cancelled
            assert query("/v1/entities/following")["entities"][0]["lifecycle"] == "ready"
            checks.append("cancelled-enrollment-retains-terminal-receipt-and-healthy-sibling")

            create("stopped")
            assert query("/v1/operations/stopped")["state"] == "running"
        finally:
            for owner in (client, discovery, runtime):
                if owner:
                    owner.close()
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
                raise AssertionError("pending chassis enrollment prevented world stop")
            finally:
                for pair in gates.values():
                    for gate in pair:
                        gate.close()
                log.close()
                os.close(watcher)
            assert process.returncode == 0 and not endpoint.exists(), "world owner did not cleanly stop"
        checks.append("pending-enrollment-stop-joins-native-owners-and-releases-endpoint")
        args.evidence.write_text(json.dumps({"checks": checks, "exit_code": process.returncode,
            "endpoint_retained": endpoint.exists(), "artifacts_sha256": artifacts}, indent=2) + "\n")
        print(json.dumps({"checks": checks, "exit_code": process.returncode}))


if __name__ == "__main__":
    main()
