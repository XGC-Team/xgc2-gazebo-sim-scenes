#!/usr/bin/env python3
"""Offline contract for scripts/spawn_robot_model (no ROS or Gazebo needed).

A fake ROS master answers the XML-RPC presence probe, and stub rospy /
gazebo_msgs / geometry_msgs modules stand in for the Gazebo services. The
stubs record when each process holds the shared spawn lock, so the tests can
compare the helper with the flock(1)-wrapped gazebo_ros spawn_model it
replaces.
"""
import importlib.machinery
import importlib.util
import json
import os
import pathlib
import socket
import subprocess
import sys
import tempfile
import textwrap
import threading
import time
import unittest
from xmlrpc.server import SimpleXMLRPCServer

PACKAGE = pathlib.Path(__file__).resolve().parents[1]
HELPER = PACKAGE / "scripts" / "spawn_robot_model"
SERVICES = [
    ["/gazebo/get_world_properties", ["/gazebo"]],
    ["/gazebo/spawn_urdf_model", ["/gazebo"]],
    ["/gazebo/spawn_sdf_model", ["/gazebo"]],
]


def load_helper():
    loader = importlib.machinery.SourceFileLoader("spawn_robot_model_under_test", str(HELPER))
    spec = importlib.util.spec_from_loader(loader.name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


class FakeMaster:
    def __init__(self, services=None):
        self.state = [[], [], services or []]
        self.server = SimpleXMLRPCServer(("127.0.0.1", 0), logRequests=False, allow_none=True)
        self.server.register_function(lambda caller: [1, "ok", self.state], "getSystemState")
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.uri = "http://127.0.0.1:{}/".format(self.server.server_address[1])

    def close(self):
        self.server.shutdown()
        self.server.server_close()


STUB_ROSPY = textwrap.dedent('''
    import fcntl, json, os, sys, time
    time.sleep(float(os.environ.get("STUB_ROSPY_IMPORT_SECONDS", "0")))
    EVENTS, STATE, LOCK = os.environ["STUB_EVENTS"], os.environ["STUB_GAZEBO_STATE"], os.environ["STUB_SPAWN_LOCK"]

    class ROSException(Exception):
        pass

    class ServiceException(Exception):
        pass

    def _record(kind, **values):
        values.update(kind=kind, pid=os.getpid(), at=time.monotonic())
        with open(EVENTS, "a") as stream:
            fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
            stream.write(json.dumps(values) + "\\n")

    def _models(update=None):
        with open(STATE + ".lock", "a+") as guard:
            fcntl.flock(guard.fileno(), fcntl.LOCK_EX)
            models = json.load(open(STATE)) if os.path.exists(STATE) else []
            if update is not None:
                models = update(models)
                json.dump(models, open(STATE, "w"))
            return models

    def myargv(argv):
        return [value for value in argv if ":=" not in value]

    def init_node(name, **_kwargs):
        _record("init_node", name=name)

    def get_namespace():
        return os.environ.get("STUB_NAMESPACE", "/")

    def get_param(name):
        params = json.loads(os.environ["STUB_PARAMS"])
        if name not in params:
            raise KeyError(name)
        return params[name]

    def wait_for_service(name, timeout=None):
        _record("wait_for_service", service=name)

    def signal_shutdown(reason):
        _record("shutdown", reason=reason)

    def loginfo(*_args):
        pass

    logerr = logwarn = loginfo

    class _Result:
        def __init__(self, **values):
            self.__dict__.update(values)

    class ServiceProxy:
        def __init__(self, name, _service_type):
            self.name = name

        def __call__(self, *args):
            if self.name.endswith("/get_world_properties"):
                return _Result(model_names=_models(), success=True, status_message="")
            model = args[0]
            _record("spawn_begin", model=model, service=self.name, robot_namespace=args[2],
                    pose=[args[3].position.x, args[3].position.y, args[3].orientation.z, args[3].orientation.w])
            time.sleep(float(os.environ.get("STUB_SPAWN_SECONDS", "0")))
            drops = int(os.environ.get("STUB_SILENT_DROPS", "0"))
            dropped = len([e for e in _events() if e["kind"] == "spawn_end" and e.get("dropped")])
            keep = dropped >= drops
            if keep:
                _models(lambda models: sorted(set(models) | {model}))
            _record("spawn_end", model=model, dropped=not keep)
            return _Result(success=True, status_message="SpawnModel: Successfully spawned entity")

    def _events():
        return [json.loads(line) for line in open(EVENTS)] if os.path.exists(EVENTS) else []
''')

STUB_MESSAGES = {
    "gazebo_msgs/__init__.py": "",
    "gazebo_msgs/srv.py": "GetWorldProperties = SpawnModel = DeleteModel = object\n",
    "geometry_msgs/__init__.py": "",
    "geometry_msgs/msg.py": textwrap.dedent('''
        class _Vector:
            x = y = z = w = 0.0

        class Pose:
            def __init__(self):
                self.position = _Vector()
                self.orientation = _Vector()
    '''),
}

# The helper with its lock redirected into the test directory; nothing else changes.
WRAPPER = textwrap.dedent('''
    import importlib.machinery, importlib.util, os, sys
    loader = importlib.machinery.SourceFileLoader("spawn_robot_model", os.environ["HELPER"])
    spec = importlib.util.spec_from_loader(loader.name, loader)
    helper = importlib.util.module_from_spec(spec)
    loader.exec_module(helper)
    helper.acquire_gazebo_spawn_lock.__defaults__ = (os.environ["STUB_SPAWN_LOCK"],)
    helper.VERIFY_WAIT_SECONDS = float(os.environ.get("STUB_VERIFY_SECONDS", helper.VERIFY_WAIT_SECONDS))
    helper.main(sys.argv)
''')

# What robot launches ran before: gazebo_ros spawn_model (import, init_node,
# read the parameter, wait for the service, spawn), wrapped in flock -x.
LEGACY_SPAWN_MODEL = textwrap.dedent('''
    import json, os, sys
    import rospy
    rospy.init_node("spawn_model", anonymous=True)
    xml = rospy.get_param(sys.argv[sys.argv.index("-param") + 1])
    rospy.wait_for_service("/gazebo/spawn_sdf_model")
    from geometry_msgs.msg import Pose
    model = sys.argv[sys.argv.index("-model") + 1]
    rospy.ServiceProxy("/gazebo/spawn_sdf_model", None)(model, xml, "/", Pose(), "")
''')


class Sandbox:
    def __init__(self, directory):
        self.root = pathlib.Path(directory)
        stubs = self.root / "stubs"
        (stubs / "rospy").mkdir(parents=True)
        (stubs / "rospy" / "__init__.py").write_text(STUB_ROSPY)
        for relative, content in STUB_MESSAGES.items():
            (stubs / relative).parent.mkdir(parents=True, exist_ok=True)
            (stubs / relative).write_text(content)
        (self.root / "wrapper.py").write_text(WRAPPER)
        (self.root / "legacy_spawn_model.py").write_text(LEGACY_SPAWN_MODEL)
        self.events = self.root / "events.jsonl"
        self.lock = self.root / "spawn.lock"
        self.environment = dict(
            os.environ, PYTHONPATH=str(stubs), HELPER=str(HELPER), STUB_EVENTS=str(self.events),
            STUB_GAZEBO_STATE=str(self.root / "models.json"), STUB_SPAWN_LOCK=str(self.lock),
            STUB_PARAMS=json.dumps({"/ugv1/robot_description": "<robot name='scout'/>",
                                    "/ugv2/robot_description": "<robot name='scout'/>",
                                    "/ugv1/gazebo_model_sdf": "<sdf version='1.6'><model name='m'/></sdf>"}),
        )

    def helper(self, master_uri, model, extra=(), environment=None):
        env = dict(self.environment, ROS_MASTER_URI=master_uri, **(environment or {}))
        return subprocess.Popen(
            [sys.executable, str(self.root / "wrapper.py"), "-urdf", "-param", "/{}/robot_description".format(model),
             "-model", model, "-x", "1.5", "-Y", "1.0", "__name:=spawn_" + model, "__log:=/tmp/x.log", *extra],
            env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    def legacy(self, master_uri, model, environment):
        env = dict(self.environment, ROS_MASTER_URI=master_uri, **environment)
        return subprocess.Popen(
            ["flock", "-x", str(self.lock), sys.executable, str(self.root / "legacy_spawn_model.py"),
             "-sdf", "-param", "/ugv1/gazebo_model_sdf", "-model", model],
            env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    def read_events(self):
        if not self.events.exists():
            return []
        return [json.loads(line) for line in self.events.read_text().splitlines()]


def unused_port():
    with socket.socket() as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


class PresenceTest(unittest.TestCase):
    def setUp(self):
        self.helper = load_helper()

    def run_wait(self, snapshots, absent=30.0, starting=60.0):
        now = [0.0]

        def probe(_uri):
            value = snapshots(now[0])
            if isinstance(value, Exception):
                raise value
            return value

        def sleep(seconds):
            now[0] += seconds

        required = ("/gazebo/get_world_properties", "/gazebo/spawn_urdf_model")
        try:
            self.helper.wait_for_gazebo("http://master:11311", "/gazebo", required, absent, starting,
                                        probe=probe, clock=lambda: now[0], sleep=sleep)
            return None, now[0]
        except self.helper.HelperExit as failure:
            return failure, now[0]

    def test_ready_returns_at_once(self):
        failure, elapsed = self.run_wait(lambda _t: [[], [], SERVICES])
        self.assertEqual((failure, elapsed), (None, 0.0))

    def test_absent_starting_and_unreachable_are_bounded(self):
        failure, elapsed = self.run_wait(lambda _t: [[], [], []], absent=30.0)
        self.assertEqual(failure.code, 5)
        self.assertLess(elapsed, 31.0)
        failure, elapsed = self.run_wait(lambda t: [[], [], SERVICES if t >= 40 else SERVICES[:1]])
        self.assertIsNone(failure)
        failure, elapsed = self.run_wait(lambda _t: [[], [], SERVICES[:1]], starting=60.0)
        self.assertEqual(failure.code, 1)
        self.assertGreaterEqual(elapsed, 60.0)
        failure, elapsed = self.run_wait(lambda _t: ConnectionRefusedError("refused"))
        self.assertEqual(failure.code, 6)
        self.assertLess(elapsed, 6.0)

    def test_sdf_needs_the_sdf_service(self):
        state = [[], [], [SERVICES[0], SERVICES[1]]]
        self.assertEqual(self.helper.gazebo_presence(
            state, "/gazebo", ("/gazebo/get_world_properties", "/gazebo/spawn_sdf_model")), "starting")

    def test_quaternion_matches_tf_convention(self):
        x, y, z, w = self.helper.quaternion_from_rpy(0.0, 0.0, 1.0)
        self.assertAlmostEqual(z, 0.479425538604203)
        self.assertAlmostEqual(w, 0.8775825618903728)
        self.assertAlmostEqual(x * x + y * y + z * z + w * w, 1.0)


class HelperProcessTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.sandbox = Sandbox(self.directory.name)

    def tearDown(self):
        self.directory.cleanup()

    def finish(self, process, timeout=60):
        stdout, stderr = process.communicate(timeout=timeout)
        return process.returncode, stdout, stderr

    def test_missing_gazebo_exits_5_before_importing_rospy(self):
        master = FakeMaster(services=[["/rosout/get_loggers", ["/rosout"]]])
        try:
            started = time.monotonic()
            code, _out, err = self.finish(self.sandbox.helper(
                master.uri, "ugv1", ("-gazebo_timeout", "1"), {"STUB_ROSPY_IMPORT_SECONDS": "30"}))
            elapsed = time.monotonic() - started
        finally:
            master.close()
        self.assertEqual(code, 5, err)
        self.assertIn("Gazebo is not running", err)
        self.assertLess(elapsed, 5.0)
        self.assertEqual(self.sandbox.read_events(), [])

    def test_missing_master_exits_6(self):
        code, _out, err = self.finish(self.sandbox.helper("http://127.0.0.1:{}".format(unused_port()), "ugv1",
                                                          ("-gazebo_timeout", "1")))
        self.assertEqual(code, 6, err)

    def test_spawn_uses_the_node_namespace_pose_and_retries_a_silent_drop(self):
        master = FakeMaster(services=SERVICES)
        try:
            code, out, err = self.finish(self.sandbox.helper(
                master.uri, "ugv1", environment={"STUB_SILENT_DROPS": "1", "STUB_NAMESPACE": "/ugv1/",
                                                 "STUB_VERIFY_SECONDS": "0.3"}))
        finally:
            master.close()
        self.assertEqual(code, 0, err)
        result = json.loads(out.split("SPAWN_ROBOT_MODEL_RESULT ", 1)[1])
        self.assertEqual((result["model"], result["outcome"]), ("ugv1", "spawned"))
        spawns = [event for event in self.sandbox.read_events() if event["kind"] == "spawn_begin"]
        self.assertEqual(len(spawns), 2)
        self.assertEqual(spawns[0]["service"], "/gazebo/spawn_urdf_model")
        self.assertEqual(spawns[0]["robot_namespace"], "/ugv1/")
        self.assertAlmostEqual(spawns[0]["pose"][0], 1.5)
        self.assertAlmostEqual(spawns[0]["pose"][2], 0.479425538604203)
        self.assertIn("shutdown", [event["kind"] for event in self.sandbox.read_events()])

    def test_existing_model_is_reused_and_hold_keeps_running_as_sleep(self):
        (self.sandbox.root / "models.json").write_text(json.dumps(["ugv1"]))
        master = FakeMaster(services=SERVICES)
        try:
            process = self.sandbox.helper(master.uri, "ugv1", ("-hold",))
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                with open("/proc/{}/cmdline".format(process.pid), "rb") as stream:
                    command = stream.read().split(b"\0")
                if command[0] == b"sleep":
                    break
                time.sleep(0.05)
            self.assertEqual(command[0], b"sleep", "a held helper must become sleep")
            self.assertIsNone(process.poll())
            process.terminate()
            code, out, _err = self.finish(process)
        finally:
            master.close()
        self.assertNotEqual(code, 0)
        self.assertEqual(json.loads(out.split("SPAWN_ROBOT_MODEL_RESULT ", 1)[1])["outcome"], "reused")
        self.assertNotIn("spawn_begin", [event["kind"] for event in self.sandbox.read_events()])

    def test_parallel_robots_hold_the_lock_only_for_the_insert(self):
        count, prepare, insert = 6, 0.6, 0.15
        timing = {"STUB_ROSPY_IMPORT_SECONDS": str(prepare), "STUB_SPAWN_SECONDS": str(insert)}
        params = {"/ugv{}/robot_description".format(i): "<robot name='scout'/>" for i in range(1, count + 1)}
        master = FakeMaster(services=SERVICES)
        try:
            started = time.monotonic()
            processes = [self.sandbox.helper(master.uri, "ugv{}".format(i), environment=dict(
                timing, STUB_PARAMS=json.dumps(params))) for i in range(1, count + 1)]
            results = [self.finish(process) for process in processes]
            parallel = time.monotonic() - started
            self.sandbox.events.unlink()
            (self.sandbox.root / "models.json").unlink()
            started = time.monotonic()
            legacy = [self.sandbox.legacy(master.uri, "m{}".format(i), timing) for i in range(count)]
            legacy_results = [self.finish(process) for process in legacy]
            serial = time.monotonic() - started
        finally:
            master.close()
        for code, _out, err in results + legacy_results:
            self.assertEqual(code, 0, err)
        self.assertLess(parallel, serial * 0.6)
        print("{} robot spawns: flock around spawn_model {:.2f} s, lock around the insert {:.2f} s".format(
            count, serial, parallel))

    def test_inserts_never_overlap(self):
        count = 5
        params = {"/ugv{}/robot_description".format(i): "<robot/>" for i in range(1, count + 1)}
        master = FakeMaster(services=SERVICES)
        try:
            processes = [self.sandbox.helper(master.uri, "ugv{}".format(i), environment={
                "STUB_SPAWN_SECONDS": "0.1", "STUB_PARAMS": json.dumps(params)}) for i in range(1, count + 1)]
            for process in processes:
                self.assertEqual(self.finish(process)[0], 0)
        finally:
            master.close()
        spans = {}
        for event in self.sandbox.read_events():
            if event["kind"] in ("spawn_begin", "spawn_end"):
                spans.setdefault(event["pid"], {})[event["kind"]] = event["at"]
        ordered = sorted((span["spawn_begin"], span["spawn_end"]) for span in spans.values())
        self.assertEqual(len(ordered), count)
        for (_, previous_end), (next_begin, _) in zip(ordered, ordered[1:]):
            self.assertGreaterEqual(next_begin, previous_end)


if __name__ == "__main__":
    unittest.main(verbosity=2)
