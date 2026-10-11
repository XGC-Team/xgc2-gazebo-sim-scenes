#!/usr/bin/env python3
"""Late/replaced masters and restarted topic consumers, without Gazebo.

The --long cell holds each peer type down for 60 seconds of wall time.
"""
import argparse
import json
import os
from pathlib import Path
import random
import select
import signal
import socket
import subprocess
import tempfile
import threading
import time
import unittest
from xmlrpc.client import ServerProxy
from xmlrpc.server import SimpleXMLRPCServer


def free_port():
    for _ in range(100):
        port = random.randrange(20000, 40001)
        with socket.socket() as reserved:
            try:
                reserved.bind(('127.0.0.1', port))
                return port
            except OSError:
                pass
    raise RuntimeError('No private ROS master port available')


def stop(process):
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
    process.wait(timeout=5)
    for stream in (process.stdin, process.stdout):
        if stream is not None:
            stream.close()


def resources(process):
    root = Path('/proc')/str(process.pid)
    fields = (root/'stat').read_text().rpartition(')')[2].split()
    return {'cpu': (int(fields[11])+int(fields[12]))/os.sysconf('SC_CLK_TCK'),
            'threads': len(list((root/'task').iterdir())), 'fds': len(list((root/'fd').iterdir()))}


class FakeMaster:
    def __init__(self, port, identity=1, stalled=False):
        self.release = threading.Event()
        self.server = SimpleXMLRPCServer(('127.0.0.1', port), logRequests=False, allow_none=True)
        self.server.register_function(lambda caller, key: [1, '', 'run-'+str(identity)], 'getParam')
        def pid(caller):
            if stalled:
                self.release.wait()
            return [1, '', identity]
        self.server.register_function(pid, 'getPid')
        self.thread = threading.Thread(target=self.server.serve_forever, kwargs={'poll_interval': .05})
        self.thread.start()

    def close(self):
        self.release.set()
        self.server.shutdown()
        self.thread.join()
        self.server.server_close()


class ParallelStartTest(unittest.TestCase):
    fixture = None
    long = False

    def setUp(self):
        self.directory = tempfile.TemporaryDirectory(prefix='gazebo-start-order-')
        self.root = Path(self.directory.name)
        self.port = free_port()
        self.environment = dict(os.environ, ROS_MASTER_URI='http://127.0.0.1:'+str(self.port),
                                ROS_HOME=str(self.root/'ros'), ROS_LOG_DIR=str(self.root/'logs'),
                                ROS_IP='127.0.0.1')
        self.environment.pop('ROS_HOSTNAME', None)
        self.processes = []
        self.files = []
        self.master = None

    def tearDown(self):
        for process in reversed(self.processes):
            stop(process)
        if self.master is not None:
            self.master.close()
        for stream in self.files:
            stream.close()
        self.directory.cleanup()

    def fixture_process(self, topics=False, fail_registration=False):
        log = (self.root/'fixture.log').open('w+')
        self.files.append(log)
        process = subprocess.Popen([self.fixture]+(['--topics'] if topics else [])+
            (['--fail-registration'] if fail_registration else []),
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=log, env=self.environment, text=True)
        self.processes.append(process)
        return process, log

    def status(self, process, command='status'):
        self.assertIsNone(process.poll())
        process.stdin.write(command+'\n')
        process.stdin.flush()
        self.assertTrue(select.select([process.stdout], [], [], 3)[0], 'fixture main loop blocked')
        line = process.stdout.readline()
        self.assertTrue(line, 'fixture exited: '+str(process.poll()))
        return json.loads(line)

    def connected(self, process, timeout=35):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            value = self.status(process)
            if value['first'] == value['second'] == 1:
                self.assertEqual(value['cancelled'], 0)
                return value
            time.sleep(.05)
        self.fail('late master did not register the waiting nodes')

    def down_window(self, process, log, seconds):
        before = resources(process)
        began = time.monotonic()
        time.sleep(seconds)
        elapsed = time.monotonic()-began
        after = resources(process)
        status = self.status(process)
        log.seek(0)
        lines = log.read().splitlines()
        self.assertTrue(lines)
        self.assertIn('0 suppressed attempts', lines[0])
        cpu = after['cpu']-before['cpu']
        self.assertLessEqual(cpu, max(.02, elapsed*.02))
        self.assertLessEqual(after['threads'], before['threads'])
        self.assertLessEqual(after['fds'], before['fds'])
        self.assertLessEqual(status['attempts'], 18 if seconds == 60 else 8)
        self.assertLessEqual(len(lines), 7 if seconds == 60 else 2)
        print('PEER_DOWN '+json.dumps({'seconds': elapsed, 'cpu_seconds': cpu,
              'attempts': status['attempts'], 'log_lines': len(lines),
              'before': before, 'after': after}), flush=True)
        return status

    def test_late_master_single_registration_and_replacement_exit(self):
        began = time.monotonic()
        process, log = self.fixture_process()
        initial = self.status(process, 'set 20')
        self.assertLess(time.monotonic()-began, 1)
        self.assertEqual((initial['first'], initial['second'], initial['cancelled']), (0, 0, 0))
        status = self.down_window(process, log, 60 if self.long else .3)
        self.assertEqual(status['first'], 0)
        self.master = FakeMaster(self.port)
        status = self.connected(process)
        self.assertEqual(status['registration_attempts'], 1)
        time.sleep(.2)
        self.assertEqual(self.status(process)['first'], 1)
        self.master.close()
        self.master = FakeMaster(self.port, identity=2)
        self.assertEqual(process.wait(timeout=15), 0)
        log.seek(0)
        self.assertIn('ROS master replaced', log.read())

    def test_unresponsive_master_does_not_block_main_loop_or_shutdown(self):
        self.master = FakeMaster(self.port, stalled=True)
        process, _ = self.fixture_process()
        time.sleep(.1)
        self.assertEqual(self.status(process)['first'], 0)
        began = time.monotonic()
        process.stdin.write('stop\n')
        process.stdin.flush()
        self.assertEqual(process.wait(timeout=2), 0)
        self.assertLess(time.monotonic()-began, 1)

    def test_registration_loss_is_retried_without_replaying_completed_nodes(self):
        process, _ = self.fixture_process(fail_registration=True)
        self.assertEqual(self.status(process)['first'], 0)
        self.master = FakeMaster(self.port)
        status = self.connected(process)
        self.assertEqual(status['registration_attempts'], 2)
        time.sleep(.2)
        self.assertEqual(self.status(process)['first'], 1)

    def test_topics_deliver_latest_revision_to_late_and_restarted_consumers(self):
        process, log = self.fixture_process(topics=True)
        for revision in range(1, 21):
            self.status(process, 'set '+str(revision))
        master_log = (self.root/'master.log').open('w+')
        self.files.append(master_log)
        master = subprocess.Popen(['rosmaster', '--core', '-p', str(self.port)],
            env=self.environment, stdout=master_log, stderr=subprocess.STDOUT)
        self.processes.append(master)
        deadline = time.monotonic()+10
        while True:
            try:
                with ServerProxy(self.environment['ROS_MASTER_URI']) as peer:
                    peer.setParam('/fixture', '/run_id', 'fixture-run')
                break
            except OSError:
                self.assertLess(time.monotonic(), deadline)
                time.sleep(.05)
        self.connected(process)
        consumer_code = '''import json, threading, time, rospy
from std_msgs.msg import UInt64
rospy.init_node('late_consumer', anonymous=True, disable_rostime=True)
received = []; done = threading.Event()
def frame(value):
    received.append(value.data); done.set()
subscriber = rospy.Subscriber('/fixture/latest', UInt64, frame, queue_size=1)
assert done.wait(10), 'no latest sample'
time.sleep(.2)
print(json.dumps(received), flush=True)
subscriber.unregister()
rospy.signal_shutdown('consumer finished')
'''
        for iteration in range(2):
            if iteration:
                before_status = self.status(process)
                before = resources(process)
                began = time.monotonic()
                time.sleep(60 if self.long else .3)
                elapsed = time.monotonic()-began
                after = resources(process)
                after_status = self.status(process)
                attempts = after_status['attempts']-before_status['attempts']
                self.assertLessEqual(attempts, 7)
                self.assertEqual((after_status['first'], after_status['second']), (1, 1))
                self.assertLessEqual(after['cpu']-before['cpu'], max(.02, elapsed*.02))
                self.assertLessEqual(after['threads'], before['threads'])
                self.assertLessEqual(after['fds'], before['fds'])
                log.seek(0)
                lines = len(log.read().splitlines())
                self.assertLessEqual(lines, 7)
                print('CONSUMER_DOWN '+json.dumps({'seconds': elapsed, 'before': before, 'after': after,
                    'master_identity_reads': attempts, 'log_lines': lines}), flush=True)
            consumer = subprocess.Popen(['python3', '-c', consumer_code], env=self.environment,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            try:
                output, errors = consumer.communicate(timeout=15)
                self.assertEqual(consumer.returncode, 0, errors)
                self.assertEqual(json.loads(output), [20])
            finally:
                stop(consumer)
                consumer.stderr.close()
        self.assertEqual(self.status(process)['first'], 1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fixture', required=True)
    parser.add_argument('--long', action='store_true')
    args, remaining = parser.parse_known_args()
    ParallelStartTest.fixture = str(Path(args.fixture).resolve(strict=True))
    ParallelStartTest.long = args.long
    unittest.main(argv=[__file__]+remaining)
