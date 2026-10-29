import importlib.machinery
import json
import yaml
import importlib.util
from pathlib import Path
import tempfile
import unittest

path = Path(__file__).resolve().parents[1]/'scripts/native_world_start'
loader = importlib.machinery.SourceFileLoader('native_world_start', str(path))
spec = importlib.util.spec_from_loader(loader.name, loader)
entry = importlib.util.module_from_spec(spec); loader.exec_module(entry)

WORLD = '<sdf version="1.6"><world name="fixture"><plugin name="native" filename="libxgc2_simulation_world.so"><socket_path>/private/world.sock</socket_path><target_id>fixture</target_id><resource_root>/private/resources</resource_root></plugin></world></sdf>'

class NativeWorldStartTest(unittest.TestCase):
    def test_explicit_world_exec_and_user_data_plugins(self):
        with tempfile.TemporaryDirectory() as directory:
            world = Path(directory)/'world.sdf'; world.write_text(WORLD)
            vrpn = Path(directory)/'vrpn.yaml'; vrpn.write_text('target: fixture\n')
            environment = {'PATH': '/usr/bin:/bin', 'XGC_SIM_ROS_NAMESPACE': '/stale', 'XGC_SIM_VRPN_CONFIG': '/stale'}
            binary, argv, env = entry.compose(['--world', str(world), '--paused', 'true', '--gui', 'false',
                '--ros-data', 'true', '--gazebo-bin', '/usr/bin/true', '--vrpn-config', str(vrpn),
                '--plugin-dir', directory, '__name:=fixture', '__ns:=/fixture'], environment)
            self.assertEqual(argv, [binary, '-u', '-s', 'libxgc2_simple_lidar_cpu.so', '-s', 'libxgc2_simple_lidar.so', '-s', 'libgazebo_ros_paths_plugin.so', '-s',
                'libxgc2_simulation_ros_data.so', '-s',
                'libgazebo_sim_vrpn_system_plugin.so', str(world)])
            self.assertEqual(env['XGC_SIM_VRPN_CONFIG'], str(vrpn))
            self.assertEqual(env['XGC_SIM_ROS_NAMESPACE'], '/fixture')
            self.assertEqual(env['GAZEBO_PLUGIN_PATH'], directory)
            self.assertEqual(environment['XGC_SIM_VRPN_CONFIG'], '/stale')
            _, argv, env = entry.compose(['--world', str(world), '--paused', 'false', '--gui', 'false',
                '--ros-data', 'false', '--gazebo-bin', '/usr/bin/true'], environment)
            self.assertEqual(argv, [binary, '-s', 'libxgc2_simple_lidar_cpu.so', '-s', 'libxgc2_simple_lidar.so', str(world)])
            self.assertNotIn('XGC_SIM_VRPN_CONFIG', env)
            self.assertNotIn('XGC_SIM_ROS_NAMESPACE', env)

    def test_unprepared_world_and_implicit_or_unknown_controls_fail(self):
        with tempfile.TemporaryDirectory() as directory:
            world = Path(directory)/'world.sdf'; world.write_text('<sdf><world name="bare"/></sdf>')
            args = ['--world', str(world), '--paused', 'false', '--gui', 'false', '--ros-data', 'false', '--gazebo-bin', '/usr/bin/true']
            with self.assertRaises(ValueError): entry.compose(args, {})
            world.write_text(WORLD)
            for extra in [['--probe'], ['__master:=http://station:11311'], ['__name:=one', '__name:=two'], ['--vrpn-config', 'relative.yaml']]:
                with self.subTest(extra=extra), self.assertRaises(ValueError): entry.compose(args+extra, {})

    def test_tracker_ports_override_is_explicit_and_strict(self):
        with tempfile.TemporaryDirectory() as directory:
            world = Path(directory)/'world.sdf'; world.write_text(WORLD)
            config = Path(directory)/'source.yaml'; config.write_text('port: 3884\ntracker_ports: {existing: 41000}\n')
            base = ['--world', str(world), '--paused', 'false', '--gui', 'false', '--ros-data', 'true',
                    '--gazebo-bin', '/usr/bin/true', '--vrpn-config', str(config)]
            _, _, env = entry.compose(base, {})
            self.assertEqual(env['XGC_SIM_VRPN_CONFIG'], str(config))
            _, _, env = entry.compose(base + ['--vrpn-tracker-ports', '{"uav1":41001,"uav2":41002}'], {})
            prepared = yaml.safe_load(Path(env['XGC_SIM_VRPN_CONFIG']).read_text())
            self.assertEqual(prepared['tracker_ports'], {'uav1': 41001, 'uav2': 41002})
            self.assertEqual(prepared['port'], 3884)
            _, _, env = entry.compose(base + ['--vrpn-tracker-ports', '{}'], {})
            self.assertEqual(yaml.safe_load(Path(env['XGC_SIM_VRPN_CONFIG']).read_text())['tracker_ports'], {})
            self.assertEqual(yaml.safe_load(config.read_text())['tracker_ports'], {'existing':41000})
            for raw in ('[]', '{"uav1":true}', '{"uav1":0}', '{"uav1":65536}', '{"uav1":1,"uav2":1}',
                        '{"uav1":1,"uav1":2}', '{"/uav1":1}', '{broken'):
                with self.subTest(raw=raw), self.assertRaises(ValueError):
                    entry.compose(base + ['--vrpn-tracker-ports', raw], {})
            with self.assertRaises(ValueError):
                entry.compose([*base[:-2], '--vrpn-tracker-ports', '{"uav1":41001}'], {})

    def test_gazebo_process_definitions_pass_the_json_map_as_one_argument(self):
        root = Path(__file__).resolve().parents[2] / 'process-definitions'
        for filename in ('gazebo-server.json', 'gazebo-server-hybrid.json'):
            definition = json.loads((root / filename).read_text())['definitions'][0]
            parameter = definition['parameters']['properties']['vrpnTrackerPorts']
            self.assertEqual(definition['parameters']['properties']['rosLogDir']['default'], '')
            self.assertEqual(parameter['type'], 'json')
            self.assertEqual(parameter['default'], '')
            args = definition['command']['args']; index = args.index('--vrpn-tracker-ports')
            self.assertEqual(args[index+1], '${vrpnTrackerPorts}')

    def test_allocated_socket_overrides_only_the_native_endpoint(self):
        with tempfile.TemporaryDirectory() as directory:
            world = Path(directory)/'world.sdf'; world.write_text(WORLD)
            socket_path = str(Path(directory)/'allocated.sock')
            args = ['--world', str(world), '--paused', 'false', '--gui', 'false',
                    '--ros-data', 'false', '--gazebo-bin', '/usr/bin/true']
            environment = {'XGC_SIM_SOCKET_PATH': '/stale.sock'}
            _, argv, env = entry.compose(args+['--socket-path', socket_path], environment)
            self.assertEqual(env['XGC_SIM_SOCKET_PATH'], socket_path)
            self.assertEqual(argv[-1], str(world))
            self.assertEqual(world.read_text(), WORLD)
            self.assertEqual(environment['XGC_SIM_SOCKET_PATH'], '/stale.sock')
            _, _, env = entry.compose(args, environment)
            self.assertNotIn('XGC_SIM_SOCKET_PATH', env)
            for invalid in ('', 'relative.sock', '/', '//private/world.sock',
                            '/private/../world.sock', '/'+('a'*107), '/private/\x00.sock'):
                with self.subTest(socket_path=invalid), self.assertRaises(ValueError):
                    entry.compose(args+['--socket-path', invalid], environment)

    def test_local_world_pins_ignition_without_repurposing_ros_interface(self):
        with tempfile.TemporaryDirectory() as directory:
            world = Path(directory)/'world.sdf'; world.write_text(WORLD)
            original = {'GAZEBO_MASTER_URI': 'http://127.0.0.1:11345',
                        'IGN_IP': '172.18.0.1', 'IGN_PARTITION': 'selected',
                        'IGN_RELAY': 'selected-relay', 'ROS_IP': '192.168.51.4'}
            args = ['--world', str(world), '--paused', 'false', '--gui', 'false',
                    '--ros-data', 'false', '--gazebo-bin', '/usr/bin/true']
            _, _, env = entry.compose(args, original)
            self.assertEqual(env['IGN_IP'], '127.0.0.1')
            for key in ('GAZEBO_MASTER_URI', 'IGN_PARTITION', 'IGN_RELAY', 'ROS_IP'):
                self.assertEqual(env[key], original[key])
            self.assertEqual(original['IGN_IP'], '172.18.0.1')

if __name__ == '__main__': unittest.main()
