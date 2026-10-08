import importlib.machinery
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
            self.assertEqual(argv, [binary, '-u', '-s', 'libxgc2_simulation_ros_data.so', '-s',
                'libgazebo_sim_vrpn_system_plugin.so', str(world)])
            self.assertEqual(env['XGC_SIM_VRPN_CONFIG'], str(vrpn))
            self.assertEqual(env['XGC_SIM_ROS_NAMESPACE'], '/fixture')
            self.assertEqual(env['GAZEBO_PLUGIN_PATH'], directory)
            self.assertEqual(environment['XGC_SIM_VRPN_CONFIG'], '/stale')
            _, argv, env = entry.compose(['--world', str(world), '--paused', 'false', '--gui', 'false',
                '--ros-data', 'false', '--gazebo-bin', '/usr/bin/true'], environment)
            self.assertEqual(argv, [binary, str(world)])
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

if __name__ == '__main__': unittest.main()
