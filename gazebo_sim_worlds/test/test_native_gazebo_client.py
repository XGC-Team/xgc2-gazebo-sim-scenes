import importlib.util
import json
from pathlib import Path
import unittest
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('native_gazebo_client', ROOT/'scripts/native_gazebo_client.py')
entry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(entry)


class NativeGazeboClientTest(unittest.TestCase):
    def test_local_master_pins_transport_only(self):
        for uri in ('http://127.0.0.1:11345', 'http://127.9.8.7:11345',
                    'http://localhost:11345', 'http://LOCALHOST:11345', 'http://[::1]:11345'):
            with self.subTest(uri=uri):
                original = {'GAZEBO_MASTER_URI': uri, 'IGN_IP': '172.18.0.1',
                            'IGN_PARTITION': 'selected', 'IGN_RELAY': 'selected-relay',
                            'ROS_IP': '192.168.51.4'}
                argv, env = entry.compose(['-g', '/private/rendering.so', '--verbose'], original)
                self.assertEqual(argv, ['gzclient', '-g', '/private/rendering.so', '--verbose'])
                self.assertEqual(env, dict(original, IGN_IP='127.0.0.1'))
                self.assertEqual(original['IGN_IP'], '172.18.0.1')

    def test_remote_master_preserves_explicit_local_interface_and_unset_state(self):
        for uri in ('http://192.168.51.9:11345', 'http://remote-host:11345', 'http://[2001:db8::1]:11345'):
            for interface in (None, '192.168.51.4'):
                with self.subTest(uri=uri, interface=interface):
                    original = {'GAZEBO_MASTER_URI': uri}
                    if interface is not None:
                        original['IGN_IP'] = interface
                    _, env = entry.compose([], original)
                    self.assertEqual(env, original)
                    self.assertNotEqual(env.get('IGN_IP'), uri)

    def test_native_default_master_is_local(self):
        _, env = entry.compose([], {})
        self.assertEqual(env, {'IGN_IP': '127.0.0.1'})

    def test_invalid_uri_stays_with_existing_gzclient_error_handling(self):
        for uri in ('', 'not-a-uri', 'http://[malformed'):
            original = {'GAZEBO_MASTER_URI': uri, 'IGN_IP': 'selected'}
            self.assertEqual(entry.compose([], original)[1], original)

    def test_entry_execs_same_client_with_selected_argv_and_environment(self):
        with mock.patch.object(entry.os, 'environ', {'GAZEBO_MASTER_URI': 'http://127.0.0.1:11345'}), \
                mock.patch.object(entry.os, 'execvpe') as execute:
            entry.main(['-g', '/private/rendering.so'])
            execute.assert_called_once_with('gzclient', ['gzclient', '-g', '/private/rendering.so'],
                                            {'GAZEBO_MASTER_URI': 'http://127.0.0.1:11345', 'IGN_IP': '127.0.0.1'})

    def test_definition_and_install_keep_existing_gui_contract(self):
        definition = json.loads((ROOT.parent/'process-definitions/gazebo-client.json').read_text())['definitions'][0]
        self.assertEqual(definition['command']['executable'], '${rosRuntimePrefix}/lib/gazebo_sim_worlds/native_gazebo_client.py')
        self.assertEqual(definition['command']['args'], ['-g', '${rosRuntimePrefix}/lib/libxgc2_gazebo_rendering.so'])
        self.assertEqual(definition['readiness'], {'kind': 'none'})
        self.assertNotIn('IGN_IP', definition['command']['env'])
        self.assertEqual(definition['setupFiles'], ['${rosRuntimePrefix}/setup.bash'])
        cmake = (ROOT/'CMakeLists.txt').read_text()
        self.assertIn('scripts/native_gazebo_client.py', cmake)
        self.assertIn('test/test_native_gazebo_client.py', cmake)


if __name__ == '__main__':
    unittest.main()
