#!/usr/bin/env python3
"""Parse every local launch composition and check peer-independent processes."""
import json
from pathlib import Path
import re
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]


class LaunchStartOrderTest(unittest.TestCase):
    def test_launch_composition_has_no_peer_ordering_hacks(self):
        paths = sorted(list(ROOT.rglob('*.launch')) + list(ROOT.rglob('*.test')))
        self.assertTrue(paths)
        for path in paths:
            with self.subTest(path=str(path.relative_to(ROOT))):
                for node in ET.parse(path).getroot().iter('node'):
                    self.assertNotEqual(node.get('required', 'false').lower(), 'true')
                    prefix = node.get('launch-prefix', '')
                    self.assertIsNone(re.search(r'\b(sleep|wait|until)\b', prefix), prefix)
                    self.assertIsNone(re.search(r'\bsleep\s+\d', node.get('args', '')))

    def test_process_readiness_uses_only_the_owned_endpoint(self):
        for path in sorted((ROOT/'process-definitions').glob('*.json')):
            with self.subTest(path=path.name):
                for definition in json.loads(path.read_text())['definitions']:
                    readiness = definition['readiness']
                    self.assertEqual(readiness['kind'], 'none' if definition['id'] == 'gazebo-client' else 'describe')
                    self.assertFalse(set(readiness) & {'masterUri', 'address'})
                    command = definition['command']
                    self.assertFalse(any(re.search(r'\b(sleep|wait-for|roscore)\b', argument)
                                         for argument in command['args']))
                    if definition['id'] != 'gazebo-client':
                        self.assertEqual(command['env']['ROS_MASTER_URI'], '${rosMasterUri}')
                        self.assertEqual(definition['services'][0]['endpointParameter'], 'socketPath')


if __name__ == '__main__':
    unittest.main()
