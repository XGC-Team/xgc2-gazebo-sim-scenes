#!/usr/bin/env python3
"""Product placement envelope, identity and runtime-composition contracts."""
import copy
import importlib.machinery
import importlib.util
import io
import json
from pathlib import Path
import sys
import types
import tempfile
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'scenes/xgc2_gazebo_scene/python'))
sys.path.insert(0, str(ROOT / 'scenes/xgc2_simple_lidar/python'))
from xgc2_gazebo_scene.robot_placement import ground_model, read_placement


def row(kind='scout_mini'):
    return dict(kind=kind, slotId='slot-7', namespace='/vehicle_7',
                binding=dict(hybridSource='simulation', initialPose=dict(x=1.25, y=-2.5, z=0.2, yaw=0.75),
                             simulationSensors=dict(simpleLidar=False)))


def load_script(name, path):
    loader = importlib.machinery.SourceFileLoader(name, str(path))
    spec = importlib.util.spec_from_loader(name, loader)
    module = importlib.util.module_from_spec(spec)
    loader.exec_module(module)
    return module


class PlacementInputTest(unittest.TestCase):
    def test_complete_envelope_preserves_frozen_values(self):
        robot = dict(row(), namespace='/fleet/vehicle_7')
        actual, mode = read_placement(io.BytesIO(json.dumps(dict(robot=robot, runMode='hybrid')).encode()))
        self.assertEqual(actual, robot)
        self.assertEqual(mode, 'hybrid')

    def test_rejects_bare_robot_physical_member_and_unbounded_input(self):
        robot = row()
        cases = [robot, dict(robot=robot, runMode='physical'),
                 dict(robot=dict(robot, binding=dict(robot['binding'], hybridSource='physical')), runMode='hybrid')]
        for value in cases:
            with self.subTest(value=value), self.assertRaises(ValueError):
                read_placement(io.BytesIO(json.dumps(value).encode()))
        with self.assertRaises(ValueError):
            read_placement(io.BytesIO(b' ' * 65537))

    def test_ground_products_use_own_model_and_sensor_only_lowering(self):
        calls = []
        fake_xacro = types.SimpleNamespace(process_file=lambda path, mappings: calls.append((path, mappings)) or
                                           types.SimpleNamespace(toxml=lambda: '<sdf><model name="original"/></sdf>' if 'model.sdf.xacro' in path else '<robot name="original"/>'))
        for kind, path, fmt in [('scout_mini', ROOT/'agilex/scout/urdf/mini.xacro', 'urdf'),
                                ('mecanum_ugv', ROOT/'mecanum/models/xgc2_mecanum_ugv/model.sdf.xacro', 'sdf')]:
            robot = row(kind)
            original = copy.deepcopy(robot)
            with patch.dict(sys.modules, xacro=fake_xacro):
                result = ground_model(robot, 'hybrid', kind, path, fmt)
            self.assertEqual(result['format'], fmt)
            self.assertEqual(robot, original)
            received_path, mappings = calls[-1]
            self.assertEqual(received_path, str(path))
            self.assertEqual(mappings['chassis_robot_id'], 'slot-7')
            self.assertEqual(mappings['robot_namespace'], 'vehicle_7')
            self.assertEqual(mappings['enable_simple_lidar'], 'false')
            self.assertEqual(mappings['x'], '1.25')
            self.assertEqual(mappings['yaw'], '0.75')

    def test_scout_runtime_starts_publishers_without_model_spawn(self):
        runtime = load_script('scout_runtime', ROOT/'agilex/scout/scripts/start_scout_runtime')
        for mode in ('simulation', 'hybrid'):
            launch = ET.fromstring(runtime.launch_document(row(), mode, '<robot name="slot-7"/>'))
            nodes = launch.findall('.//node')
            self.assertEqual({node.attrib['type'] for node in nodes}, {'gazebo_model_ground_truth', 'sim_scout_status'})
            self.assertFalse(any('spawn' in node.attrib['type'] for node in nodes))
            self.assertEqual(launch.find('param').attrib['name'], '/vehicle_7/robot_description')
            for node in nodes:
                values = {p.attrib['name']: p.attrib['value'] for p in node.findall('param')}
                self.assertEqual(values['use_sim_time'], str(mode == 'simulation').lower())
            arguments = {a.attrib['name']: a.attrib['value'] for a in launch.findall('include/arg')}
            self.assertEqual(arguments['run_mode'], mode)
            self.assertEqual(arguments['robot_state_publisher_ns'], '/vehicle_7')

    def test_fs150_renderer_preserves_frozen_identity(self):
        sys.path.insert(0, str(ROOT/'fs150-sitl/scripts'))
        import render_fs150_indoor_sdf as renderer
        robot = row('px4_multirotor')
        robot['asset'] = dict(px4=dict(modelId='fs150'))
        robot['derived'] = dict(sitlInstanceId=6)
        with tempfile.TemporaryDirectory() as directory:
            description = Path(directory)
            (description/'urdf').mkdir()
            (description/'urdf/fs150_photo.urdf').write_text('<robot><link name="base_link"><collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision></link></robot>')
            (description/'models').mkdir()
            (description/'models/fixture.dae').write_text('<COLLADA/>')
            visual = ET.Element('robot')
            for name in ['base_link', 'rotor_0', 'rotor_1', 'rotor_2', 'rotor_3']:
                link = ET.SubElement(visual, 'link', name=name)
                ET.SubElement(ET.SubElement(ET.SubElement(link, 'visual'), 'geometry'), 'mesh', filename='package://fs150_description/models/fixture.dae')
                if name != 'base_link':
                    joint = ET.SubElement(visual, 'joint', name=name+'_joint', type='fixed')
                    ET.SubElement(joint, 'parent', link='base_link')
                    ET.SubElement(joint, 'child', link=name)
                    ET.SubElement(joint, 'origin', xyz='0 0 0', rpy='0 0 0')
            (description/'urdf/fs150_visual.urdf').write_text(ET.tostring(visual, encoding='unicode'))
            with patch.object(renderer, 'description_package', return_value=description):
                model = renderer.prepare_frozen_model(robot, ROOT/'fs150-sitl/models/fs150/iris.sdf')
        self.assertEqual(model['format'], 'sdf')
        sdf = ET.fromstring(model['content'])
        self.assertEqual(sdf.find('model').attrib['name'], 'slot-7')
        self.assertEqual(sdf.find('.//plugin[@name="mavlink_interface"]/mavlink_tcp_port').text, '4566')


if __name__ == '__main__':
    unittest.main()
