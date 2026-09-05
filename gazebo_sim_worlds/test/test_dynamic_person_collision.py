#!/usr/bin/env python3
"""Asset geometry checks, not a substitute for Gazebo physics contacts."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]


class DynamicPersonCollisionTest(unittest.TestCase):
    def test_all_three_people_have_torso_collision(self):
        world = ET.parse(ROOT / 'worlds/simple_box_dynamic_3/simple_box_dynamic_3.world').getroot()
        people = [model for model in world.findall('world/model') if model.get('name', '').startswith('person')]
        self.assertEqual({p.get('name') for p in people},
                         {f'person{i}_0.5_0.5_1.8' for i in (1, 2, 3)})
        for person in people:
            with self.subTest(person=person.get('name')):
                collision = person.find('link/collision')
                size = list(map(float, collision.findtext('geometry/box/size').split()))
                pose = list(map(float, collision.findtext('pose').split()))
                self.assertEqual(size, [0.35, 0.75, 1.8])
                self.assertAlmostEqual(pose[2] - size[2] / 2, 0.0)
                self.assertAlmostEqual(pose[2] + size[2] / 2, 1.8)
                visual = person.find('link/visual')
                visual_pose = list(map(float, visual.findtext('pose').split()))
                self.assertEqual(pose[5], visual_pose[5])
                self.assertEqual(visual.findtext('geometry/mesh/uri'), 'model://person/meshes/walking.dae')

    def test_plugin_uses_checked_fractional_timing(self):
        source = (ROOT.parent / 'xgc2_gazebo_scene/src/obstacle_path_plugin.cpp').read_text()
        self.assertNotIn('int duration = distance', source)
        self.assertIn('SegmentDuration(distance, this->velocity, &duration)', source)
        self.assertIn('SegmentDuration(angleABSDiff, this->angularVelocity, &duration)', source)
        self.assertIn('UniqueKeyframeIndices(this->timeKnot, &keyframe_indices)', source)
        self.assertLess(source.index('this->path.size() < 2'), source.index('this->path.push_back(this->path[0])'))


if __name__ == '__main__':
    unittest.main()
