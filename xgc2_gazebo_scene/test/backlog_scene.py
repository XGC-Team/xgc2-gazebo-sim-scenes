#!/usr/bin/env python3
import unittest
from pathlib import Path
import xml.etree.ElementTree as ET
import time
import rospy
import rostest
from gazebo_msgs.msg import ModelState, ContactsState
from gazebo_msgs.srv import GetModelState, SetModelState
from std_srvs.srv import Empty

class SceneBacklog(unittest.TestCase):
    def test_fractional_time_and_original_person_torso(self):
        directory=Path(__file__).resolve().parent
        original=ET.parse(directory.parents[1]/'gazebo_sim_worlds/worlds/simple_box_dynamic_3/simple_box_dynamic_3.world')
        fixture=ET.parse(directory/'backlog_scene.world')
        # Fixtures preserve each production collision verbatim. Freeze motion
        # and omit visuals only to isolate the torso-height contact contract.
        for model in original.findall('.//world/model')[:3]:
            actual=fixture.find(".//model[@name='%s']"%model.attrib['name'])
            self.assertEqual(ET.tostring(model.find('link/collision')),ET.tostring(actual.find('link/collision')))
        rospy.wait_for_service('/gazebo/unpause_physics',timeout=20)
        contacts=[[],[],[]]
        subscribers=[rospy.Subscriber('/probe%d/contacts'%i,ContactsState,lambda m,i=i:contacts[i].append(m.states)) for i in range(3)]
        rospy.ServiceProxy('/gazebo/unpause_physics',Empty)()
        state=rospy.ServiceProxy('/gazebo/get_model_state',GetModelState)
        deadline=rospy.Time.from_sec(.75)
        while rospy.Time.now()<deadline:rospy.sleep(.001)
        value=state('fractional_path','world')
        self.assertTrue(value.success)
        self.assertAlmostEqual(value.pose.position.x,.375,delta=.015)
        rospy.sleep(.25)
        for samples in contacts:
            self.assertGreater(len(samples),5)
            self.assertFalse(any(samples))
        setter=rospy.ServiceProxy('/gazebo/set_model_state',SetModelState)
        for i in range(3):
            pose=ModelState();pose.model_name='probe%d'%i;pose.pose.orientation.w=1
            pose.pose.position.x=.30;pose.pose.position.y=i*2;pose.pose.position.z=.9
            self.assertTrue(setter(pose).success)
        rospy.sleep(.3)
        for i,samples in enumerate(contacts):
            hits=[hit for sample in samples for hit in sample]
            self.assertTrue(hits,'original person %d must contact at torso height'%i)
            self.assertTrue(any('person' in hit.collision1_name or 'person' in hit.collision2_name for hit in hits))
        for subscriber in subscribers:subscriber.unregister()

if __name__=='__main__':
    rospy.init_node('backlog_scene')
    rostest.rosrun('xgc2_gazebo_scene','backlog_scene',SceneBacklog)
