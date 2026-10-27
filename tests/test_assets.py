"""Static contract checks catch frame/joint/config drift without a ROS installation."""
import json
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SIM = ROOT / 'src/sort_place_sim'


class AssetTests(unittest.TestCase):
    def test_package_xml(self):
        for package in (ROOT / 'src').iterdir():
            xml = ET.parse(package / 'package.xml').getroot()
            self.assertEqual(xml.findtext('name'), package.name)

    def test_scene_matches_config(self):
        config = json.loads((SIM / 'config/cell.json').read_text())
        world = ET.parse(SIM / 'worlds/cell.sdf').getroot().find('world')
        self.assertEqual(world.get('name'), 'sort_cell')
        models = {m.get('name'): m for m in world.findall('model')}
        for part in config['parts']:
            model = models[part['id']]
            pose = [float(v) for v in model.findtext('pose').split()]
            self.assertEqual(pose[:3], part['pose'][:3])
            self.assertEqual(pose[5], part['pose'][3])
            self.assertEqual(model.findtext('static'), 'false')
        for slot in config['slots']:
            self.assertIn('slot_' + slot['id'], models)

    def test_urdf_joints_and_tcp_offsets(self):
        robot = ET.parse(SIM / 'urdf/scara.urdf.xacro').getroot()
        joints = {j.get('name'): j for j in robot.findall('joint')}
        self.assertEqual([joints[f'joint{i}'].get('type') for i in range(1, 5)],
                         ['revolute', 'revolute', 'prismatic', 'revolute'])
        self.assertAlmostEqual(sum(float(joints[name].find('origin').get('xyz').split()[2])
                                   for name in ['joint1', 'joint2', 'joint3', 'joint4']), .96)
        self.assertEqual(joints['joint4'].find('child').get('link'), 'nozzle_link')


if __name__ == '__main__':
    unittest.main()
