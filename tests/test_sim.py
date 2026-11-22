"""Basic model/controller tests without MuJoCo."""
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from sim.control import BalanceController, Command, RobotState

MODEL = Path(__file__).resolve().parents[1] / "models" / "micro_wheeled_leg.xml"

class TestStructure(unittest.TestCase):
    def test_four_actuators(self):
        root = ET.parse(MODEL)
        self.assertEqual(len(root.findall("./worldbody/body/freejoint")), 1)
        actuators = root.findall("./actuator/*")
        self.assertEqual(len(actuators), 4)
        self.assertEqual(
            {a.attrib["name"] for a in actuators},
            {"left_wheel_torque", "right_wheel_torque", "left_hip_target", "right_hip_target"},
        )
        joints = {n.attrib["name"] for n in root.findall(".//joint") if "name" in n.attrib}
        for actuator in actuators:
            self.assertIn(actuator.attrib["joint"], joints)

class TestControl(unittest.TestCase):
    def setUp(self):
        self.ctrl = BalanceController()
        self.state = RobotState(0, 0, 0, 0, 0, 0)

    def test_symmetry(self):
        a, b, c, d = self.ctrl.step(self.state, Command(), .001)
        self.assertEqual(a, b)
        self.assertAlmostEqual(c, -d)

    def test_pitch_recovery_direction(self):
        a, b, _, _ = self.ctrl.step(RobotState(.08, 0, 0, 0, 0, 0), Command(), .001)
        self.assertGreater(a, 0)
        self.assertGreater(b, 0)

    def test_differential_yaw(self):
        a, b, _, _ = self.ctrl.step(self.state, Command(yaw_rate=.2), .001)
        self.assertLess(a, b)

    def test_leg_crouch(self):
        _, _, a, b = self.ctrl.step(self.state, Command(crouch=1.0), .001)
        self.assertGreater(a, .6)
        self.assertLess(b, -.6)

    def test_saturation(self):
        a, b, c, d = self.ctrl.step(RobotState(10, 10, 1, 1, 1, 10), Command(), .001)
        self.assertLessEqual(abs(a), self.ctrl.torque_limit)
        self.assertLessEqual(abs(b), self.ctrl.torque_limit)
        self.assertLessEqual(abs(c), .90)
        self.assertLessEqual(abs(d), .90)

if __name__ == "__main__":
    unittest.main()
