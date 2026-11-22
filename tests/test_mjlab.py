"""Static + optional runtime tests for the mjlab 1.6.1 plugin."""
import importlib.util
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
XML = ROOT / "src/micro_wl_mjlab/models/micro_wheeled_leg.xml"

class TestStatic(unittest.TestCase):
    def test_xml_is_entity_only(self):
        root = ET.parse(XML).getroot()
        self.assertEqual(len(root.findall("./actuator/*")), 4)
        self.assertEqual(len(root.findall("./worldbody/body/freejoint")), 1)
        self.assertEqual(len(root.findall("./worldbody/geom")), 0)
        names = {e.get("name") for e in root.findall(".//joint")}
        for actuator in root.findall("./actuator/*"):
            self.assertIn(actuator.get("joint"), names)

    def test_package_entrypoint(self):
        # TOML is standard library on Python>=3.11.
        import tomllib
        with (ROOT / "pyproject.toml").open("rb") as f:
            meta = tomllib.load(f)
        self.assertEqual(meta["project"]["entry-points"]["mjlab.tasks"]["micro-wheeled-leg"], "micro_wl_mjlab")
        self.assertIn("mjlab==1.6.1", meta["project"]["dependencies"])

@unittest.skipUnless(importlib.util.find_spec("mujoco") is not None, "MuJoCo is not installed")
class TestMujoco(unittest.TestCase):
    def test_model_compile(self):
        import mujoco
        m = mujoco.MjModel.from_xml_path(str(XML))
        self.assertEqual(m.nu, 4)

@unittest.skipUnless(importlib.util.find_spec("mjlab") is not None, "mjlab is not installed")
class TestMjlabRegistration(unittest.TestCase):
    def test_registry_and_cfg(self):
        import micro_wl_mjlab
        from mjlab.tasks.registry import list_tasks, load_env_cfg
        self.assertIn(micro_wl_mjlab.TASK_ID, list_tasks())
        cfg = load_env_cfg(micro_wl_mjlab.TASK_ID)
        self.assertEqual(cfg.scene.num_envs, 1)
        self.assertEqual(tuple(cfg.actions), ("wheels", "hips"))
        self.assertEqual(cfg.decimation, 5)

if __name__ == "__main__":
    unittest.main()
