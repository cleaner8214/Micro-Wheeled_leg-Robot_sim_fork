"""MuJoCo loader/step smoke test; skipped if MuJoCo is not installed."""
import math
import unittest
from pathlib import Path
try:
    import mujoco
except ImportError:
    mujoco = None

MODEL = Path(__file__).resolve().parents[1] / "models" / "micro_wheeled_leg.xml"

@unittest.skipUnless(mujoco is not None, "requires MuJoCo 4.0.0")
class TestPhysics(unittest.TestCase):
    def test_load_and_step(self):
        model = mujoco.MjModel.from_xml_path(str(MODEL))
        self.assertEqual(model.nu, 4)
        data = mujoco.MjData(model)
        data.ctrl[:] = (0.0, 0.0, 0.12, -0.12)
        for _ in range(100):
            mujoco.mj_step(model, data)
        self.assertTrue(math.isfinite(float(data.qpos[2])))
        self.assertGreater(float(data.time), 0.0)

if __name__ == "__main__":
    unittest.main()
