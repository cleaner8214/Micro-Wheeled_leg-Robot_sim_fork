"""Run simplified two-wheel leg robot in MuJoCo 4.0.0."""
import argparse
import csv
import math
from pathlib import Path
import time

import mujoco
from sim.control import BalanceController, Command, RobotState

MODEL_PATH = Path(__file__).resolve().parents[1] / "models" / "micro_wheeled_leg.xml"

def observe(model: mujoco.MjModel, data: mujoco.MjData, body_id: int, root_id: int) -> RobotState:
    rot = data.xmat[body_id].reshape(3, 3)
    pitch = math.atan2(float(rot[0, 2]), float(rot[2, 2]))
    roll = math.atan2(-float(rot[1, 2]), float(rot[2, 2]))
    gyro = data.sensor("base_gyro").data
    start = model.jnt_dofadr[root_id]
    vel = data.qvel[start:start + 3]
    forward = rot[:2, 0]
    forward_speed = float(vel[0] * forward[0] + vel[1] * forward[1])
    return RobotState(
        pitch=pitch, pitch_rate=float(gyro[1]),
        roll=roll, roll_rate=float(gyro[0]),
        yaw_rate=float(gyro[2]), forward_speed=forward_speed,
    )

def run(args: argparse.Namespace) -> int:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base")
    root_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
    ctrl = BalanceController()
    cmd = Command(forward_speed=args.speed, yaw_rate=args.turn_rate, crouch=args.crouch)
    period = max(1, int(round(0.05 / model.opt.timestep)))
    steps = int(math.ceil(args.duration / model.opt.timestep))
    logfile = None
    writer = None
    if args.csv is not None:
        args.csv.parent.mkdir(parents=True, exist_ok=True)
        logfile = args.csv.open("w", encoding="utf-8", newline="")
        writer = csv.writer(logfile)
        writer.writerow(["t_s", "pitch_deg", "roll_deg", "x_m", "z_m", "speed_mps", "tau_left_Nm", "tau_right_Nm"])
    fallen = False
    latest = None
    viewer = None
    try:
        if args.render:
            from mujoco import viewer as mj_viewer
            viewer = mj_viewer.launch_passive(model, data)
        for i in range(steps):
            state = observe(model, data, base_id, root_id)
            tl, tr, hl, hr = ctrl.step(state, cmd, model.opt.timestep)
            if args.no_balance:
                tl, tr = 0.0, 0.0
            data.ctrl[:] = (tl, tr, hl, hr)
            data.xfrc_applied[base_id, :] = 0.0
            if args.push_force and 2.0 <= data.time < 2.1:
                data.xfrc_applied[base_id, 0] = args.push_force
            mujoco.mj_step(model, data)
            if i % period == 0 or i == steps - 1:
                latest = observe(model, data, base_id, root_id)
                if writer is not None:
                    writer.writerow([
                        f"{data.time:.4f}", f"{math.degrees(latest.pitch):.3f}",
                        f"{math.degrees(latest.roll):.3f}", f"{data.qpos[0]:.4f}",
                        f"{data.qpos[2]:.4f}", f"{latest.forward_speed:.4f}",
                        f"{tl:.5f}", f"{tr:.5f}",
                    ])
            if viewer is not None and i % 10 == 0:
                if not viewer.is_running():
                    break
                viewer.sync()
                time.sleep(model.opt.timestep * 10)
            if abs(state.pitch) > math.radians(60) or data.qpos[2] < 0.035:
                fallen = True
                break
    finally:
        if viewer is not None:
            viewer.close()
        if logfile is not None:
            logfile.close()
    if latest is None:
        latest = observe(model, data, base_id, root_id)
    print(
        f"time={data.time:.2f}s pitch={math.degrees(latest.pitch):.2f}deg "
        f"roll={math.degrees(latest.roll):.2f}deg "
        f"x={data.qpos[0]:.3f}m height={data.qpos[2]:.3f}m "
        f"status={'FALLEN' if fallen else 'FINISHED'}"
    )
    return int(fallen)

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Minimal 4-actuator MuJoCo wheeled-leg simulation")
    p.add_argument("--duration", type=float, default=8.0, help="simulated seconds")
    p.add_argument("--speed", type=float, default=0.0, help="forward m/s")
    p.add_argument("--turn-rate", type=float, default=0.0, help="yaw rad/s")
    p.add_argument("--crouch", type=float, default=0.0, help="crouch factor [0,1]")
    p.add_argument("--push-force", type=float, default=0.0, help="external X force 2.0–2.1s, N")
    p.add_argument("--no-balance", action="store_true", help="open-loop wheel comparison")
    p.add_argument("--render", action="store_true", help="launch graphical viewer")
    p.add_argument("--headless", action="store_true", help="run without viewer (default)")
    p.add_argument("--csv", type=Path, help="CSV output path")
    a = p.parse_args()
    if a.duration <= 0: p.error("--duration must be positive")
    if not 0 <= a.crouch <= 1: p.error("--crouch must be in [0,1]")
    if a.render and a.headless: p.error("--render and --headless are exclusive")
    return a

if __name__ == "__main__":
    raise SystemExit(run(parse_args()))
