"""Run the mjlab-native classical baseline or interactive viewer."""
import argparse
import csv
import math
from pathlib import Path

from mjlab.envs import ManagerBasedRlEnv
from mjlab.viewer import NativeMujocoViewer

from micro_wl_mjlab.controller import ClassicalController
from micro_wl_mjlab.task import make_env_cfg

def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="mjlab 1.6.1 wheeled-leg simulation")
    p.add_argument("--duration", type=float, default=8.0, help="simulated seconds (headless)")
    p.add_argument("--speed", type=float, default=0.0, help="forward body velocity target m/s")
    p.add_argument("--turn", type=float, default=0.0, help="yaw rate target rad/s")
    p.add_argument("--crouch", type=float, default=0.0, help="leg crouch fraction [0,1]")
    p.add_argument("--num-envs", type=int, default=1)
    p.add_argument("--device", default="cuda:0", help="cuda:0 or cpu")
    p.add_argument("--viewer", choices=["headless", "native"], default="headless")
    p.add_argument("--csv", type=Path, default=None, help="record env 0 each step")
    return p

def main() -> int:
    args = parser().parse_args()
    if args.duration <= 0 or args.num_envs <= 0 or not 0.0 <= args.crouch <= 1.0:
        raise SystemExit("duration/num-envs must be positive, crouch in [0,1]")
    cfg = make_env_cfg(play=True)
    cfg.scene.num_envs = args.num_envs
    cfg.episode_length_s = max(args.duration + 1.0, 60.0)
    cfg.auto_reset = args.viewer == "native"
    env = ManagerBasedRlEnv(cfg=cfg, device=args.device)
    try:
        controller = ClassicalController(env, speed=args.speed, turn=args.turn, crouch=args.crouch)
        obs, _ = env.reset()
        controller.reset()
        if args.viewer == "native":
            NativeMujocoViewer(env, controller).run()
            return 0

        dt = cfg.sim.mujoco.timestep * cfg.decimation
        num_steps = math.ceil(args.duration / dt)
        fout = None
        writer = None
        if args.csv is not None:
            args.csv.parent.mkdir(parents=True, exist_ok=True)
            fout = args.csv.open("w", encoding="utf-8", newline="")
            writer = csv.writer(fout)
            writer.writerow(["t_s", "pitch_deg", "roll_deg", "x_m", "height_m", "left_action", "right_action"])
        failed = False
        try:
            for step in range(num_steps):
                action = controller(obs)
                obs, reward, terminated, truncated, info = env.step(action)
                del reward, info
                robot = env.scene["robot"]
                g = robot.data.projected_gravity_b[0]
                pitch = math.degrees(math.atan2(g[0].item(), -g[2].item()))
                roll = math.degrees(math.atan2(-g[1].item(), -g[2].item()))
                pos = robot.data.root_link_pos_w[0]
                if writer:
                    writer.writerow([
                        f"{(step+1)*dt:.4f}", f"{pitch:.3f}", f"{roll:.3f}",
                        f"{pos[0].item():.5f}", f"{pos[2].item():.5f}",
                        f"{action[0,0].item():.5f}", f"{action[0,1].item():.5f}",
                    ])
                if bool((terminated | truncated).any().item()):
                    failed = True
                    print(f"terminated at t={(step+1)*dt:.3f}s")
                    break
            print(
                f"t={(step+1)*dt:.3f}s, pitch={pitch:.2f}deg, roll={roll:.2f}deg, "
                f"x={pos[0].item():.4f}m, z={pos[2].item():.4f}m, "
                f"status={'TERMINATED' if failed else 'FINISHED'}"
            )
        finally:
            if fout is not None:
                fout.close()
        return 1 if failed else 0
    finally:
        env.close()

if __name__ == "__main__":
    raise SystemExit(main())
