"""Batched classical balance baseline.

Action convention: wheel effort / 0.035 Nm, hip angle / 0.70 rad.
All controller gains and physical model parameters are provisional.
"""
import torch

class ClassicalController:
    WHEEL_TAU_MAX = 0.035
    HIP_ACTION_SCALE = 0.70

    def __init__(self, env, speed: float = 0.0, turn: float = 0.0, crouch: float = 0.0):
        if not 0.0 <= crouch <= 1.0:
            raise ValueError("crouch must be between 0 and 1")
        self.env = env
        self.speed_cmd = speed
        self.turn_cmd = turn
        self.crouch = crouch
        self.distance = torch.zeros(env.num_envs, device=env.device)
        self.target_distance = torch.zeros_like(self.distance)
        self.dt = env.cfg.sim.mujoco.timestep * env.cfg.decimation

    def reset(self):
        self.distance.zero_()
        self.target_distance.zero_()

    def __call__(self, obs) -> torch.Tensor:
        del obs  # Use physical state directly, not the RL observation ordering.
        robot = self.env.scene["robot"]
        g = robot.data.projected_gravity_b
        omega = robot.data.root_link_ang_vel_b
        v = robot.data.root_link_lin_vel_b
        pitch = torch.atan2(g[:, 0], -g[:, 2])
        roll = torch.atan2(-g[:, 1], -g[:, 2])
        vel = v[:, 0]

        # Detect environment resets in the viewer (episode_length_buf returns to 0).
        reset_mask = self.env.episode_length_buf == 0
        self.distance[reset_mask] = 0.0
        self.target_distance[reset_mask] = 0.0

        self.distance += vel * self.dt
        self.target_distance += self.speed_cmd * self.dt
        u = (
            0.048 * pitch
            + 0.0030 * omega[:, 1]
            + 0.0034 * (self.distance - self.target_distance)
            + 0.0046 * (vel - self.speed_cmd)
        )
        yaw = 0.003 * (self.turn_cmd - omega[:, 2])

        tl = torch.clamp((u - yaw) / self.WHEEL_TAU_MAX, -1.0, 1.0)
        tr = torch.clamp((u + yaw) / self.WHEEL_TAU_MAX, -1.0, 1.0)

        nominal = 0.12 + 0.55 * self.crouch
        delta = torch.clamp(-0.50 * roll - 0.035 * omega[:, 0], -0.20, 0.20)
        hl = torch.clamp((nominal + delta) / self.HIP_ACTION_SCALE, -1.0, 1.0)
        hr = torch.clamp((-nominal + delta) / self.HIP_ACTION_SCALE, -1.0, 1.0)
        return torch.stack((tl, tr, hl, hr), dim=-1)
