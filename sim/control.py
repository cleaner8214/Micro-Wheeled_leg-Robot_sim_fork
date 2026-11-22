"""Simplified wheel balance, yaw and symmetric leg-height feedback.

Positive pitch (about +Y) leans forward; positive wheel torque rolls forward.
Gains are initial guesses, NOT values identified from the physical robot.
"""
from dataclasses import dataclass

def clip(value: float, bound: float) -> float:
    return max(-bound, min(bound, value))

@dataclass
class RobotState:
    pitch: float
    pitch_rate: float
    roll: float
    roll_rate: float
    yaw_rate: float
    forward_speed: float

@dataclass
class Command:
    forward_speed: float = 0.0
    yaw_rate: float = 0.0
    crouch: float = 0.0

class BalanceController:
    """Four-state balance plus differential wheel and hip commands."""
    k_pitch = 0.048
    k_pitch_rate = 0.0030
    k_position = 0.0034
    k_velocity = 0.0046
    k_yaw_rate = 0.003
    k_roll = 0.50
    k_roll_rate = 0.035
    torque_limit = 0.035
    hip_delta_limit = 0.20

    def __init__(self) -> None:
        self.distance = 0.0
        self.target_distance = 0.0

    def step(self, state: RobotState, command: Command, dt: float) -> tuple[float, float, float, float]:
        if dt <= 0:
            raise ValueError("dt must be positive")
        crouch = max(0.0, min(1.0, command.crouch))
        self.distance += state.forward_speed * dt
        self.target_distance += command.forward_speed * dt
        balance = (
            self.k_pitch * state.pitch
            + self.k_pitch_rate * state.pitch_rate
            + self.k_position * (self.distance - self.target_distance)
            + self.k_velocity * (state.forward_speed - command.forward_speed)
        )
        turn = self.k_yaw_rate * (command.yaw_rate - state.yaw_rate)
        left_torque = clip(balance - turn, self.torque_limit)
        right_torque = clip(balance + turn, self.torque_limit)
        alpha = 0.12 + 0.55 * crouch
        roll_delta = clip(
            -self.k_roll * state.roll - self.k_roll_rate * state.roll_rate,
            self.hip_delta_limit,
        )
        left_hip = max(-0.90, min(0.90, alpha + roll_delta))
        right_hip = max(-0.90, min(0.90, -alpha + roll_delta))
        return left_torque, right_torque, left_hip, right_hip
