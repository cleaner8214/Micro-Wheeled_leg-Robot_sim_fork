"""Native mjlab 1.6.1 manager-based task (independent external plugin)."""
from pathlib import Path
import math

import mujoco
import torch
from mjlab.actuator.xml_actuator import XmlActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs import mdp
from mjlab.envs.mdp.actions import JointEffortActionCfg, JointPositionActionCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.observation_manager import ObservationGroupCfg, ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.termination_manager import TerminationTermCfg
from mjlab.rl import RslRlModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg
from mjlab.scene import SceneCfg
from mjlab.sim import MujocoCfg, SimulationCfg
from mjlab.terrains import TerrainEntityCfg
from mjlab.viewer import ViewerConfig

MODEL_XML = Path(__file__).parent / "models" / "micro_wheeled_leg.xml"

def robot_spec() -> mujoco.MjSpec:
    return mujoco.MjSpec.from_file(str(MODEL_XML))

def robot_cfg() -> EntityCfg:
    return EntityCfg(
        spec_fn=robot_spec,
        articulation=EntityArticulationInfoCfg(
            actuators=(
                XmlActuatorCfg(target_names_expr=("left_wheel_joint", "right_wheel_joint")),
                XmlActuatorCfg(target_names_expr=("left_hip", "right_hip")),
            ),
        ),
        init_state=EntityCfg.InitialStateCfg(
            pos=(0.0, 0.0, 0.086),
            joint_pos={".*": 0.0},
            joint_vel={".*": 0.0},
        ),
    )

def upright_reward(env) -> torch.Tensor:
    gravity_b = env.scene["robot"].data.projected_gravity_b
    return torch.clamp(-gravity_b[:, 2], min=0.0, max=1.0)

def stationary_reward(env) -> torch.Tensor:
    vel = env.scene["robot"].data.root_link_lin_vel_b
    return torch.exp(-(vel[:, 0] / 0.10).square())

def action_penalty(env) -> torch.Tensor:
    return env.action_manager.action.square().sum(dim=-1)

def make_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
    terms = {
        "base_ang_vel": ObservationTermCfg(func=mdp.base_ang_vel),
        "projected_gravity": ObservationTermCfg(func=mdp.projected_gravity),
        "base_lin_vel": ObservationTermCfg(func=mdp.base_lin_vel),
        "joint_pos": ObservationTermCfg(func=mdp.joint_pos_rel),
        "joint_vel": ObservationTermCfg(func=mdp.joint_vel_rel),
        "last_action": ObservationTermCfg(func=mdp.last_action),
    }
    return ManagerBasedRlEnvCfg(
        scene=SceneCfg(
            terrain=TerrainEntityCfg(terrain_type="plane"),
            entities={"robot": robot_cfg()},
            num_envs=1,
            env_spacing=1.0,
        ),
        observations={
            "actor": ObservationGroupCfg(terms={**terms}, enable_corruption=False),
            "critic": ObservationGroupCfg(terms={**terms}, enable_corruption=False),
        },
        actions={
            "wheels": JointEffortActionCfg(
                entity_name="robot",
                actuator_names=("left_wheel_joint", "right_wheel_joint"),
                scale=0.035,
            ),
            "hips": JointPositionActionCfg(
                entity_name="robot",
                actuator_names=("left_hip", "right_hip"),
                scale=0.70,
                use_default_offset=False,
            ),
        },
        events={
            "reset_default": EventTermCfg(func=mdp.reset_scene_to_default, mode="reset"),
        },
        rewards={
            "upright": RewardTermCfg(func=upright_reward, weight=2.0),
            "stationary": RewardTermCfg(func=stationary_reward, weight=0.5),
            "action_penalty": RewardTermCfg(func=action_penalty, weight=-0.02),
        },
        terminations={
            "time_out": TerminationTermCfg(func=mdp.time_out, time_out=True),
            "fall": TerminationTermCfg(func=mdp.bad_orientation, params={"limit_angle": math.radians(60)}),
            "body_low": TerminationTermCfg(
                func=mdp.root_height_below_minimum,
                params={"minimum_height": 0.035},
            ),
        },
        viewer=ViewerConfig(
            origin_type=ViewerConfig.OriginType.ASSET_BODY,
            entity_name="robot",
            body_name="base",
            distance=0.45,
            elevation=-20.0,
            azimuth=140.0,
        ),
        sim=SimulationCfg(
            mujoco=MujocoCfg(timestep=0.002, iterations=20, ls_iterations=20),
        ),
        decimation=5,
        episode_length_s=10.0 if not play else 60.0,
        auto_reset=True,
    )

def make_runner_cfg() -> RslRlOnPolicyRunnerCfg:
    """A preliminary PPO baseline, not a verified trained policy."""
    return RslRlOnPolicyRunnerCfg(
        actor=RslRlModelCfg(
            hidden_dims=(128, 128),
            activation="elu",
            obs_normalization=True,
            distribution_cfg={
                "class_name": "GaussianDistribution",
                "init_std": 0.3,
                "std_type": "scalar",
            },
        ),
        critic=RslRlModelCfg(
            hidden_dims=(128, 128),
            activation="elu",
            obs_normalization=True,
        ),
        algorithm=RslRlPpoAlgorithmCfg(
            learning_rate=3.0e-4,
            entropy_coef=0.001,
            gamma=0.99,
            lam=0.95,
            num_learning_epochs=5,
            num_mini_batches=4,
            desired_kl=0.02,
        ),
        experiment_name="micro_wl_balance",
        num_steps_per_env=24,
        max_iterations=3000,
        save_interval=100,
    )
