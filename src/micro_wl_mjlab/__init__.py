"""mjlab 1.6.1 plugin: registration occurs on import."""
from mjlab.tasks.registry import register_mjlab_task
from micro_wl_mjlab.task import make_env_cfg, make_runner_cfg

TASK_ID = "Mjlab-Micro-WheeledLeg-Balance"

register_mjlab_task(
    task_id=TASK_ID,
    env_cfg=make_env_cfg(),
    play_env_cfg=make_env_cfg(play=True),
    rl_cfg=make_runner_cfg(),
)
