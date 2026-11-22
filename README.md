# Micro-Wheeled_leg-Robot · mjlab 1.6.1 仿真版

基于 [MuShibo/Micro-Wheeled_leg-Robot](https://github.com/MuShibo/Micro-Wheeled_leg-Robot) 的**纯仿真、最小可扩展实验工程**。对接 [mujocolab/mjlab](https://github.com/mujocolab/mjlab) **v1.6.1**；无需购买电机、电池、PCB，不需要 ESP32、Arduino 或 Isaac Sim。**不修改本地 mjlab 源码**。

> 复现边界：这里是**等效轮腿动力学模型**，而不是原项目 STEP 机械装配体的严格转化。当前左右腿各由一个摆动关节近似，**没有原机构的闭链约束**；几何、质量、惯量、轮胎接触、执行器上限和控制增益是工程初值。**代码入库不代表已经在 mjlab 1.6.1 物理运行稳定**，需要在你的机器上执行下述检查。

## 1. 工程结构

```text
Micro-Wheeled_leg-Robot_sim_fork/
├── pyproject.toml
├── README.md
├── src/micro_wl_mjlab/
│   ├── __init__.py                 # mjlab.tasks 插件注册
│   ├── task.py                     # Entity / Scene / MDP / PPO 配置
│   ├── controller.py               # 经典四状态平衡反馈（基线）
│   ├── run.py                      # headless / native viewer 入口
│   └── models/micro_wheeled_leg.xml # 简化 MJCF
└── tests/test_mjlab.py             # 静态 / XML / 任务注册检查
```

机器人的 4 个动作按顺序为：**左轮力矩、右轮力矩、左腿关节目标位置、右腿关节目标位置**。轮部是力矩执行器，左右腿是位置执行器。场景中地面由 mjlab 的 `TerrainEntityCfg` 提供，模型自身不重复定义地面。仿真步长 **0.002 s**，每 5 个物理步控制一次，**100 Hz 控制频率**。

## 2. 使用已有的 ~/mjlab v1.6.1 环境安装

本项目是一个**外部 mjlab 插件**，不需要重新安装一套 CUDA、PyTorch 或 mjlab。下面假定你已经在 `~/mjlab` 使用 uv 创建了 `.venv`。如果没有 `.venv`，先按你现有的 mjlab 工作流程执行 `cd ~/mjlab && uv sync`。

```bash
# 1) 检查已有 mjlab，不需要切换或修改它的 Git 分支
cd ~/mjlab
git describe --tags --always
uv run --no-sync python -c 'import importlib.metadata as m; print("mjlab", m.version("mjlab"))'
# 目标是已安装的 mjlab==1.6.1（Git 分支名不等于 Python 包版本）

# 2) 克隆本仿真分支（与 ~/mjlab 并列放置）
cd ~
git clone -b sim/mjlab-v1.6.1 \
  https://github.com/cleaner8214/Micro-Wheeled_leg-Robot_sim_fork.git

# 3) 仅将本项目安装为可编辑插件；复用 ~/mjlab/.venv，不更改 mjlab 源码或 uv.lock
cd ~/Micro-Wheeled_leg-Robot_sim_fork
uv pip install --python ~/mjlab/.venv/bin/python -e . --no-deps

# 4) 检查插件注册是否成功（注意 --no-sync）
cd ~/mjlab
uv run --no-sync list-envs --keyword Micro-WheeledLeg
```

应该出现 `Mjlab-Micro-WheeledLeg-Balance`。如果 `mjlab` 版本不是 1.6.1，**不要静默升级**；先核实 `~/mjlab/pyproject.toml`、`uv.lock` 和 `uv run --no-sync python` 指向的环境。

**为什么推荐 `--no-sync`？** 本项目通过 `uv pip install -e` 安装进已有虚拟环境，但不是 `~/mjlab/pyproject.toml` 的声明依赖。直接执行 `uv run` 可能重新同步 mjlab 环境并移除额外安装的插件；`--no-sync` 可以避免这种行为。如果你打算长期共同维护两个仓库，也可以在 **mjlab 工作副本**中显式把本项目加入 uv 依赖（那会改变当地 `pyproject.toml`/`uv.lock`，本指南不默认这样做）。

## 3. 逐级测试（按顺序执行）

以下命令均在 `~/mjlab` 目录、当前 mjlab 的 `.venv` 下运行。

### 3.1 静态 / 模型加载 / 插件注册

```bash
cd ~/mjlab
uv run --no-sync python -m unittest discover \
  -s ~/Micro-Wheeled_leg-Robot_sim_fork/tests -p 'test_*.py' -v
```

检查 XML 是否有效、4 个执行器与关节是否匹配、MJCF 能否编译、任务是否出现在 mjlab 注册表。若缺少依赖，相关动态检查会跳过；**通过静态测试不能证明仿真能运行**。

### 3.2 单环境无界面控制验证

```bash
cd ~/mjlab
uv run --no-sync micro-wl-sim --device cuda:0 --num-envs 1 --duration 8
```

打印模拟时间、机身俯仰/横滚角、运动距离、机身高度和 `FINISHED/TERMINATED`。返回码 `0` 表示达到设定时间且未触发终止条件；非零表示中途触发了跌倒或异常终止。它**不等同于控制鲁棒性证明**。

### 3.3 图形化观察

```bash
cd ~/mjlab
uv run --no-sync micro-wl-sim --device cuda:0 --viewer native

# 可选：较小速度与 30% 收腿
uv run --no-sync micro-wl-sim --device cuda:0 --viewer native \
  --speed 0.04 --crouch 0.3

# 可选：差速转向指令（rad/s）
uv run --no-sync micro-wl-sim --device cuda:0 --viewer native --turn 0.2
```

这里使用 **mjlab 自带的 NativeMujocoViewer**，而不是旧版 `mujoco.viewer` 单实例脚本。如果你的图形桌面后端出现问题，可先运行无界面版本定位问题。GUI 模式下倒地会自动重置环境。内置 `play --agent zero` 同样使用自动重置。

### 3.4 记录 CSV 以便调参

```bash
cd ~/mjlab
uv run --no-sync micro-wl-sim --device cuda:0 --duration 8 \
  --csv ~/Micro-Wheeled_leg-Robot_sim_fork/results/balance.csv
```

CSV 的字段包含 `t_s, pitch_deg, roll_deg, x_m, height_m, left_action, right_action`。其中 action 是标准化指令，轮部实际目标扭矩还需要乘 **0.035 N·m**。建议绘制机身俯仰角、两轮动作、机身高度随时间的变化，判断是否发散、饱和或出现极限振荡。

## 4. 任务接口与学习路线

| 文件 | 核心内容 | 自行扩展方向 |
| --- | --- | --- |
| `models/micro_wheeled_leg.xml` | 浮动底座、轮部接触、腿部铰链、执行器 | 替换为 CAD 实际尺寸，构造四杆闭链 |
| `task.py::robot_cfg` | `MjSpec`、`EntityCfg`、`XmlActuatorCfg` | 修改刚体惯量、驱动模式、碰撞参数 |
| `task.py::make_env_cfg` | `SceneCfg`、观测、动作、奖励、终止 | 设计地形、随机初始姿态、动作噪声 |
| `controller.py` | 四状态平衡反馈、差速和腿部横滚补偿 | 建立线性化模型，LQR 增益整定 |
| `run.py` | 单环境测试、GUI 及 CSV | 扰动测试、批量参数扫描、自动化指标 |
| `task.py::make_runner_cfg` | PPO 实验参数 | 模型独立调试后训练强化学习策略 |

建议按以下顺序学习与验证：

1. **验证动力学**：在无控制指令下观察重力、接触和关节方向，核对初始轮地接触、自由度与动作符号。若模型在初始状态下穿透地面或轮子方向相反，优先修正 MJCF，不要靠不断调增益掩盖问题。
2. **验证经典控制**：从 `controller.py` 的角度反馈开始，依次加入角速度、轮部位置和轮速，记录稳定时间、角度峰值和力矩饱和次数。已有增益仅是初值。
3. **提高模型保真度**：从原作者 STEP 测量连杆长度、质心、质量与转动惯量，再考虑使用 `<equality>` 实现机构闭链，并重新验证轮地接触。
4. **强化学习扩展**：确认能可靠 reset、稳定地无界面并行运行后，再优化观测、奖励、动作和 PPO 参数；目前奖励主要是**站立与抑制运动**，**不能直接当作速度跟踪任务**。

**参考资料（按 mjlab v1.6.1 源码接口）**：`~/mjlab/src/mjlab/tasks/cartpole/cartpole_env_cfg.py`（最小原生任务），`~/mjlab/src/mjlab/tasks/velocity/velocity_env_cfg.py`（速度任务），`~/mjlab/src/mjlab/envs/mdp/`（MDP 模块）以及 [mjlab 官方文档](https://mujocolab.github.io/mjlab/v1.6.0/)（1.6.1 的公开源码 API 应以本地 tag 为准）。

## 5. 可选：PPO 基础训练（并非已经验证的训练结果）

该分支已经注册一个 `mjlab` 原生 PPO 任务，但它使用的是**初始奖励配置与未经整定的模型**。先通过 3.1–3.4 再启动训练：

```bash
cd ~/mjlab
# 先小规模运行，避免调试阶段占满 GPU
uv run --no-sync train Mjlab-Micro-WheeledLeg-Balance \
  --env.scene.num-envs 64 \
  --agent.logger tensorboard \
  --agent.max-iterations 100

# 检查任务的零动作（无训练权重），此命令不运行经典控制器
uv run --no-sync play Mjlab-Micro-WheeledLeg-Balance \
  --agent zero --num-envs 1 --viewer native
```

**注意**：`play --agent zero` 与 `micro-wl-sim --viewer native` 是两条不同的路径：前者用于观察动作接口在零输入下的表现；后者用于测试经典控制基线。训练产生的模型要通过 `play --checkpoint-file <实际模型路径>` 加载，不要假定一定训练成功。

## 6. 常见问题

- **列表中找不到任务**：检查安装用的 Python 和运行用的 Python 是否同为 `~/mjlab/.venv/bin/python`；用 `uv run --no-sync python -c 'import micro_wl_mjlab; print(micro_wl_mjlab.TASK_ID)'` 查看详细报错。
- **`ModuleNotFoundError: mjlab` 或版本不符**：很可能安装到别的 Conda 或系统 Python 环境，重新用 `uv pip install --python ~/mjlab/.venv/bin/python -e . --no-deps`。
- **Native viewer 打不开**：先无界面测试，查看 CUDA/Warp 初始化与 DISPLAY/OpenGL 报错；不要同时混用单独安装的 `mujoco 4.0.0` Conda 环境。
- **机器人倒地**：本项目**尚未通过实测证明可稳定站立**，优先检查轮关节正方向、重心与接触、轮部扭矩上限，再对反馈增益进行系统整定。
- **与原实物外观不一致**：这是控制验证阶段的简化模型。结构细化需要原机械总装 STEP，属于后续独立工作，不是当前项目缺失的 PCB。

## 7. 版本和许可边界

本分支绑定 **mjlab==1.6.1** 的 API，依赖通过本机 `~/mjlab/.venv` 复用。未在当前执行环境内安装 mjlab 或启动 CUDA/Warp 物理仿真；实际运行结果以你本地验证为准。上游原仓库缺少覆盖全部资产的明确根级许可证，直接再分发原作者 CAD/PCB 内容时需另行核实授权；此分支只保留新编写的简化仿真模型和代码。原项目内容仍可从本仓库 `master` 分支和 Git 历史中查看。
