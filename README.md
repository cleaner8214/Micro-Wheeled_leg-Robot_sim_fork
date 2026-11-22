# Micro-Wheeled_leg-Robot — MuJoCo 仿真精简版

本分支是仿真专用的最小复现工程，不需要采购、加工、PCB、ESP32 或 Arduino。上游项目：[MuShibo/Micro-Wheeled_leg-Robot](https://github.com/MuShibo/Micro-Wheeled_leg-Robot)。原始 fork 内容保存在 `master` 分支。

## 范围

- `models/micro_wheeled_leg.xml`：浮动底盘、左右独立腿部铰链与两轮，四个执行器（两轮力矩、两腿位置）。
- `sim/control.py`：四状态自平衡反馈、差速转向、腿部对称收放及简单横滚补偿。
- `sim/run.py`：无界面仿真、窗口可视化、外力扰动、CSV 输出。
- `tests/`：基本结构/控制逻辑测试，及安装 MuJoCo 后的模型加载烟雾测试。

**重要限制**：这不是从原作者 STEP 文件生成的精确 MJCF。腿部采用单转动副代替原始连杆机构，几何、质量、惯量、摩擦与执行器力矩上限全部是初步工程假设。尚未实现原机的 FOC、真实舵机动力学、四杆闭链、跳跃控制、ROS2 或强化学习。不能由这里的结果推断原实体机器人的性能。

## 环境

Ubuntu 22.04、Python 3.12、MuJoCo 4.0.0。已有 `env_mujoco_v400_py312` 环境可直接使用：

    conda activate env_mujoco_v400_py312
    python -c 'import mujoco; print(mujoco.__version__)'

如未安装：

    python -m pip install -r requirements.txt

## 运行方法

在仓库根目录：

    # 直立仿真，默认不弹出窗口
    python -m sim.run --duration 8 --headless

    # GUI 显示 (GLFW/OpenGL)
    python -m sim.run --duration 15 --render

    # 前进及 40% 收腿
    python -m sim.run --duration 12 --speed 0.04 --crouch 0.4 --render

    # t=2.0～2.1 秒施加 0.3 N 的 +X 方向外力，保存 20Hz 状态记录
    python -m sim.run --duration 8 --push-force 0.3 --csv results/push.csv --headless

    # 关掉轮部平衡反馈，对照开环姿态
    python -m sim.run --duration 8 --no-balance --headless

    # 静态/控制单元测试；安装了 MuJoCo 才执行动力学加载烟雾测试
    python -m unittest discover -s tests -v

仿真状态在每次运行末尾打印。`FALLEN` 表示俯仰角超过 60° 或底盘高度低于 35 mm；返回码为 1。`FINISHED` 表示到时未触发失败条件，并不等同于理论稳定性证明。

## 模型参数与验证现状

模型轮半径 **0.015 m**，腿部连杆约 **0.071 m**，机身质量 **0.20 kg**，单轮质量 **0.025 kg**，单轮扭矩限幅 **0.035 N·m**。控制增益位于 `sim/control.py`，需要通过动力学实测整定。

当前只通过不依赖 MuJoCo 的 Python 控制测试与 XML 结构验证；由于工程创建环境没有 MuJoCo，**没有完成实际 MuJoCo 动力学稳定性测试**。请先在目标 Ubuntu 环境运行上述命令，确认 XML 能加载以及自平衡响应，再推进模型参数辨识。

建议验证顺序：静止站立 → 前后移动 → 腿部高度调节 → 外力扰动 → 转向。后续需要更接近原机结构时，从上游 `1.RobotModel/OriginalRobotModel.stp` 提取质心/惯量与连杆尺寸，增加真实闭链约束。

原作者根仓库未明确给所有资产设置统一许可证；在再分发原始机械资料时请确认授权范围。
