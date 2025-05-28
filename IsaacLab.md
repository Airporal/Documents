# IsaacLab教程

> IsaacLab是开源强化学习开发平台，基于IsaacSim仿真环境，结合开源强化学习库，方便进行Agent训练、部署

## 1  IsaacLab基本使用

​		首先需要安装IsaacSim、IsaacLab，安装步骤参考[官方文档](https://docs.robotsfan.com/isaaclab/source/setup/quickstart.html)。

### 		1.1. IsaacLab强化学习流程

​		USD资产文件、编写Ariiculation类和ArticulationCfg类来将USD导入到IsaacLab中、完成场景配置类，将所有资产组成一个场景。

---

​		之后设计机器人学习**任务**,基于马尔可夫决策过程,设计状态、动作、奖励、重置、结束等过程。可以使用基于管理器的配置方法和直接配置。

+ 基于管理器的配置方法

![manager-based-light](/media/airporal/8333B1863791CF8A/learn/Notebook/IsaacLab.assets/manager-based-light.svg)

​	RL任务需要完成 观测配置、动作配置、奖励配置、终止配置。

​	Manager_based的环境将任务分解为多个模块，由相应的管理器来配置，管理器继承至envs.ManagerBasedEnvcfg。不同的管理器由继承至envs.ManagerBasedEnv的环境类来处理。

​		示例代码框架：

```python
import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on creating a cartpole base environment.")
parser.add_argument("--num_envs", type=int, default=16, help="Number of environments to spawn.")

# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
"""Rest everything follows."""

import math
import torch

import isaaclab.envs.mdp as mdp
from isaaclab.envs import ManagerBasedEnv, ManagerBasedEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

from isaaclab_tasks.manager_based.classic.cartpole.cartpole_env_cfg import CartpoleSceneCfg

@configclass
class ActionsCfg:
    pass

@configclass
class ObservationsCfg:
    pass

@configclass
class EventCfg:
    pass

@configclass
class CartpoleEnvCfg(ManagerBasedEnvCfg):
    pass

def main():
    pass

if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
```

​		main中运行实例化CartpoleEnvCfg配置，这是一个继承至ManagerBasedEnvCfg的类，设置额外的命令行传入的参数，并根据配置实例化ManagerBasedEnv类，并执行循环，由于不需要梯度下降的自动求导，因此调用torch.inference_mode禁用了自动求导。

```python
def main():
    """Main function."""
    # parse the arguments
    env_cfg = CartpoleEnvCfg()
    env_cfg.scene.num_envs = args_cli.num_envs
    env_cfg.sim.device = args_cli.device
    # setup base environment
    env = ManagerBasedEnv(cfg=env_cfg)

    # simulate physics
    count = 0
    while simulation_app.is_running():
        with torch.inference_mode():
            # reset
            if count % 300 == 0:
                count = 0
                env.reset()
                print("-" * 80)
                print("[INFO]: Resetting environment...")
            # sample random actions
            joint_efforts = torch.randn_like(env.action_manager.action)
            # step the environment
            obs, _ = env.step(joint_efforts)
            # print current orientation of pole
            print("[Env 0]: Pole joint: ", obs["policy"][0][1].item())
            # update counter
            count += 1

    # close the environment
    env.close()
```

​		在ManagerBasedEnvCfg的子类配置中，首先需要实例化一个Scene，Scene是一个用于管理场景中各个组建的类。然后实例化ManagerBased框架中的各个Manager的配置类。

​		ManagerBasedEnvCfg中还可以对仿真的初始参数，如仿真步长、重力等。统一在一个__post_intit__函数中定义。

```python
@configclass
class CartpoleEnvCfg(ManagerBasedEnvCfg):
    """Configuration for the cartpole environment."""

    # Scene settings
    scene = CartpoleSceneCfg(num_envs=1024, env_spacing=2.5)
    # Basic settings
    observations = ObservationsCfg()
    actions = ActionsCfg()
    events = EventCfg()

    def __post_init__(self):
        """Post initialization."""
        # viewer settings
        self.viewer.eye = [4.5, 0.0, 6.0]
        self.viewer.lookat = [0.0, 0.0, 2.0]
        # step settings
        self.decimation = 4  # env step every 4 sim steps: 200Hz / 4 = 50Hz
        # simulation settings
        self.sim.dt = 0.005  # sim step every 5ms: 200Hz

```

​		管理Action的配置：通常直接调用Articulation.set_joint_effort_target可以对机器人的关节施加控制。ActionsCfg如下，使用mdp库中的各种ActionCfg建立需要的实例。

```python
@configclass
class ActionsCfg:
    """Action specifications for the environment."""

    joint_efforts = mdp.JointEffortActionCfg(asset_name="robot", joint_names=["slider_to_cart"], scale=5.0)
```

​		管理观测的配置：观测类定义智能体为了决策动作而需要观测的状态，观测类由多个观测组构成，每个观测组用于观测不同的观测空间，每个观测组下由多个观测的项以及一个用于设置参数的函数组成，观测组继承至ObservationGroupCfg，观测项由实例化的ObservationTermCfg定义，同时需要传入一个用于计算该观测的函数。

```python
@configclass
class ObservationsCfg:
    """Observation specifications for the environment."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()
```

​		管理事件的配置：管理事件主要包括对startup、reset、interval的事件处理，方便仿生发生这些事件时执行重置、随机化等。每个事件都实例化至EventTermCfg，执行的函数可以自行编写，也可以使用mdp中的函数，并传入参数：

```python
@configclass
class EventCfg:
    """Configuration for events."""

    # on startup
    add_pole_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=["pole"]),
            "mass_distribution_params": (0.1, 0.5),
            "operation": "add",
        },
    )

    # on reset
    reset_cart_position = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["slider_to_cart"]),
            "position_range": (-1.0, 1.0),
            "velocity_range": (-0.1, 0.1),
        },
    )

    reset_pole_position = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["cart_to_pole"]),
            "position_range": (-0.125 * math.pi, 0.125 * math.pi),
            "velocity_range": (-0.01 * math.pi, 0.01 * math.pi),
        },
    )
```



+ 直接配置

![direct-based-light](/media/airporal/8333B1863791CF8A/learn/Notebook/IsaacLab.assets/direct-based-light.svg)

​		直接配置实现一个定义观测、动作、奖励的类。这个类继承至envs.DirectRLEnv或者envs.DirectMARLEnv，并且定义envs.DirectRLEnvCfg或envs.DirectMARLEnvCfg的配置参数，定义一个直接配置的一部分内容如下：

```python
def _get_rewards(self) -> torch.Tensor:
    total_reward = compute_rewards(
        self.cfg.rew_scale_alive,
        self.cfg.rew_scale_terminated,
        self.cfg.rew_scale_pole_pos,
        self.cfg.rew_scale_cart_vel,
        self.cfg.rew_scale_pole_vel,
        self.joint_pos[:, self._pole_dof_idx[0]],
        self.joint_vel[:, self._pole_dof_idx[0]],
        self.joint_pos[:, self._cart_dof_idx[0]],
        self.joint_vel[:, self._cart_dof_idx[0]],
        self.reset_terminated,
    )
    return total_reward
@torch.jit.script
def compute_rewards(
    rew_scale_alive: float,
    rew_scale_terminated: float,
    rew_scale_pole_pos: float,
    rew_scale_cart_vel: float,
    rew_scale_pole_vel: float,
    pole_pos: torch.Tensor,
    pole_vel: torch.Tensor,
    cart_pos: torch.Tensor,
    cart_vel: torch.Tensor,
    reset_terminated: torch.Tensor,
):
    rew_alive = rew_scale_alive * (1.0 - reset_terminated.float())
    rew_termination = rew_scale_terminated * reset_terminated.float()
    rew_pole_pos = rew_scale_pole_pos * torch.sum(torch.square(pole_pos).unsqueeze(dim=1), dim=-1)
    rew_cart_vel = rew_scale_cart_vel * torch.sum(torch.abs(cart_vel).unsqueeze(dim=1), dim=-1)
    rew_pole_vel = rew_scale_pole_vel * torch.sum(torch.abs(pole_vel).unsqueeze(dim=1), dim=-1)
    total_reward = rew_alive + rew_termination + rew_pole_pos + rew_cart_vel + rew_pole_vel
    return total_reward
```

​		使用直接式定义，可以将所有组件定义在一个类中，方便实现复杂逻辑，同时可以使用改进的其它包，如Pytorch JIT或Warp等实现。

---

​		设计完RL任务后，还需要设计Agent的模型、神经网络的策略、价值函数；定义超参数和策略、价值模型的架构。

---

​		完成了任务设计和框架后，需要将环境注册到Gymnasium注册表中，以便使用唯一的环境名称创建环境，这使得环境可以在不同的RL算法和实验中可访问和重用。

​		在运行RL任务时，环境封装gymnasium.Wrapper类提供不改变环境的情况下，修改环境的行为的接口，需要使用合适的封装器或自定义的封装器，以获得需要的数据类型。

---

​		最后运行训练，IsaacLab直接使用开源RL库：StableBaselines3、RSL-RL、RL-GAMES、SKRL。运行相应的训练脚本即可开始训练。

+ 单GPU训练

![single-gpu-training-light](/media/airporal/8333B1863791CF8A/learn/Notebook/IsaacLab.assets/single-gpu-training-light.svg)

​		IsaacSim中执行Actions，并进行渲染和物理仿真，最后返回用户在观测配置类中定义的状态。		

​		IsaacLab中添加测量噪声，并将观测的状态以tensor的形式缓存，计算出奖励，并将状态传入RL库中进行训练。

​		RL库将观测的状态传递给策略，策略通过RL算法（PPO、TRPO等）进行训练，以输出适合机器人执行的正确动作。

+ 多GPU训练

![multi-gpu-training-light](/media/airporal/8333B1863791CF8A/learn/Notebook/IsaacLab.assets/multi-gpu-training-light.svg)

​		使用多GPU训练时，进程数（--nproc_per_node参数）不能多于可用GPU数量。梯度在各个GPU上分别计算并汇总，然后进行反向迭代。

​		还可以在多个计算机的GPU上进行[多节点训练](https://docs.robotsfan.com/isaaclab/source/features/multi_gpu.html)。	

---

​		完成训练后，将训练好的模型以pt、onnx等格式导出。在机器人上需要完成控制接口、状态估计接口、和模型推理接口，使用训练好的参数完成任务。

### 1.2 workflow

> 使用vscode 作为开发IDE

+ 生成settings.json和launch.json配置文件

​		在vscode 中运行生成脚本，首先按住ctrl+shift+p，然后选择run task，并运行setup_python_env即可自动生成配置文件。

​		注意，可将使用的python解释器更改为自己指定的环境。

+ 拓展开发

​		所有在config文件夹下包含extension.toml的文件都被识别为一个拓展，目录结构和python包相似：

```
<extension-name>
├── config
│   └── extension.toml
├── docs
│   ├── CHANGELOG.md
│   └── README.md
├── <extension-name>
│   ├── __init__.py
│   ├── ....
│   └── scripts
├── setup.py
└── tests
```

​		其中extension.toml文件包含拓展的元数据，包括名称、版本、描述关系等；docs目录包含拓展文档，extension-name目录包含拓展的主要python文件，scripts中包含加载到Omniverse中的应用程序，tests目录包含了拓展的测试。

![image-20250508161052839](/media/airporal/8333B1863791CF8A/learn/Notebook/IsaacLab.assets/image-20250508161052839.png)

​		拓展可能对其它python包存在依赖，需要在setup.py文件中指定，对于ROS、APT之类的依赖项需要在extension.toml文件中的isaac_lab_settings中指定：

```bash
[isaac_lab_settings]
# apt dependencies
apt_deps = ["libboost-all-dev"]

# ROS workspace
# note: if this path is relative, it is relative to the extension directory's root
ros_ws = "/home/user/catkin_ws"
```

​		依赖项通过tools/install_deps.py文件自动安装。

+ 独立应用程序

​		以脚本的形式控制Isaacsim仿真以及完成任务称为一个独立的应用程序，独立应用程序使用AppLauncher控制启动仿真器，使用SimulationContext直接控制仿真。

```python
"""Launch Isaac Sim Simulator first."""

from isaaclab.app import AppLauncher

# launch omniverse app
app_launcher = AppLauncher(headless=False)
simulation_app = app_launcher.app


"""Rest everything follows."""

from isaaclab.sim import SimulationContext

if __name__ == "__main__":
   # get simulation context
   simulation_context = SimulationContext()
   # reset and play simulation
   simulation_context.reset()
   # step simulation
   simulation_context.step()
   # stop simulation
   simulation_context.stop()

   # close the simulation
   simulation_app.close()
```

+ 模板生成器

​		使用IsaacLab提供的模板生成器可以生成不依赖IsaacLab的外部项目，或依赖IsaacLab的内部项目。

```bash
./isaacLab.sh --new
```

​		该脚本指导完成项目生成。

![image-20250508163705269](/media/airporal/8333B1863791CF8A/learn/Notebook/IsaacLab.assets/image-20250508163705269.png)

​		生成外部项目后，需要安装项目包（项目被视为一个新的Python库）:

```bash
python -m pip install -e source/<given-project-name>
```

​		可以查看或运行项目中的任务：

```bash
python scripts/list_envs.py
# 运行
python scripts/<specific-rl-library>/train.py --task=<Task-Name>
```

​		查看项目生成的Readme文档可了解其它操作说明。

---

​		对于内部任务，直接在isaacLab中生成了一个task.

![image-20250508164709883](/media/airporal/8333B1863791CF8A/learn/Notebook/IsaacLab.assets/image-20250508164709883.png)

​		生成位置在：

```bash
/home/airporal/isaacsim/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct
```

+ 超参数设置方法

> 超参数的设置方法和不同强化学习库的定义方法一置。

例如：

```python
python scripts/reinforcement_learning/rsl_rl/train.py --task=Isaac-Cartpole-v0 --headless env.actions.joint_effort.scale=10.0 agent.seed=2024
```

​		查看[此链接](https://docs.robotsfan.com/isaaclab/source/features/hydra.html)。

​		修改某些参数时，会导致一些其它参数不在满足映射关系，需要同时修改那些参数。

### 1.3 Omniverse

​		Omniverse 3D内容工作流程提供三个主要组件：

+ USD Composer：Omniverse中应用的文件格式
+ PhysX SDK：Omniverse的物理引擎
+ RTX渲染器：NVIDIA RTX GPU的光线跟踪技术实现的基于物理的渲染

## 2 IsaacLab 核心零件

+ Camera

​		查看此链接以获取Camera API使用方法。

+ [IMU](https://docs.robotsfan.com/isaaclab/source/overview/core-concepts/sensors/imu.html)
+ [接触力传感器](https://docs.robotsfan.com/isaaclab/source/overview/core-concepts/sensors/contact_sensor.html)

+ [TF](https://docs.robotsfan.com/isaaclab/source/overview/core-concepts/sensors/frame_transformer.html)

+ [射线传感器](https://docs.robotsfan.com/isaaclab/source/overview/core-concepts/sensors/ray_caster.html)

+ [关节控制器](https://docs.robotsfan.com/isaaclab/source/overview/core-concepts/motion_generators.html)

​		关节控制器通过配置ActuatorControlCfg可实现以下控制模式：

- [x] 扭矩控制 t_abs
- [x] 速度控制 v_abs或v_rel
- [x] 位置控制 p_abs 或 p_rel

---

+ 强化学习脚本

![image-20250508215102814](/media/airporal/8333B1863791CF8A/learn/Notebook/IsaacLab.assets/image-20250508215102814.png)

RL—GAMES：

```bash
# install python module (for rl-games)
./isaaclab.sh -i rl_games
# run script for training
./isaaclab.sh -p scripts/reinforcement_learning/rl_games/train.py --task Isaac-Ant-v0 --headless
# run script for playing with 32 environments
./isaaclab.sh -p scripts/reinforcement_learning/rl_games/play.py --task Isaac-Ant-v0 --num_envs 32 --checkpoint /PATH/TO/model.pth
# run script for playing a pre-trained checkpoint with 32 environments
./isaaclab.sh -p scripts/reinforcement_learning/rl_games/play.py --task Isaac-Ant-v0 --num_envs 32 --use_pretrained_checkpoint
# run script for recording video of a trained agent (requires installing `ffmpeg`)
./isaaclab.sh -p scripts/reinforcement_learning/rl_games/play.py --task Isaac-Ant-v0 --headless --video --video_length 200
```

训练输出：

```python
fps step: 112918 fps step and policy inference: 104337 fps total: 78179 epoch: 1/150 frames: 0
=> saving checkpoint 'IsaacLab/logs/rl_games/cartpole_direct/2024-12-28_20-23-06/nn/last_cartpole_direct_ep_150_rew_294.18793.pth'
saving next best rewards:  [294.18793]
```

RSL-RL

```PYTHON
# install python module (for rsl-rl)
./isaaclab.sh -i rsl_rl
# run script for training
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Reach-Franka-v0 --headless
# run script for playing with 32 environments
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py --task Isaac-Reach-Franka-v0 --num_envs 32 --load_run run_folder_name --checkpoint model.pt
# run script for playing a pre-trained checkpoint with 32 environments
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py --task Isaac-Reach-Franka-v0 --num_envs 32 --use_pretrained_checkpoint
# run script for recording video of a trained agent (requires installing `ffmpeg`)
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py --task Isaac-Reach-Franka-v0 --headless --video --video_length 200
```

```python
                        Learning iteration 0/150

                     Computation: 50355 steps/s (collection: 1.106s, learning 0.195s)
             Value function loss: 22.0539
                  Surrogate loss: -0.0086
           Mean action noise std: 1.00
                     Mean reward: -5.49
             Mean episode length: 15.79
--------------------------------------------------------------------------------
                 Total timesteps: 65536
                  Iteration time: 1.30s
                      Total time: 1.30s
                             ETA: 195.2s
```

SKRL

```PYTHON
# install python module (for skrl)
./isaaclab.sh -i skrl
# run script for training
./isaaclab.sh -p scripts/reinforcement_learning/skrl/train.py --task Isaac-Reach-Franka-v0 --headless
# run script for playing with 32 environments
./isaaclab.sh -p scripts/reinforcement_learning/skrl/play.py --task Isaac-Reach-Franka-v0 --num_envs 32 --checkpoint /PATH/TO/model.pt
# run script for playing a pre-trained checkpoint with 32 environments
./isaaclab.sh -p scripts/reinforcement_learning/skrl/play.py --task Isaac-Reach-Franka-v0 --num_envs 32 --use_pretrained_checkpoint
# run script for recording video of a trained agent (requires installing `ffmpeg`)
./isaaclab.sh -p scripts/reinforcement_learning/skrl/play.py --task Isaac-Reach-Franka-v0 --headless --video --video_length 200

# run script for training with the MAPPO algorithm (IPPO is also supported)
./isaaclab.sh -p scripts/reinforcement_learning/skrl/train.py --task Isaac-Shadow-Hand-Over-Direct-v0 --headless --algorithm MAPPO
# run script for playing with 32 environments with the MAPPO algorithm (IPPO is also supported)
./isaaclab.sh -p scripts/reinforcement_learning/skrl/play.py --task Isaac-Shadow-Hand-Over-Direct-v0 --num_envs 32 --algorithm MAPPO --checkpoint /PATH/TO/model.pt
```

```python
0%|                                          | 2/4800 [00:00<10:02,  7.96it/s]
```

Stable-Baselines3

```python
# install python module (for stable-baselines3)
./isaaclab.sh -i sb3
# run script for training
# note: we set the device to cpu since SB3 doesn't optimize for GPU anyway
./isaaclab.sh -p scripts/reinforcement_learning/sb3/train.py --task Isaac-Cartpole-v0 --headless --device cpu
# run script for playing with 32 environments
./isaaclab.sh -p scripts/reinforcement_learning/sb3/play.py --task Isaac-Cartpole-v0 --num_envs 32 --checkpoint /PATH/TO/model.zip
# run script for playing a pre-trained checkpoint with 32 environments
./isaaclab.sh -p scripts/reinforcement_learning/sb3/play.py --task Isaac-Cartpole-v0 --num_envs 32 --use_pretrained_checkpoint
# run script for recording video of a trained agent (requires installing `ffmpeg`)
./isaaclab.sh -p scripts/reinforcement_learning/sb3/play.py --task Isaac-Cartpole-v0 --headless --video --video_length 200
```

Stable-Baselines3不支持GPU！！！

```
------------------------------------------
| rollout/                |              |
|    ep_len_mean          | 30.8         |
|    ep_rew_mean          | 2.87         |
| time/                   |              |
|    fps                  | 8824         |
|    iterations           | 2            |
|    time_elapsed         | 14           |
|    total_timesteps      | 131072       |
| train/                  |              |
|    approx_kl            | 0.0079056695 |
|    clip_fraction        | 0.0842       |
|    clip_range           | 0.2          |
|    entropy_loss         | -1.42        |
|    explained_variance   | 0.0344       |
|    learning_rate        | 0.0003       |
|    loss                 | 10.4         |
|    n_updates            | 20           |
|    policy_gradient_loss | -0.0119      |
|    std                  | 1            |
|    value_loss           | 17           |
------------------------------------------
```

训练过程中的记录保存在logs目录中的Tensorboard中，查看日志：

```python
# execute from the root directory of the repository
./isaaclab.sh -p -m tensorboard.main --logdir=logs
```

## 3 Tutorials

### 3.1 模拟器使用

#### 3.1.1 启动仿真

​		使用**argparse**标准库获取参数命令行传递的参数，之后使用默认参数填充整个参数数据结构，最后加载解析参数并加载到App中，只有启动模拟应用程序才可以使用其它的依赖项。

```python
import argparse

from isaaclab.app import AppLauncher

# create argparser
parser = argparse.ArgumentParser(description="Tutorial on creating an empty stage.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()
# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app
```

​		启动仿真环境的参数可以通过命令行在运行脚本时指定，也可以用环境变量指定，也可以在代码中使用argparse设置。

```python
parser.add_argument(
    "--height", type=int, default=720, help="Height of the viewport and generated images. Defaults to 720"
)
```

​		这些参数也可以在代码的其它部分被使用：

```python
args_cli.height == 720
```

​		打开仿真器后，导入需要的python模块。

```python
# isaaclab.sim 中保护所有模拟器操作相关的子包
# SimulationCfg配置仿真的物理参数
# SimulationContext 控制模拟器步进、播放、暂停、处理回调函数等。
from isaaclab.sim import SimulationCfg, SimulationContext
```

```python
def main():
    """Main function."""

    # Initialize the simulation context
    sim_cfg = SimulationCfg(dt=0.01)
    sim = SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.0])

    # 初始化物理场景，并播放
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")

    # Simulate physics
    while simulation_app.is_running():
        # perform step
        sim.step()


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()
```

---

#### 3.1.2 配置[渲染模式](https://docs.robotsfan.com/isaaclab/source/how-to/configure_rendering.html)

```python
# rendering modes include performance, balanced, and quality
rendering_mode = "performance"

# carb setting dictionary can include any rtx carb setting which will overwrite the native preset setting
carb_settings = {"rtx.reflections.enabled": True}

# Initialize render config
render_cfg = sim_utils.RenderCfg(
rendering_mode=rendering_mode,
carb_settings=carb_settings,
)

# Initialize the simulation context with render coofig
sim_cfg = sim_utils.SimulationCfg(render=render_cfg)
sim = sim_utils.SimulationContext(sim_cfg)
```

​		配置RenderCfg可以牺牲一些渲染质量来提高仿真速度。

#### 3.1.3 添加日志信息

```python
def main():
    """Main function."""
    # Specify that the logs must be in logs/docker_tutorial
    log_dir_path = os.path.join("logs")
    if not os.path.isdir(log_dir_path):
        os.mkdir(log_dir_path)
    # In the container, the absolute path will be
    # /workspace/isaaclab/logs/docker_tutorial, because
    # all python execution is done through /workspace/isaaclab/isaaclab.sh
    # and the calling process' path will be /workspace/isaaclab
    log_dir_path = os.path.abspath(os.path.join(log_dir_path, "docker_tutorial"))
    if not os.path.isdir(log_dir_path):
        os.mkdir(log_dir_path)
    print(f"[INFO] Logging experiment to directory: {log_dir_path}")

    # Initialize the simulation context
    sim_cfg = SimulationCfg(dt=0.01)
    sim = SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.0])

    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")

    # Prepare to count sim_time
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0

    # Open logging file
    with open(os.path.join(log_dir_path, "log.txt"), "w") as log_file:
        # Simulate physics
        while simulation_app.is_running():
            log_file.write(f"{sim_time}" + "\n")
            # perform step
            sim.step()
            sim_time += sim_dt

```

#### 3.1.4 录制仿真动画

​		IsaacSim中使用Stage Recorder拓展来监听USD Stage中的所有运动和USD属性变化，并保存在USD文件中，该文件包含更改的时间样本，可以回放渲染动画。

​		IsaacLab中，使用BaseEnvWindow类来记录物理模拟的动画。需要禁用Fabric以允许读取和写入。例如：

```python
./isaaclab.sh -p scripts/environments/state_machine/lift_cube_sm.py --num_envs 8 --device cpu --disable_fabric
```

​		录制完的文件Stage、TimeSample_tk001保存在以下路径:![image-20250514215314386](/media/airporal/8333B1863791CF8A/learn/Notebook/IsaacLab.assets/image-20250514215314386.png)

​		要播放录制的文件，需要使用IsaacSim打开一个空世界，然后将两个USD文件都作为子层添加，在Content浏览器中找到两个文件，右键单击选择Insert As Sublayer，之后播放即可。

![image-20250514220225198](/media/airporal/8333B1863791CF8A/learn/Notebook/IsaacLab.assets/image-20250514220225198.png)

​		要在训练过程中录制剪辑视频：

```python
python scripts/reinforcement_learning/rl_games/train.py --task=Isaac-Cartpole-v0 --headless --video --video_length 100 --video_interval 500
```

​		直接在运行时设置即可，无需额外的设置。

### 3.2 仿真交互1 

> 仿真中设计普通刚体、机器人、软体的交互，分别采用不同的API完成。

#### 3.2.1 刚体的交互

```python
# sim_utils中包含了导入、创建USD、mtl、URDF等文件到仿真器中的工具
# prim_utils中包含了和prim交互的所有操作
import isaacsim.core.utils.prims as prim_utils
import isaaclab.sim as sim_utils
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

def design_scene():
    """Designs the scene by spawning ground plane, light, objects and meshes from usd files."""
    # Ground-plane
    cfg_ground = sim_utils.GroundPlaneCfg()
    cfg_ground.func("/World/defaultGroundPlane", cfg_ground)

    # spawn distant light
    cfg_light_distant = sim_utils.DistantLightCfg(
        intensity=3000.0,
        color=(0.75, 0.75, 0.75),
    )
    cfg_light_distant.func("/World/lightDistant", cfg_light_distant, translation=(1, 0, 10))

    # create a new xform prim for all objects to be spawned under
    prim_utils.create_prim("/World/Objects", "Xform")
    # spawn a red cone
    cfg_cone = sim_utils.ConeCfg(
        radius=0.15,
        height=0.5,
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
    )
    cfg_cone.func("/World/Objects/Cone1", cfg_cone, translation=(-1.0, 1.0, 1.0))
    cfg_cone.func("/World/Objects/Cone2", cfg_cone, translation=(-1.0, -1.0, 1.0))

    # spawn a green cone with colliders and rigid body
    cfg_cone_rigid = sim_utils.ConeCfg(
        radius=0.15,
        height=0.5,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(),
        mass_props=sim_utils.MassPropertiesCfg(mass=1.0),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)),
    )
    cfg_cone_rigid.func(
        "/World/Objects/ConeRigid", cfg_cone_rigid, translation=(-0.2, 0.0, 2.0), orientation=(0.5, 0.0, 0.5, 0.0)
    )

    # spawn a blue cuboid with deformable body
    cfg_cuboid_deformable = sim_utils.MeshCuboidCfg(
        size=(0.2, 0.5, 0.2),
        deformable_props=sim_utils.DeformableBodyPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.0, 1.0)),
        physics_material=sim_utils.DeformableBodyMaterialCfg(),
    )
    cfg_cuboid_deformable.func("/World/Objects/CuboidDeformable", cfg_cuboid_deformable, translation=(0.15, 0.0, 2.0))

    # spawn a usd file of a table into the scene
    cfg = sim_utils.UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/SeattleLabTable/table_instanceable.usd")
    cfg.func("/World/Objects/Table", cfg, translation=(0.0, 0.0, 1.05))

```

#### 3.2.2 软体的交互

​		软体的状态由网格的**节点和速度定义，**节点和速度定义在simulation world frame中，储存在assets.DeformableObject.data中。用户为一些节点指定位置目标，其余节点通过FEM求解器进行模拟。

```python
# Deformable Object
cfg = DeformableObjectCfg(
prim_path="/World/Origin.*/Cube",
spawn=sim_utils.MeshCuboidCfg(
size=(0.2, 0.2, 0.2),
deformable_props=sim_utils.DeformableBodyPropertiesCfg(rest_offset=0.0, contact_offset=0.001),
visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.5, 0.1, 0.0)),
physics_material=sim_utils.DeformableBodyMaterialCfg(poissons_ratio=0.4, youngs_modulus=1e5),
),
init_state=DeformableObjectCfg.InitialStateCfg(pos=(0.0, 0.0, 1.0)),
debug_vis=True,
)
```

​		当使用通配符*来创建多个对象实例时，可以通过这个对象entities的num_instances来访问创建的实例数量。注意，软体的状态由软体的节点Nodal定义：

```python
def run_simulator(sim: sim_utils.SimulationContext, entities: dict[str, DeformableObject], origins: torch.Tensor):
    """Runs the simulation loop."""
    # Extract scene entities
    # note: we only do this here for readability. In general, it is better to access the entities directly from
    #   the dictionary. This dictionary is replaced by the InteractiveScene class in the next tutorial.
    cube_object = entities["cube_object"]
    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0

    # Nodal kinematic targets of the deformable bodies
    nodal_kinematic_target = cube_object.data.nodal_kinematic_target.clone()

    # Simulate physics
    while simulation_app.is_running():
        # reset
        if count % 250 == 0:
            # reset counters
            sim_time = 0.0
            count = 0

            # reset the nodal state of the object
            nodal_state = cube_object.data.default_nodal_state_w.clone()
            # 生成位置
            pos_w = torch.rand(cube_object.num_instances, 3, device=sim.device) * 0.1 + origins
            # 生成四元数
            quat_w = math_utils.random_orientation(cube_object.num_instances, device=sim.device)
            nodal_state[..., :3] = cube_object.transform_nodal_pos(nodal_state[..., :3], pos_w, quat_w)

            # write nodal state to simulation
            cube_object.write_nodal_state_to_sim(nodal_state)

            # Write the nodal state to the kinematic target and free all vertices
            nodal_kinematic_target[..., :3] = nodal_state[..., :3]
            nodal_kinematic_target[..., 3] = 1.0
            cube_object.write_nodal_kinematic_target_to_sim(nodal_kinematic_target)

            # reset buffers
            cube_object.reset()

            print("----------------------------------------")
            print("[INFO]: Resetting object state...")

        # update the kinematic target for cubes at index 0 and 3
        # we slightly move the cube in the z-direction by picking the vertex at index 0
        nodal_kinematic_target[[0, 3], 0, 2] += 0.001
        # set vertex at index 0 to be kinematically constrained
        # 0: constrained, 1: free
        nodal_kinematic_target[[0, 3], 0, 3] = 0.0
        # write kinematic target to simulation
        cube_object.write_nodal_kinematic_target_to_sim(nodal_kinematic_target)

        # write internal data to simulation
        cube_object.write_data_to_sim()
        # perform step
        sim.step()
        #体支持用户驱动的运动学控制，其中用户可 update sim-time
        sim_time += sim_dt
        count += 1
        # update buffers
        cube_object.update(sim_dt)
        # print the root position
        if count % 50 == 0:
            print(f"Root position (in world): {cube_object.data.root_pos_w[:, :3]}")

```

​		每个entities有多个Instances，则Nodel_state包含n个instance的m个节点的4个值，四个值分别是三维的Nodel目标位置和用来表示固定还是释放的bool数据。

​		当entities变得多的时候，配置变得复杂，此时使用from isaaclab.scene import InteractiveScene, InteractiveSceneCfg来管理配置。

#### 3.2.3 机器人的交互

​		机器人有不同的关节，首先需要使用ArticulationCfg来描述机器人，至少需要设置机器人的路径、驱动关节的配置。

```python
import numpy as np
import torch

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import AssetBaseCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

JETBOT_CONFIG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/Jetbot/jetbot.usd"),
    actuators={"wheel_acts": ImplicitActuatorCfg(joint_names_expr=[".*"], damping=None, stiffness=None)},
)
```

​		设置ImplicitActuatorCfg时，需要指定关节名，使用正则表达式.*代表所有关节，同时将刚度和阻尼设置为None表示使用默认值。更加复杂的配置：

```python
DOFBOT_CONFIG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAAC_NUCLEUS_DIR}/Robots/Dofbot/dofbot.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "joint1": 0.0,
            "joint2": 0.0,
            "joint3": 0.0,
            "joint4": 0.0,
        },
        pos=(0.25, -0.25, 0.0),
    ),
    actuators={
        "front_joints": ImplicitActuatorCfg(
            joint_names_expr=["joint[1-2]"],
            effort_limit_sim=100.0,
            velocity_limit_sim=100.0,
            stiffness=10000.0,
            damping=100.0,
        ),
        "joint3_act": ImplicitActuatorCfg(
            joint_names_expr=["joint3"],
            effort_limit_sim=100.0,
            velocity_limit_sim=100.0,
            stiffness=10000.0,
            damping=100.0,
        ),
        "joint4_act": ImplicitActuatorCfg(
            joint_names_expr=["joint4"],
            effort_limit_sim=100.0,
            velocity_limit_sim=100.0,
            stiffness=10000.0,
            damping=100.0,
        ),
    },
)
```

​				

​		使用继承至InteractiveSceneCfg的类来创建一个新的场景：

```python
class NewRobotsSceneCfg(InteractiveSceneCfg):
    """Designs the scene."""

    # Ground-plane
    ground = AssetBaseCfg(prim_path="/World/defaultGroundPlane", spawn=sim_utils.GroundPlaneCfg())

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # robot
    Jetbot = JETBOT_CONFIG.replace(prim_path="{ENV_REGEX_NS}/Jetbot")
    Dofbot = DOFBOT_CONFIG.replace(prim_path="{ENV_REGEX_NS}/Dofbot")
```

​		注意，不可交互的地面与光源使用AssetBaseCfg来指定,prim使用绝对路径表示，可以交互的robot使用ArticulationCfg来指定，并用变量ENV_REGEX_NS的相对路径表示。特殊变量ENV_REGEX_NS会在环境创建时被环境名称替代，任何带有ENV_REGEX_NS的路径都会被替换（/World/envs/env_i）。例如：

![image-20250509221330877](/media/airporal/8333B1863791CF8A/learn/Notebook/IsaacLab.assets/image-20250509221330877.png)

​		通过SimulationContext和InteractiveScene来获取建立的场景和机器人，控制仿真逻辑。

```python
def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    sim_dt = sim.get_physics_dt()
    sim_time = 0.0
    count = 0

    while simulation_app.is_running():
        # reset
        if count % 500 == 0:
            # reset counters
            count = 0
            # reset the scene entities to their initial positions offset by the environment origins
            root_jetbot_state = scene["Jetbot"].data.default_root_state.clone()
            root_jetbot_state[:, :3] += scene.env_origins
            root_dofbot_state = scene["Dofbot"].data.default_root_state.clone()
            root_dofbot_state[:, :3] += scene.env_origins

            # copy the default root state to the sim for the jetbot's orientation and velocity
            scene["Jetbot"].write_root_pose_to_sim(root_jetbot_state[:, :7])
            scene["Jetbot"].write_root_velocity_to_sim(root_jetbot_state[:, 7:])
            scene["Dofbot"].write_root_pose_to_sim(root_dofbot_state[:, :7])
            scene["Dofbot"].write_root_velocity_to_sim(root_dofbot_state[:, 7:])

            # copy the default joint states to the sim
            joint_pos, joint_vel = (
                scene["Jetbot"].data.default_joint_pos.clone(),
                scene["Jetbot"].data.default_joint_vel.clone(),
            )
            scene["Jetbot"].write_joint_state_to_sim(joint_pos, joint_vel)
            joint_pos, joint_vel = (
                scene["Dofbot"].data.default_joint_pos.clone(),
                scene["Dofbot"].data.default_joint_vel.clone(),
            )
            scene["Dofbot"].write_joint_state_to_sim(joint_pos, joint_vel)
            # clear internal buffers
            scene.reset()
            print("[INFO]: Resetting Jetbot and Dofbot state...")

        # drive around
        if count % 100 < 75:
            # Drive straight by setting equal wheel velocities
            action = torch.Tensor([[10.0, 10.0]])
        else:
            # Turn by applying different velocities
            action = torch.Tensor([[5.0, -5.0]])

        scene["Jetbot"].set_joint_velocity_target(action)

        # wave
        wave_action = scene["Dofbot"].data.default_joint_pos
        wave_action[:, 0:4] = 0.25 * np.sin(2 * np.pi * 0.5 * sim_time)
        scene["Dofbot"].set_joint_position_target(wave_action)

        scene.write_data_to_sim()
        sim.step()
        sim_time += sim_dt
        count += 1
        scene.update(sim_dt)

```

> 相比自己实现design_scene，使用InteractiveScene和InteractiveSceneCfg结合使得代码更加高效。

#### 3.2.4 添加传感器

​		机器人的传感器一般以低于模拟频率的频率更新，可以获得不同的本体感知和外部感知信息。

​		所有的传感器都继承至sensors.SensorBase类，每个传感器都根据各自的配置类进行配置，其中更新频率通过sensors.SensorBaseCfg.update_period属性以秒为单位指定。

​		传感器附加到场景中的prims中，它们可以于场景中已有的prim相关联，或者附加到现有的prim上，例如相机传感器有一个对应的prim在场景中创建，而接触传感器是刚体prim的一个属性。

​		camera的配置主要包括相机类型、更新周期、分辨率、数据类型（可以是rgb\distance_to_image_plane\normals）、绑定的prim类型、相机相对于机器人基准坐标系的偏移量。

```python
# sensors
camera = CameraCfg(
    prim_path="{ENV_REGEX_NS}/Robot/base/front_cam",
    update_period=0.1,
    height=480,
    width=640,
    data_types=["rgb", "distance_to_image_plane"],
    spawn=sim_utils.PinholeCameraCfg(
    focal_length=24.0, focus_distance=400.0, horizontal_aperture=20.955, clipping_range=(0.1, 1.0e5)
    ),
    offset=CameraCfg.OffsetCfg(pos=(0.510, 0.0, 0.015), rot=(0.5, -0.5, 0.5, -0.5), convention="ros"),
)
```

​		这里的ENV_REGEX_NS是环境命名空间，使用的相机是PinholeCamera。

​		可能会一次开启多个环境，而每个环境都有相机，导致相机数量过多影响仿真速度，IsaacLab提供了benchmark_camera.py脚本，用于自动确定环境中可以运行相机的最大数量，可以查看在某个任务下运行若干个相机时的性能：

```python
./isaaclab.sh -p scripts/benchmarks/benchmark_cameras.py \
--task Isaac-Cartpole-v0 --num_tiled_cameras 100 \
--task_num_cameras_per_env 2 \
--tiled_camera_data_types rgb
```

​		可以自动查找最大可运行相机数量：

```python
./isaaclab.sh -p scripts/benchmarks/benchmark_cameras.py \
--task Isaac-Cartpole-v0 --num_tiled_cameras 100 \
--task_num_cameras_per_env 2 \
--tiled_camera_data_types rgb --autotune \
--autotune_max_percentage_util 100 80 50 50
```

​		可以查看在没有任务下相机的性能，例如，使用两个相机查看100个随机物体

---



​		光线传感器是使用NVIDIA Warp光线投射内核的虚拟传感器，通过RayCasterCfg，可以指定要投射的光线模式和要投射的网格。由于是虚拟传感器，因此不需要在场景中为其创建对应的prim，而是将其附加到场景中的一个prim中，该prim用于指定传感器的位置。

​		光线传感器需要指定绑定的prim_path、更新频率、光线模式以及一些其他的参数。其中debug_vis可以可视化光线击中的网格，attach_yaw_only可以仅仅显示yaw的运动。

```python
height_scanner = RayCasterCfg(
    prim_path="{ENV_REGEX_NS}/Robot/base",
    update_period=0.02,
    offset=RayCasterCfg.OffsetCfg(pos=(0.0, 0.0, 20.0)),
    attach_yaw_only=True,
    pattern_cfg=patterns.GridPatternCfg(resolution=0.1, size=[1.6, 1.0]),
    debug_vis=True,
    mesh_prim_paths=["/World/defaultGroundPlane"],
)
```

​		接触传感器包装physX接触报告API以获得机器人与环境的接触信息，接触传感器依赖于PhysX，需要将activate_contact_sensors设置为true来启用刚体的接触传感器。

​		通过ContactSensorCfg可以指定要获取的接触力传感器的prims，这里使用正则表达式将接触力传感器附加到机器人的所有FOOT上，并设置更新频率为0表示于模拟相同的频率更新、指定只储存最后六个模拟步骤的接触信息，并可视化接触点。

```python
contact_forces = ContactSensorCfg(
    prim_path="{ENV_REGEX_NS}/Robot/.*_FOOT", update_period=0.0, history_length=6, debug_vis=True
)
```

​		TF信息通过FrameTransformer与配置类FrameTransformerCfg来设置，并需要指定prim_path表示父prim、要获取的target_frames列表，列表中的每个prim需要使用类FrameTransformerCfg.FrameCfg类配置。

```python
def define_sensor() -> FrameTransformer:
    """Defines the FrameTransformer sensor to add to the scene."""
    # define offset
    rot_offset = math_utils.quat_from_euler_xyz(torch.zeros(1), torch.zeros(1), torch.tensor(-math.pi / 2))
    pos_offset = math_utils.quat_apply(rot_offset, torch.tensor([0.08795, 0.01305, -0.33797]))

    # Example using .* to get full body + LF_FOOT
    frame_transformer_cfg = FrameTransformerCfg(
        prim_path="/World/Robot/base",
        target_frames=[
            FrameTransformerCfg.FrameCfg(prim_path="/World/Robot/.*"),
            FrameTransformerCfg.FrameCfg(
                prim_path="/World/Robot/LF_SHANK",
                name="LF_FOOT_USER",
                offset=OffsetCfg(pos=tuple(pos_offset.tolist()), rot=tuple(rot_offset[0].tolist())),
            ),
        ],
        debug_vis=False,
    )
    frame_transformer = FrameTransformer(frame_transformer_cfg)

    return frame_transformer
def design_scene() -> dict:
    """Design the scene."""
    # Populate scene
    # -- Ground-plane
    cfg = sim_utils.GroundPlaneCfg()
    cfg.func("/World/defaultGroundPlane", cfg)
    # -- Lights
    cfg = sim_utils.DistantLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    cfg.func("/World/Light", cfg)
    # -- Robot
    robot = Articulation(ANYMAL_C_CFG.replace(prim_path="/World/Robot"))
    # -- Sensors
    frame_transformer = define_sensor()

    # return the scene information
    scene_entities = {"robot": robot, "frame_transformer": frame_transformer}
    return scene_entities
```

#### 3.2.5 添加可视化标记物

> 使用isaacsim.utls.debug_draw或markers.VisualizationMarkers向仿真中添加一些可视化标志

​		首先配置标记：

```python
marker_cfg = VisualizationMarkersCfg(
    prim_path="/Visuals/myMarkers",
    markers={
        "frame": sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/frame_prim.usd",
            scale=(0.5, 0.5, 0.5),
        ),
        "arrow_x": sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/UIElements/arrow_x.usd",
            scale=(1.0, 0.5, 0.5),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 1.0)),
        ),
        "cube": sim_utils.CuboidCfg(
            size=(1.0, 1.0, 1.0),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
        ),
        "sphere": sim_utils.SphereCfg(
            radius=0.5,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 1.0, 0.0)),
        ),
        "cylinder": sim_utils.CylinderCfg(
            radius=0.5,
            height=1.0,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.0, 0.0, 1.0)),
        ),
        "cone": sim_utils.ConeCfg(
            radius=0.5,
            height=1.0,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 1.0, 0.0)),
        ),
        "mesh": sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
            scale=(10.0, 10.0, 10.0),
        ),
        "mesh_recolored": sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Blocks/DexCube/dex_cube_instanceable.usd",
            scale=(10.0, 10.0, 10.0),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.25, 0.0)),
        ),
        "robot_mesh": sim_utils.UsdFileCfg(
            usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/ANYbotics/ANYmal-C/anymal_c.usd",
            scale=(2.0, 2.0, 2.0),
            visual_material=sim_utils.GlassMdlCfg(glass_color=(0.0, 0.1, 0.0)),
        ),
    },
)
```

​		标记也可以是简单的集合形状，也可以是复杂的模型。设置完标记后，使用以下方法显示：

```python
while simulation_app.is_running():
    # rotate the markers around the z-axis for visualization
    marker_orientations = quat_from_angle_axis(yaw, torch.tensor([0.0, 0.0, 1.0]))
    # visualize
    my_visualizer.visualize(marker_locations, marker_orientations, marker_indices=marker_indices)
    # roll corresponding indices to show how marker prototype can be changed
    if yaw[0].item() % (0.5 * torch.pi) < 0.01:
```

​		

---

### 3.4 强化学习

> 所有继承自ManagerBasedRLEnv、DirectRLEnv都与gymnasium.Wrapper 兼容，因为底层实现了gymnasium.Env的接口。

#### 3.4.1 基于管理器设置强化学习

​		基于管理器设置强化学习需要实现MDP过程的管理器配置。整体是实现一个继承至类ManagerBasedRLEnvCfg的配置，修改配置参数而不是实现。关于基于管理器配置的实现可以单独放在一个python文件中，方便管理。

```python
@configclass
class CartpoleEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the cartpole environment."""

    # Scene settings
    scene: CartpoleSceneCfg = CartpoleSceneCfg(num_envs=4096, env_spacing=4.0)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    events: EventCfg = EventCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()

    # Post initialization
    def __post_init__(self) -> None:
        """Post initialization."""
        # general settings
        self.decimation = 2
        self.episode_length_s = 5
        # viewer settings
        self.viewer.eye = (8.0, 0.0, 5.0)
        # simulation settings
        self.sim.dt = 1 / 120
        self.sim.render_interval = self.decimation
```

​		场景的配置使用默认提供的例子，而ActionCfg()的实现如下，奖励函数包含五个任务：

​		鼓励存活（游戏未termerinated)

​		惩罚失败（游戏termerinated)

​		鼓励保持向上的期望位置（任务目标）

​		惩罚过大的速度（沿着轴运动）

​		惩罚过大的角速度（杆摆动）

```python
@configclass
class RewardsCfg:
    """Reward terms for the MDP."""

    # (1) Constant running reward
    alive = RewTerm(func=mdp.is_alive, weight=1.0)
    # (2) Failure penalty
    terminating = RewTerm(func=mdp.is_terminated, weight=-2.0)
    # (3) Primary task: keep pole upright
    pole_pos = RewTerm(
        func=mdp.joint_pos_target_l2,
        weight=-1.0,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["cart_to_pole"]), "target": 0.0},
    )
    # (4) Shaping tasks: lower cart velocity
    cart_vel = RewTerm(
        func=mdp.joint_vel_l1,
        weight=-0.01,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["slider_to_cart"])},
    )
    # (5) Shaping tasks: lower pole angular velocity
    pole_vel = RewTerm(
        func=mdp.joint_vel_l1,
        weight=-0.005,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["cart_to_pole"])},
    )

```

​		TerminationsCfg决定每个回合的终止条件，通常有两种情况，第一种是达到最大指定回合长度而没有完成任务，此时终止，另一种是完成任务，为了学到更多的方法而重新开始。

```python
class TerminationsCfg:
    """Termination terms for the MDP."""

    # (1) Time out
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    # (2) Cart out of bounds
    cart_out_of_bounds = DoneTerm(
        func=mdp.joint_pos_out_of_manual_limit,
        params={"asset_cfg": SceneEntityCfg("robot", joint_names=["slider_to_cart"]), "bounds": (-3.0, 3.0)},
    )
```

​		ActionsCfg、ObservationsCfg、EnentCfg参考1.1小节定义各自组件：

```python

@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    joint_effort = mdp.JointEffortActionCfg(asset_name="robot", joint_names=["slider_to_cart"], scale=100.0)


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Observations for policy group."""

        # observation terms (order preserved)
        joint_pos_rel = ObsTerm(func=mdp.joint_pos_rel)
        joint_vel_rel = ObsTerm(func=mdp.joint_vel_rel)

        def __post_init__(self) -> None:
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""

    # reset
    reset_cart_position = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["slider_to_cart"]),
            "position_range": (-1.0, 1.0),
            "velocity_range": (-0.5, 0.5),
        },
    )

    reset_pole_position = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=["cart_to_pole"]),
            "position_range": (-0.25 * math.pi, 0.25 * math.pi),
            "velocity_range": (-0.25 * math.pi, 0.25 * math.pi),
        },
    )
```

> 除上述管理器外，有时还需要定义命令管理器、课程管理器

**域随机化**：在包含EventTermCfg变量的配置类中可以使用域随机化，所有域随机化都是在EventTerm中使用一个进行域随机化的函数完成。

```python
@configclass
class EventCfg:
  robot_physics_material = EventTerm(
      func=mdp.randomize_rigid_body_material,
      mode="reset",
      params={
          "asset_cfg": SceneEntityCfg("robot", body_names=".*"),
          "static_friction_range": (0.7, 1.3),
          "dynamic_friction_range": (1.0, 1.0),
          "restitution_range": (1.0, 1.0),
          "num_buckets": 250,
      },
  )
  robot_joint_stiffness_and_damping = EventTerm(
      func=mdp.randomize_actuator_gains,
      mode="reset",
      params={
          "asset_cfg": SceneEntityCfg("robot", joint_names=".*"),
          "stiffness_distribution_params": (0.75, 1.5),
          "damping_distribution_params": (0.3, 3.0),
          "operation": "scale",
          "distribution": "log_uniform",
      },
  )
  reset_gravity = EventTerm(
      func=mdp.randomize_physics_scene_gravity,
      mode="interval",
      is_global_time=True,
      interval_range_s=(36.0, 36.0),  # time_s = num_steps * (decimation * dt)
      params={
          "gravity_distribution_params": ([0.0, 0.0, 0.0], [0.0, 0.0, 0.4]),
          "operation": "add",
          "distribution": "gaussian",
      },
  )
```

#### 3.4.2 动作和观测的噪声

​		在执行动作和进行观测时，分别存在各自的噪声和偏差，以下对此进行了配置。

```python
@configclass
class MyTaskConfig:

    # at every time-step add gaussian noise + bias. The bias is a gaussian sampled at reset
    action_noise_model: NoiseModelWithAdditiveBiasCfg = NoiseModelWithAdditiveBiasCfg(
      noise_cfg=GaussianNoiseCfg(mean=0.0, std=0.05, operation="add"),
      bias_noise_cfg=GaussianNoiseCfg(mean=0.0, std=0.015, operation="abs"),
    )

        
    # at every time-step add gaussian noise + bias. The bias is a gaussian sampled at reset
    observation_noise_model: NoiseModelWithAdditiveBiasCfg = NoiseModelWithAdditiveBiasCfg(
      noise_cfg=GaussianNoiseCfg(mean=0.0, std=0.002, operation="add"),
      bias_noise_cfg=GaussianNoiseCfg(mean=0.0, std=0.0001, operation="abs"),
    )
```

​		也可以仅仅加噪声：

```python
  action_noise_model: GaussianNoiseCfg = GaussianNoiseCfg(mean=0.0, std=0.05, operation="add")
```



#### 3.4.3 直接式设置强化学习

> 直接式强化学习允许更加灵活的控制强化学习交互过程，如使用PytorchJIT等

​		使用直接式强化学习需要实现继承至DirectRLEnvCfg和继承至DirectRLEnv的配置类和管理实例。在配置类中需要定义动作和状态数、定义特定任务的属性如重置条件的阈值、奖励的缩放项等：

```python
@configclass
class CartpoleEnvCfg(DirectRLEnvCfg):
    # env
    decimation = 2
    episode_length_s = 5.0
    action_scale = 100.0  # [N]
    action_space = 1
    observation_space = 4
    state_space = 0

    # simulation
    sim: SimulationCfg = SimulationCfg(dt=1 / 120, render_interval=decimation)

    # robot
    robot_cfg: ArticulationCfg = CARTPOLE_CFG.replace(prim_path="/World/envs/env_.*/Robot")
    cart_dof_name = "slider_to_cart"
    pole_dof_name = "cart_to_pole"

    # scene
    scene: InteractiveSceneCfg = InteractiveSceneCfg(num_envs=4096, env_spacing=4.0, replicate_physics=True)

    # reset
    max_cart_pos = 3.0  # the cart is reset if it exceeds that position [m]
    initial_pole_angle_range = [-0.25, 0.25]  # the range in which the pole angle is sampled from on reset [rad]

    # reward scales
    rew_scale_alive = 1.0
    rew_scale_terminated = -2.0
    rew_scale_pole_pos = -1.0
    rew_scale_cart_vel = -0.01
    rew_scale_pole_vel = -0.005

```

​		配置类基本设置了在管理类中的各个组件的参数，将配置类传入DiectRLEnv的子类中，并在此定义强化学习的各个组件：

```python
class CartpoleEnv(DirectRLEnv):
    cfg: CartpoleEnvCfg

    def __init__(self, cfg: CartpoleEnvCfg, render_mode: str | None = None, **kwargs):
        super().__init__(cfg, render_mode, **kwargs)

        self._cart_dof_idx, _ = self.cartpole.find_joints(self.cfg.cart_dof_name)
        self._pole_dof_idx, _ = self.cartpole.find_joints(self.cfg.pole_dof_name)
        self.action_scale = self.cfg.action_scale

        self.joint_pos = self.cartpole.data.joint_pos
        self.joint_vel = self.cartpole.data.joint_vel
```

​		在此子类中，要定义场景：

```python
def _setup_scene(self):
    self.cartpole = Articulation(self.cfg.robot_cfg)
    # add ground plane
    spawn_ground_plane(prim_path="/World/ground", cfg=GroundPlaneCfg())
    # clone and replicate
    self.scene.clone_environments(copy_from_source=False)
    # add articulation to scene
    self.scene.articulations["cartpole"] = self.cartpole
    # add lights
    light_cfg = sim_utils.DomeLightCfg(intensity=2000.0, color=(0.75, 0.75, 0.75))
    light_cfg.func("/World/Light", light_cfg)
```

​		定义奖励：

```python
def _get_rewards(self) -> torch.Tensor:
    total_reward = compute_rewards(
    self.cfg.rew_scale_alive,
    self.cfg.rew_scale_terminated,
    self.cfg.rew_scale_pole_pos,
    self.cfg.rew_scale_cart_vel,
    self.cfg.rew_scale_pole_vel,
    self.joint_pos[:, self._pole_dof_idx[0]],
    self.joint_vel[:, self._pole_dof_idx[0]],
    self.joint_pos[:, self._cart_dof_idx[0]],
    self.joint_vel[:, self._cart_dof_idx[0]],
    self.reset_terminated,
    )
return total_reward
```

​		奖励以一个外部函数compute_rewards的形式定义，传入这些参数后，计算出整体的奖励，这里compute_rewards使用pytorch jit进行加速：

```python
@torch.jit.script
def compute_rewards(
    rew_scale_alive: float,
    rew_scale_terminated: float,
    rew_scale_pole_pos: float,
    rew_scale_cart_vel: float,
    rew_scale_pole_vel: float,
    pole_pos: torch.Tensor,
    pole_vel: torch.Tensor,
    cart_pos: torch.Tensor,
    cart_vel: torch.Tensor,
    reset_terminated: torch.Tensor,
):
    rew_alive = rew_scale_alive * (1.0 - reset_terminated.float())
    rew_termination = rew_scale_terminated * reset_terminated.float()
    rew_pole_pos = rew_scale_pole_pos * torch.sum(torch.square(pole_pos).unsqueeze(dim=1), dim=-1)
    rew_cart_vel = rew_scale_cart_vel * torch.sum(torch.abs(cart_vel).unsqueeze(dim=1), dim=-1)
    rew_pole_vel = rew_scale_pole_vel * torch.sum(torch.abs(pole_vel).unsqueeze(dim=1), dim=-1)
    total_reward = rew_alive + rew_termination + rew_pole_pos + rew_cart_vel + rew_pole_vel
    return total_reward
```

​		@torch.jit.script装饰器会使用TorchScript将函数模板化，使得函数保存为 `.pt` 文件，然后在不依赖 Python 的环境中加载和运行，比如用 C++ 运行模型。

​		观测由一个字典组成:

```python
def _get_observations(self) -> dict:
    obs = torch.cat(
    (
    self.joint_pos[:, self._pole_dof_idx[0]].unsqueeze(dim=1),
    self.joint_vel[:, self._pole_dof_idx[0]].unsqueeze(dim=1),
    self.joint_pos[:, self._cart_dof_idx[0]].unsqueeze(dim=1),
    self.joint_vel[:, self._cart_dof_idx[0]].unsqueeze(dim=1),
    ),
    dim=-1,
    )
    observations = {"policy": obs}
return observations
```

​		设置终止条件，以判断是否终止：

```python
def _get_dones(self) -> tuple[torch.Tensor, torch.Tensor]:
    self.joint_pos = self.cartpole.data.joint_pos
    self.joint_vel = self.cartpole.data.joint_vel

    time_out = self.episode_length_buf >= self.max_episode_length - 1
    out_of_bounds = torch.any(torch.abs(self.joint_pos[:, self._cart_dof_idx]) > self.cfg.max_cart_pos, dim=1)
    out_of_bounds = out_of_bounds | torch.any(torch.abs(self.joint_pos[:, self._pole_dof_idx]) > math.pi / 2, dim=1)
return out_of_bounds, time_out
```

​		设置重置操作，以在终止时重置环境：

```python
def _reset_idx(self, env_ids: Sequence[int] | None):
    if env_ids is None:
    env_ids = self.cartpole._ALL_INDICES
    super()._reset_idx(env_ids)

    joint_pos = self.cartpole.data.default_joint_pos[env_ids]
    joint_pos[:, self._pole_dof_idx] += sample_uniform(
    self.cfg.initial_pole_angle_range[0] * math.pi,
    self.cfg.initial_pole_angle_range[1] * math.pi,
    joint_pos[:, self._pole_dof_idx].shape,
    joint_pos.device,
    )
    joint_vel = self.cartpole.data.default_joint_vel[env_ids]

    default_root_state = self.cartpole.data.default_root_state[env_ids]
    default_root_state[:, :3] += self.scene.env_origins[env_ids]

    self.joint_pos[env_ids] = joint_pos
    self.joint_vel[env_ids] = joint_vel

    self.cartpole.write_root_pose_to_sim(default_root_state[:, :7], env_ids)
    self.cartpole.write_root_velocity_to_sim(default_root_state[:, 7:], env_ids)
    self.cartpole.write_joint_state_to_sim(joint_pos, joint_vel, None, env_ids)
```

​		执行动作的函数分为加载动作到缓存区和应用动作两步：

```python
def _pre_physics_step(self, actions: torch.Tensor) -> None:
	self.actions = self.action_scale * actions.clone()

def _apply_action(self) -> None:
	self.cartpole.set_joint_effort_target(self.actions, joint_ids=self._cart_dof_idx)
```

#### 3.4.5 注册环境

​		之前的方法中没有注册环境，而是直接将管理配置类传入管理类来建立一个环境实例，将环境注册到gymnasium注册表中可以使得这一步骤更加简洁和可拓展。

​		注册后，仅仅需要使用train.py 脚本，并将任务名以参数的形式传入即可。

​		注册环境首先需要make一下环境，得到一个env，env可以用于代理环境的运行，如下：

```python
def main():
    """Random actions agent with Isaac Lab environment."""
    # create environment configuration
    env_cfg = parse_env_cfg(
        args_cli.task, device=args_cli.device, num_envs=args_cli.num_envs, use_fabric=not args_cli.disable_fabric
    )
    # create environment
    env = gym.make(args_cli.task, cfg=env_cfg)

    # print info (this is vectorized environment)
    print(f"[INFO]: Gym observation space: {env.observation_space}")
    print(f"[INFO]: Gym action space: {env.action_space}")
    # reset environment
    env.reset()
    # simulate environment
    while simulation_app.is_running():
        # run everything in inference mode
        with torch.inference_mode():
            # sample actions from -1 to 1
            actions = 2 * torch.rand(env.action_space.shape, device=env.unwrapped.device) - 1
            # apply actions
            env.step(actions)

    # close the simulator
    env.close()
```

​		使用gym.register来注册一个环境，在环境包的__init__.py中完成。需要的参数有id、entry_point、环境额外的参数kwargs、disable_env_checker。

```python
gym.register(
    id="Isaac-Cartpole-RGB-TheiaTiny-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.cartpole_camera_env_cfg:CartpoleTheiaTinyCameraEnvCfg",
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_feature_ppo_cfg.yaml",
    },
)
```

​		其中id是环境名称，一般以Isaac-作为前缀，后接任务名，最后是机器人名与版本号。entry-point是环境类的入口点，入口点表示为<module>:<class>的形式，入口点用于在创建环境实例的时候导入环境类；kwargs设置一些配置入口点，配置入口点可以是配置类或一个Yaml文件。

​		在基于直接调用的实现中，将环境的入口定义为自己实现的那个类，而不是DirectRLEnv。

```python
import gymnasium as gym

from . import agents

##
# Register Gym environments.
##

gym.register(
    id="Isaac-Cartpole-Direct-v0",
    entry_point=f"{__name__}.cartpole_env:CartpoleEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": f"{__name__}.cartpole_env:CartpoleEnvCfg",
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:CartpolePPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
        "sb3_cfg_entry_point": f"{agents.__name__}:sb3_ppo_cfg.yaml",
    },
)
```

​		在使用gym来注册时，需要导入isaaclab_tasks，这会遍历所有的子包并注册它们各自的环境。

​		此外，ManagerBasedRLEnv虽然符合gymnasium.Env接口，但它并不是一个标准的gym，因为输入输出是torch tensors 而不是numpy。

#### 3.4.6 使用RL Agent进行训练

​		每个学习库下有train.py脚本，包含了训练的配置。

​		由于不同学习库使用的API接口要求的数据类型和格式不一，在ManagerBasedEnv中没有基于任意特定的学习库来设计，而是需要通过各个包装器将环境转换为所期望的接口。

​		在使用gym.make后，环境变成了gymnasium代理的，但是于强化学习库不兼容，此时使用包装器转化为符合某个强化学习库的兼容格式，如Sb3VecEnvWrapper。

```python
# wrap for video recording
if args_cli.video:
    video_kwargs = {
    "video_folder": os.path.join(log_dir, "videos", "train"),
    "step_trigger": lambda step: step % args_cli.video_interval == 0,
    "video_length": args_cli.video_length,
    "disable_logger": True,
    }
    print("[INFO] Recording videos during training.")
    print_dict(video_kwargs, nesting=4)
    env = gym.wrappers.RecordVideo(env, **video_kwargs)

# wrap around environment for stable baselines
env = Sb3VecEnvWrapper(env)

if "normalize_input" in agent_cfg:
    env = VecNormalize(
        env,
        training=True,
        norm_obs="normalize_input" in agent_cfg and agent_cfg.pop("normalize_input"),
        norm_reward="normalize_value" in agent_cfg and agent_cfg.pop("normalize_value"),
        clip_obs="clip_obs" in agent_cfg and agent_cfg.pop("clip_obs"),
        gamma=agent_cfg["gamma"],
        clip_reward=np.inf,
    )
```

​		包装器RecordVideo用于记录环境的视频并保存到指定目录

​		包装器Sb3VecEnvWrapper(env)用于将env转化为SB3兼容

​		包装器VecNormalize用于标准化环境的奖励和观测。

​		包装后，可以使用各个强化学习库建立Agent，并进行学习：

```python
# create agent from stable baselines
agent = PPO(policy_arch, env, verbose=1, **agent_cfg)
# configure the logger
new_logger = configure(log_dir, ["stdout", "tensorboard"])
agent.set_logger(new_logger)

# callbacks for agent
checkpoint_callback = CheckpointCallback(save_freq=1000, save_path=log_dir, name_prefix="model", verbose=2)
# train the agent
agent.learn(total_timesteps=n_timesteps, callback=checkpoint_callback)
# save the final model
agent.save(os.path.join(log_dir, "model"))

# close the simulator
env.close()

```

+ 执行训练过程

​		无界面headless模式：此时只执行物理模拟步骤

```python
python scripts/reinforcement_learning/sb3/train.py --task Isaac-Cartpole-v0 --num_envs 64 --headless
```

​		无界面但离屏渲染camera，此时虽然没有界面，但是可以渲染video，并将渲染的视频保存在logs/sb3/Isaac-Cartpole-v0/<run-dir>/videos/train中。

```python
python scripts/reinforcement_learning/sb3/train.py --task Isaac-Cartpole-v0 --num_envs 64 --headless --video
```

​		交互模式

```python
./isaaclab.sh -p scripts/reinforcement_learning/sb3/train.py --task Isaac-Cartpole-v0 --num_envs 64
```

​		使用tensorboard监视训练进度

```python
python -m tensorboard.main --logdir logs/sb3/Isaac-Cartpole-v0
```

​		加载训练参数与play

```python
./isaaclab.sh -p scripts/reinforcement_learning/sb3/play.py --task Isaac-Cartpole-v0 --num_envs 32 --use_last_checkpoint

./isaaclab.sh -p scripts/reinforcement_learning/rl_games/play.py --task Isaac-H1-Direct-v0 --num_envs 2
```

#### 3.4.7 修改现有的RL环境

> IsaacLab提供了一些任务例子，可以建立自己的任务或者直接修改例子

​		RL 环境主要以任务的形式表述，源文件放在isaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/robot_name下

​		这是以一个python包的形式封装的文件，包含agents文件夹、__init__.py文件以及一些此机器人使用的task的python脚本，每个脚本应该对应一个task。

​		在__init__.py文件将环境注册到gym中

```python
gym.register(
    id="Isaac-H1-Direct-v0",
    entry_point="isaaclab_tasks.direct.humanoid:H1Env",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": H1EnvCfg,
        "rl_games_cfg_entry_point": f"{agents.__name__}:rl_games_ppo_cfg.yaml",
        "rsl_rl_cfg_entry_point": f"{agents.__name__}.rsl_rl_ppo_cfg:HumanoidPPORunnerCfg",
        "skrl_cfg_entry_point": f"{agents.__name__}:skrl_ppo_cfg.yaml",
    },
)
```

​		其它的实现可以基于已有的任务构建，选择一个任务源文件，简单复制重命名后，按照自己的需要修改配置类（包括机器人配置、场景配置、强化学习环境参数、仿真配置、奖励缩放配置等）

#### 3.4.8 USD中部署策略

​		在完成训练后，将训练的策略导出为离线策略（JIT），然后可以自定义一个新的usd的配置场景，将JIT策略在机器人上进行推理。

```python
def main():
    """Main function."""
    # load the trained jit policy
    policy_path = os.path.abspath(args_cli.checkpoint)
    file_content = omni.client.read_file(policy_path)[2]
    file = io.BytesIO(memoryview(file_content).tobytes())
    policy = torch.jit.load(file, map_location=args_cli.device)

    # setup environment
    env_cfg = H1RoughEnvCfg_PLAY()
    env_cfg.scene.num_envs = 1
    env_cfg.curriculum = None
    env_cfg.scene.terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="usd",
        usd_path=f"{ISAAC_NUCLEUS_DIR}/Environments/Simple_Warehouse/warehouse.usd",
    )
    env_cfg.sim.device = args_cli.device
    if args_cli.device == "cpu":
        env_cfg.sim.use_fabric = False

    # create environment
    env = ManagerBasedRLEnv(cfg=env_cfg)

    # run inference with the policy
    obs, _ = env.reset()
    with torch.inference_mode():
        while simulation_app.is_running():
            action = policy(obs["policy"])
            obs, _, _, _, _ = env.step(action)


if __name__ == "__main__":
    main()
    simulation_app.close()

```

​		首先需要完成训练：

```bash
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Velocity-Rough-H1-v0 --headless
```

​		可以使用tensorboard查看训练日志。之后运行play脚本，同时会将策略导出为jit或onnx文件：

```python
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py --task Isaac-Velocity-Rough-H1-v0 --num_envs 64 --checkpoint logs/rsl_rl/h1_rough/EXPERIMENT_NAME/POLICY_FILE.pt
```

​		最后使用导出的jit策略在新的USD场景及机器人上推理：

```python
./isaaclab.sh -p scripts/tutorials/03_envs/policy_inference_in_usd.py --checkpoint logs/rsl_rl/h1_rough/EXPERIMENT_NAME/exported/policy.pt
```

#### 3.4.9 环境包装

> 环境包装器是一种修改环境行为而不修改环境本身的方法，可以用来修改观测、记录是视频、强制时间限制等。

​		使用包装器里的强制重置方法，来在step、render之前强制调用reset方法：

```python
"""Launch Isaac Sim Simulator first."""


from isaaclab.app import AppLauncher

# launch omniverse app in headless mode
app_launcher = AppLauncher(headless=True)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym

import isaaclab_tasks  # noqa: F401
from isaaclab_tasks.utils import load_cfg_from_registry

# create base environment
cfg = load_cfg_from_registry("Isaac-Reach-Franka-v0", "env_cfg_entry_point")
env = gym.make("Isaac-Reach-Franka-v0", cfg=cfg)
# wrap environment to enforce that reset is called before step
env = gym.wrappers.OrderEnforcing(env)
```

​		使用包装器里的记录视频，首先需要设置相机的默认参数：

```python
@configclass
class ViewerCfg:
    """Configuration of the scene viewport camera."""

    eye: tuple[float, float, float] = (7.5, 7.5, 7.5)
    """Initial camera position (in m). Default is (7.5, 7.5, 7.5)."""

    lookat: tuple[float, float, float] = (0.0, 0.0, 0.0)
    """Initial camera target position (in m). Default is (0.0, 0.0, 0.0)."""

    cam_prim_path: str = "/OmniverseKit_Persp"
    """The camera prim path to record images from. Default is "/OmniverseKit_Persp",
    which is the default camera in the viewport.
    """

    resolution: tuple[int, int] = (1280, 720)
    """The resolution (width, height) of the camera specified using :attr:`cam_prim_path`.
    Default is (1280, 720).
    """

    origin_type: Literal["world", "env", "asset_root", "asset_body"] = "world"
    """The frame in which the camera position (eye) and target (lookat) are defined in. Default is "world".

    Available options are:

    * ``"world"``: The origin of the world.
    * ``"env"``: The origin of the environment defined by :attr:`env_index`.
    * ``"asset_root"``: The center of the asset defined by :attr:`asset_name` in environment :attr:`env_index`.
    * ``"asset_body"``: The center of the body defined by :attr:`body_name` in asset defined by :attr:`asset_name` in environment :attr:`env_index`.
    """

    env_index: int = 0
    """The environment index for frame origin. Default is 0.

    This quantity is only effective if :attr:`origin` is set to "env" or "asset_root".
    """

    asset_name: str | None = None
    """The asset name in the interactive scene for the frame origin. Default is None.

    This quantity is only effective if :attr:`origin` is set to "asset_root".
    """

    body_name: str | None = None
    """The name of the body in :attr:`asset_name` in the interactive scene for the frame origin. Default is None.

    This quantity is only effective if :attr:`origin` is set to "asset_body".
    """
```

​		然后使用包装器在离屏渲染下记录视频：

```python
"""Launch Isaac Sim Simulator first."""


from isaaclab.app import AppLauncher

# launch omniverse app in headless mode with off-screen rendering
app_launcher = AppLauncher(headless=True, enable_cameras=True)
simulation_app = app_launcher.app

"""Rest everything follows."""

import gymnasium as gym

# adjust camera resolution and pose
env_cfg.viewer.resolution = (640, 480)
env_cfg.viewer.eye = (1.0, 1.0, 1.0)
env_cfg.viewer.lookat = (0.0, 0.0, 0.0)
# create isaac-env instance
# set render mode to rgb_array to obtain images on render calls
env = gym.make(task_name, cfg=env_cfg, render_mode="rgb_array")
# wrap for video recording
video_kwargs = {
    "video_folder": "videos/train",
    "step_trigger": lambda step: step % 1500 == 0,
    "video_length": 200,
}
env = gym.wrappers.RecordVideo(env, **video_kwargs)
```

​		不同的强化学习库有不同的API接口，因此ManagerBasedRLEnv和DirectRLEnv的接口不建立在已有的强化学习库上，而是使用包装器来使得接口与强化学习框架兼容。

​		如使得注册的环境与SB3的接口兼容：

```python
from isaaclab_rl.sb3 import Sb3VecEnvWrapper

# create isaac-env instance
env = gym.make(task_name, cfg=env_cfg)
# wrap around environment for stable baselines
env = Sb3VecEnvWrapper(env)
```

​		还可以使用unwrapped()属性来查看环境是否可以应用某个包装器。

#### 3.4.10 添加学习库

​		IsaacLab支持任何低层python的强化学习库，强化学习库应当支持python3.10。

+ 使用库的不同版本

​		如果要安装不同版本的已经兼容的强化学习库或使用自己修改过的版本：

​		首先按照默认脚本安装指定的库

​		然后在要安装的版本的源码下运行：

```python
# Assuming you are in the root directory of the Isaac Lab repository
cd IsaacLab

# Note: If you are using a virtual environment, make sure to activate it before running the following command
./isaaclab.sh -p -m pip install -e /path/to/rsl_rl
```

​		此时会自动安装该库到默认环境中，使用以下命令查看版本信息：

```python
./isaaclab.sh -p -m pip show rsl-rl-lib
```

+ 集成新的库

​		集成新的库可以直接将该库安装到虚拟环境中，然后为该库制作一个包装器。

​		首先在source/isaaclab_tasks/setup.py中将该库添加到依赖项INSTALL_REQUIRES中，然后安装该库，可参考使用库的不同版本中的安装方法。

​		为库创建包装器，具体参考source/isaaclab_rl中的实现。

​		为库创建workflower脚本训练和评估智能体，参考scripts/reinforcement_learning中的实现。

​		可选：添加一些测试在source/isaaclab_rl/test中，添加一些文档在docs/source/api/lab_tasks/isaaclab_rl.rst。

### 3.5 空间规划器

#### 3.5.1 任务空间规划器

> 任务空间规划器对末端点路径进行规划，使用逆运动学求解关节角度

​		使用DifferentialIKController, DifferentialIKControllerCfg创建一个控制器和控制器配置类，即可建立自己的空间规划器。

```python
diff_ik_cfg = DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls")
diff_ik_controller = DifferentialIKController(diff_ik_cfg, num_envs=scene.num_envs, device=sim.device)
```

​		空间规划器可以选择多种姿态或位置跟踪，一般固定机器人选择pose，移动机器人选择position。

​		发送关节命令：

```python
# reset controller
diff_ik_controller.reset()
diff_ik_controller.set_command(ik_commands)
```

​		发送命令后，机器人并不能立刻运动到位，需要对中间状态进行规划，规划器的选择由配置类参数ik_method决定，需要获取仿真过程中的姿态、雅克比矩阵、当前关节位置的输入，以计算下一个关节指令：

```python
jacobian = robot.root_physx_view.get_jacobians()[:, ee_jacobi_idx, :, robot_entity_cfg.joint_ids]
ee_pose_w = robot.data.body_state_w[:, robot_entity_cfg.body_ids[0], 0:7]
root_pose_w = robot.data.root_state_w[:, 0:7]
joint_pos = robot.data.joint_pos[:, robot_entity_cfg.joint_ids]
# compute frame in root frame
ee_pos_b, ee_quat_b = subtract_frame_transforms(
root_pose_w[:, 0:3], root_pose_w[:, 3:7], ee_pose_w[:, 0:3], ee_pose_w[:, 3:7]
)
# compute the joint commands
joint_pos_des = diff_ik_controller.compute(ee_pos_b, ee_quat_b, jacobian, joint_pos)
```

​		控制器只负责规划，得到每一步的关节输入，需要最后将关节位置目标应用到机器人上。

```python
# apply actions
robot.set_joint_position_target(joint_pos_des, joint_ids=robot_entity_cfg.joint_ids)
scene.write_data_to_sim()
```

​		整体流程：

```python
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
This script demonstrates how to use the differential inverse kinematics controller with the simulator.

The differential IK controller can be configured in different modes. It uses the Jacobians computed by
PhysX. This helps perform parallelized computation of the inverse kinematics.

.. code-block:: bash

    # Usage
    ./isaaclab.sh -p scripts/tutorials/05_controllers/run_diff_ik.py

"""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on using the differential IK controller.")
parser.add_argument("--robot", type=str, default="franka_panda", help="Name of the robot.")
parser.add_argument("--num_envs", type=int, default=128, help="Number of environments to spawn.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.controllers import DifferentialIKController, DifferentialIKControllerCfg
from isaaclab.managers import SceneEntityCfg
from isaaclab.markers import VisualizationMarkers
from isaaclab.markers.config import FRAME_MARKER_CFG
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.utils import configclass
from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR
from isaaclab.utils.math import subtract_frame_transforms

##
# Pre-defined configs
##
from isaaclab_assets import FRANKA_PANDA_HIGH_PD_CFG, UR10_CFG  # isort:skip


@configclass
class TableTopSceneCfg(InteractiveSceneCfg):
    """Configuration for a cart-pole scene."""

    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
        init_state=AssetBaseCfg.InitialStateCfg(pos=(0.0, 0.0, -1.05)),
    )

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # mount
    table = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/Table",
        spawn=sim_utils.UsdFileCfg(
            usd_path=f"{ISAAC_NUCLEUS_DIR}/Props/Mounts/Stand/stand_instanceable.usd", scale=(2.0, 2.0, 2.0)
        ),
    )

    # articulation
    if args_cli.robot == "franka_panda":
        robot = FRANKA_PANDA_HIGH_PD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    elif args_cli.robot == "ur10":
        robot = UR10_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    else:
        raise ValueError(f"Robot {args_cli.robot} is not supported. Valid: franka_panda, ur10")


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    """Runs the simulation loop."""
    # Extract scene entities
    # note: we only do this here for readability.
    robot = scene["robot"]

    # Create controller
    diff_ik_cfg = DifferentialIKControllerCfg(command_type="pose", use_relative_mode=False, ik_method="dls")
    diff_ik_controller = DifferentialIKController(diff_ik_cfg, num_envs=scene.num_envs, device=sim.device)

    # Markers
    frame_marker_cfg = FRAME_MARKER_CFG.copy()
    frame_marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
    ee_marker = VisualizationMarkers(frame_marker_cfg.replace(prim_path="/Visuals/ee_current"))
    goal_marker = VisualizationMarkers(frame_marker_cfg.replace(prim_path="/Visuals/ee_goal"))

    # Define goals for the arm
    ee_goals = [
        [0.5, 0.5, 0.7, 0.707, 0, 0.707, 0],
        [0.5, -0.4, 0.6, 0.707, 0.707, 0.0, 0.0],
        [0.5, 0, 0.5, 0.0, 1.0, 0.0, 0.0],
    ]
    ee_goals = torch.tensor(ee_goals, device=sim.device)
    # Track the given command
    current_goal_idx = 0
    # Create buffers to store actions
    ik_commands = torch.zeros(scene.num_envs, diff_ik_controller.action_dim, device=robot.device)
    ik_commands[:] = ee_goals[current_goal_idx]

    # Specify robot-specific parameters
    if args_cli.robot == "franka_panda":
        robot_entity_cfg = SceneEntityCfg("robot", joint_names=["panda_joint.*"], body_names=["panda_hand"])
    elif args_cli.robot == "ur10":
        robot_entity_cfg = SceneEntityCfg("robot", joint_names=[".*"], body_names=["ee_link"])
    else:
        raise ValueError(f"Robot {args_cli.robot} is not supported. Valid: franka_panda, ur10")
    # Resolving the scene entities
    robot_entity_cfg.resolve(scene)
    # Obtain the frame index of the end-effector
    # For a fixed base robot, the frame index is one less than the body index. This is because
    # the root body is not included in the returned Jacobians.
    if robot.is_fixed_base:
        ee_jacobi_idx = robot_entity_cfg.body_ids[0] - 1
    else:
        ee_jacobi_idx = robot_entity_cfg.body_ids[0]

    # Define simulation stepping
    sim_dt = sim.get_physics_dt()
    count = 0
    # Simulation loop
    while simulation_app.is_running():
        # reset
        if count % 150 == 0:
            # reset time
            count = 0
            # reset joint state
            joint_pos = robot.data.default_joint_pos.clone()
            joint_vel = robot.data.default_joint_vel.clone()
            robot.write_joint_state_to_sim(joint_pos, joint_vel)
            robot.reset()
            # reset actions
            ik_commands[:] = ee_goals[current_goal_idx]
            joint_pos_des = joint_pos[:, robot_entity_cfg.joint_ids].clone()
            # reset controller
            diff_ik_controller.reset()
            diff_ik_controller.set_command(ik_commands)
            # change goal
            current_goal_idx = (current_goal_idx + 1) % len(ee_goals)
        else:
            # obtain quantities from simulation
            jacobian = robot.root_physx_view.get_jacobians()[:, ee_jacobi_idx, :, robot_entity_cfg.joint_ids]
            ee_pose_w = robot.data.body_state_w[:, robot_entity_cfg.body_ids[0], 0:7]
            root_pose_w = robot.data.root_state_w[:, 0:7]
            joint_pos = robot.data.joint_pos[:, robot_entity_cfg.joint_ids]
            # compute frame in root frame
            ee_pos_b, ee_quat_b = subtract_frame_transforms(
                root_pose_w[:, 0:3], root_pose_w[:, 3:7], ee_pose_w[:, 0:3], ee_pose_w[:, 3:7]
            )
            # compute the joint commands
            joint_pos_des = diff_ik_controller.compute(ee_pos_b, ee_quat_b, jacobian, joint_pos)

        # apply actions
        robot.set_joint_position_target(joint_pos_des, joint_ids=robot_entity_cfg.joint_ids)
        scene.write_data_to_sim()
        # perform step
        sim.step()
        # update sim-time
        count += 1
        # update buffers
        scene.update(sim_dt)

        # obtain quantities from simulation
        ee_pose_w = robot.data.body_state_w[:, robot_entity_cfg.body_ids[0], 0:7]
        # update marker positions
        ee_marker.visualize(ee_pose_w[:, 0:3], ee_pose_w[:, 3:7])
        goal_marker.visualize(ik_commands[:, 0:3] + scene.env_origins, ik_commands[:, 3:7])


def main():
    """Main function."""
    # Load kit helper
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.0])
    # Design scene
    scene_cfg = TableTopSceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()

```

#### 3.5.2 运动空间规划器

> 当需要更加复杂的控制，如力、力矩控制；给机器人施加额外的力等，需要使用操作空间控制器OSC

​		如图，机械臂在跟踪目标运动过程中会与斜面发生碰撞以受到额外的力，此时需要使用OSC。

![image-20250513222011869](/media/airporal/8333B1863791CF8A/learn/Notebook/IsaacLab.assets/image-20250513222011869.png)

​		OperationalSpaceController在机器人任务空间的规划过程中同时进行运动和关节力、力矩的计算。任务空间的参考系可以是欧几里德空间下的任意坐标系，默认是机器人的基座标系。使用OperationalSpaceControllerCfg创建OSC规划器的配置：

```python
# Create the OSC
osc_cfg = OperationalSpaceControllerCfg(
    target_types=["pose_abs", "wrench_abs"],
    impedance_mode="variable_kp",
    inertial_dynamics_decoupling=True,
    partial_inertial_dynamics_decoupling=False,
    gravity_compensation=False,
    motion_damping_ratio_task=1.0,
    contact_wrench_stiffness_task=[0.0, 0.0, 0.1, 0.0, 0.0, 0.0],
    motion_control_axes_task=[1, 1, 0, 1, 1, 1],
    contact_wrench_control_axes_task=[0, 0, 1, 0, 0, 0],
    nullspace_control="position",
)
osc = OperationalSpaceController(osc_cfg, num_envs=scene.num_envs, device=sim.device)
```

​		参数target_types用来设置控制模式：

+ 对于运动控制：pose_abs表示相对于基坐标的当前姿态，还可以设置为pose_rel表示相对于末端执行器的当前姿态；
+ 对于力矩控制：设置为force_abs
+ 对于同时应用力、力矩控制，可以设置为：`["pose_abs", "wrench_abs"]` 或 `["pose_rel", "wrench_abs"]` 。

​		根据实际原理，设置motion_control_axes_task、force_control_axes_task来指定力或力矩所应用的轴。

​		motion_damping_ratio_task、motion_control_stiffness来指定刚度和阻尼的比值。如果需要在运行时更改这些值，则需要将impedance_mode设置为variable_kp（表示刚度可变）或variable(表示刚度和阻尼可变)，此时，还需要设置motion_stiffness_limits_task 和motion_damping_limits_task来限制刚度和阻尼的变化范围。

​		不设置contact_wrench_stiffness_task表示使用开环力控制，设置后可以引入设置的值作为刚度来实现闭环力控制。partial_inertial_dynamics_decoupling设置为True可以忽略平移和旋转轴之间的惯性耦合。

​		gravity_compensation表示是否进行重力补偿。

​		自由度冗余的机器人在运动时存在运动学的零空间，零空间是机器人空间姿态不变而关节位置改变的状态，零空间可以用来进行多任务优化，控制零空间使得放在满足主要任务的前提下进行壁障、力控制等。nullspace_control参数用于零空间设置，零空间的解偶在inertial_dynamics_decouping为True和partial_inertial_dynamics_decoupling设置为False时才可用。nullspace_control默认为none，可以设置为position，表示集成一个零空间的PD控制器。

​		使用时将命令转化为任务空间，然后计算关节指令，最后应用于机器人。整体代码如下：

```python
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""
This script demonstrates how to use the operational space controller (OSC) with the simulator.

The OSC controller can be configured in different modes. It uses the dynamical quantities such as Jacobians and
mass matricescomputed by PhysX.

.. code-block:: bash

    # Usage
    ./isaaclab.sh -p scripts/tutorials/05_controllers/run_osc.py

"""

"""Launch Isaac Sim Simulator first."""

import argparse

from isaaclab.app import AppLauncher

# add argparse arguments
parser = argparse.ArgumentParser(description="Tutorial on using the operational space controller.")
parser.add_argument("--num_envs", type=int, default=128, help="Number of environments to spawn.")
# append AppLauncher cli args
AppLauncher.add_app_launcher_args(parser)
# parse the arguments
args_cli = parser.parse_args()

# launch omniverse app
app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

"""Rest everything follows."""

import torch

import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation, AssetBaseCfg
from isaaclab.controllers import OperationalSpaceController, OperationalSpaceControllerCfg
from isaaclab.markers import VisualizationMarkers
from isaaclab.markers.config import FRAME_MARKER_CFG
from isaaclab.scene import InteractiveScene, InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg
from isaaclab.utils import configclass
from isaaclab.utils.math import (
    combine_frame_transforms,
    matrix_from_quat,
    quat_inv,
    quat_rotate_inverse,
    subtract_frame_transforms,
)

##
# Pre-defined configs
##
from isaaclab_assets import FRANKA_PANDA_HIGH_PD_CFG  # isort:skip


@configclass
class SceneCfg(InteractiveSceneCfg):
    """Configuration for a simple scene with a tilted wall."""

    # ground plane
    ground = AssetBaseCfg(
        prim_path="/World/defaultGroundPlane",
        spawn=sim_utils.GroundPlaneCfg(),
    )

    # lights
    dome_light = AssetBaseCfg(
        prim_path="/World/Light", spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75))
    )

    # Tilted wall
    tilted_wall = AssetBaseCfg(
        prim_path="{ENV_REGEX_NS}/TiltedWall",
        spawn=sim_utils.CuboidCfg(
            size=(2.0, 1.5, 0.01),
            collision_props=sim_utils.CollisionPropertiesCfg(),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0), opacity=0.1),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(kinematic_enabled=True),
            activate_contact_sensors=True,
        ),
        init_state=AssetBaseCfg.InitialStateCfg(
            pos=(0.6 + 0.085, 0.0, 0.3), rot=(0.9238795325, 0.0, -0.3826834324, 0.0)
        ),
    )

    contact_forces = ContactSensorCfg(
        prim_path="/World/envs/env_.*/TiltedWall",
        update_period=0.0,
        history_length=2,
        debug_vis=False,
    )

    robot = FRANKA_PANDA_HIGH_PD_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")
    robot.actuators["panda_shoulder"].stiffness = 0.0
    robot.actuators["panda_shoulder"].damping = 0.0
    robot.actuators["panda_forearm"].stiffness = 0.0
    robot.actuators["panda_forearm"].damping = 0.0
    robot.spawn.rigid_props.disable_gravity = True


def run_simulator(sim: sim_utils.SimulationContext, scene: InteractiveScene):
    """Runs the simulation loop.

    Args:
        sim: (SimulationContext) Simulation context.
        scene: (InteractiveScene) Interactive scene.
    """

    # Extract scene entities for readability.
    robot = scene["robot"]
    contact_forces = scene["contact_forces"]

    # Obtain indices for the end-effector and arm joints
    ee_frame_name = "panda_leftfinger"
    arm_joint_names = ["panda_joint.*"]
    ee_frame_idx = robot.find_bodies(ee_frame_name)[0][0]
    arm_joint_ids = robot.find_joints(arm_joint_names)[0]

    # Create the OSC
    osc_cfg = OperationalSpaceControllerCfg(
        target_types=["pose_abs", "wrench_abs"],
        impedance_mode="variable_kp",
        inertial_dynamics_decoupling=True,
        partial_inertial_dynamics_decoupling=False,
        gravity_compensation=False,
        motion_damping_ratio_task=1.0,
        contact_wrench_stiffness_task=[0.0, 0.0, 0.1, 0.0, 0.0, 0.0],
        motion_control_axes_task=[1, 1, 0, 1, 1, 1],
        contact_wrench_control_axes_task=[0, 0, 1, 0, 0, 0],
        nullspace_control="position",
    )
    osc = OperationalSpaceController(osc_cfg, num_envs=scene.num_envs, device=sim.device)

    # Markers
    frame_marker_cfg = FRAME_MARKER_CFG.copy()
    frame_marker_cfg.markers["frame"].scale = (0.1, 0.1, 0.1)
    ee_marker = VisualizationMarkers(frame_marker_cfg.replace(prim_path="/Visuals/ee_current"))
    goal_marker = VisualizationMarkers(frame_marker_cfg.replace(prim_path="/Visuals/ee_goal"))

    # Define targets for the arm
    ee_goal_pose_set_tilted_b = torch.tensor(
        [
            [0.6, 0.15, 0.3, 0.0, 0.92387953, 0.0, 0.38268343],
            [0.6, -0.3, 0.3, 0.0, 0.92387953, 0.0, 0.38268343],
            [0.8, 0.0, 0.5, 0.0, 0.92387953, 0.0, 0.38268343],
        ],
        device=sim.device,
    )
    ee_goal_wrench_set_tilted_task = torch.tensor(
        [
            [0.0, 0.0, 10.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 10.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 10.0, 0.0, 0.0, 0.0],
        ],
        device=sim.device,
    )
    kp_set_task = torch.tensor(
        [
            [360.0, 360.0, 360.0, 360.0, 360.0, 360.0],
            [420.0, 420.0, 420.0, 420.0, 420.0, 420.0],
            [320.0, 320.0, 320.0, 320.0, 320.0, 320.0],
        ],
        device=sim.device,
    )
    ee_target_set = torch.cat([ee_goal_pose_set_tilted_b, ee_goal_wrench_set_tilted_task, kp_set_task], dim=-1)

    # Define simulation stepping
    sim_dt = sim.get_physics_dt()

    # Update existing buffers
    # Note: We need to update buffers before the first step for the controller.
    robot.update(dt=sim_dt)

    # Get the center of the robot soft joint limits
    joint_centers = torch.mean(robot.data.soft_joint_pos_limits[:, arm_joint_ids, :], dim=-1)

    # get the updated states
    (
        jacobian_b,
        mass_matrix,
        gravity,
        ee_pose_b,
        ee_vel_b,
        root_pose_w,
        ee_pose_w,
        ee_force_b,
        joint_pos,
        joint_vel,
    ) = update_states(sim, scene, robot, ee_frame_idx, arm_joint_ids, contact_forces)

    # Track the given target command
    current_goal_idx = 0  # Current goal index for the arm
    command = torch.zeros(
        scene.num_envs, osc.action_dim, device=sim.device
    )  # Generic target command, which can be pose, position, force, etc.
    ee_target_pose_b = torch.zeros(scene.num_envs, 7, device=sim.device)  # Target pose in the body frame
    ee_target_pose_w = torch.zeros(scene.num_envs, 7, device=sim.device)  # Target pose in the world frame (for marker)

    # Set joint efforts to zero
    zero_joint_efforts = torch.zeros(scene.num_envs, robot.num_joints, device=sim.device)
    joint_efforts = torch.zeros(scene.num_envs, len(arm_joint_ids), device=sim.device)

    count = 0
    # Simulation loop
    while simulation_app.is_running():
        # reset every 500 steps
        if count % 500 == 0:
            # reset joint state to default
            default_joint_pos = robot.data.default_joint_pos.clone()
            default_joint_vel = robot.data.default_joint_vel.clone()
            robot.write_joint_state_to_sim(default_joint_pos, default_joint_vel)
            robot.set_joint_effort_target(zero_joint_efforts)  # Set zero torques in the initial step
            robot.write_data_to_sim()
            robot.reset()
            # reset contact sensor
            contact_forces.reset()
            # reset target pose
            robot.update(sim_dt)
            _, _, _, ee_pose_b, _, _, _, _, _, _ = update_states(
                sim, scene, robot, ee_frame_idx, arm_joint_ids, contact_forces
            )  # at reset, the jacobians are not updated to the latest state
            command, ee_target_pose_b, ee_target_pose_w, current_goal_idx = update_target(
                sim, scene, osc, root_pose_w, ee_target_set, current_goal_idx
            )
            # set the osc command
            osc.reset()
            command, task_frame_pose_b = convert_to_task_frame(osc, command=command, ee_target_pose_b=ee_target_pose_b)
            osc.set_command(command=command, current_ee_pose_b=ee_pose_b, current_task_frame_pose_b=task_frame_pose_b)
        else:
            # get the updated states
            (
                jacobian_b,
                mass_matrix,
                gravity,
                ee_pose_b,
                ee_vel_b,
                root_pose_w,
                ee_pose_w,
                ee_force_b,
                joint_pos,
                joint_vel,
            ) = update_states(sim, scene, robot, ee_frame_idx, arm_joint_ids, contact_forces)
            # compute the joint commands
            joint_efforts = osc.compute(
                jacobian_b=jacobian_b,
                current_ee_pose_b=ee_pose_b,
                current_ee_vel_b=ee_vel_b,
                current_ee_force_b=ee_force_b,
                mass_matrix=mass_matrix,
                gravity=gravity,
                current_joint_pos=joint_pos,
                current_joint_vel=joint_vel,
                nullspace_joint_pos_target=joint_centers,
            )
            # apply actions
            robot.set_joint_effort_target(joint_efforts, joint_ids=arm_joint_ids)
            robot.write_data_to_sim()

        # update marker positions
        ee_marker.visualize(ee_pose_w[:, 0:3], ee_pose_w[:, 3:7])
        goal_marker.visualize(ee_target_pose_w[:, 0:3], ee_target_pose_w[:, 3:7])

        # perform step
        sim.step(render=True)
        # update robot buffers
        robot.update(sim_dt)
        # update buffers
        scene.update(sim_dt)
        # update sim-time
        count += 1


# Update robot states
def update_states(
    sim: sim_utils.SimulationContext,
    scene: InteractiveScene,
    robot: Articulation,
    ee_frame_idx: int,
    arm_joint_ids: list[int],
    contact_forces,
):
    """Update the robot states.

    Args:
        sim: (SimulationContext) Simulation context.
        scene: (InteractiveScene) Interactive scene.
        robot: (Articulation) Robot articulation.
        ee_frame_idx: (int) End-effector frame index.
        arm_joint_ids: (list[int]) Arm joint indices.
        contact_forces: (ContactSensor) Contact sensor.

    Returns:
        jacobian_b (torch.tensor): Jacobian in the body frame.
        mass_matrix (torch.tensor): Mass matrix.
        gravity (torch.tensor): Gravity vector.
        ee_pose_b (torch.tensor): End-effector pose in the body frame.
        ee_vel_b (torch.tensor): End-effector velocity in the body frame.
        root_pose_w (torch.tensor): Root pose in the world frame.
        ee_pose_w (torch.tensor): End-effector pose in the world frame.
        ee_force_b (torch.tensor): End-effector force in the body frame.
        joint_pos (torch.tensor): The joint positions.
        joint_vel (torch.tensor): The joint velocities.

    Raises:
        ValueError: Undefined target_type.
    """
    # obtain dynamics related quantities from simulation
    ee_jacobi_idx = ee_frame_idx - 1
    jacobian_w = robot.root_physx_view.get_jacobians()[:, ee_jacobi_idx, :, arm_joint_ids]
    mass_matrix = robot.root_physx_view.get_generalized_mass_matrices()[:, arm_joint_ids, :][:, :, arm_joint_ids]
    gravity = robot.root_physx_view.get_gravity_compensation_forces()[:, arm_joint_ids]
    # Convert the Jacobian from world to root frame
    jacobian_b = jacobian_w.clone()
    root_rot_matrix = matrix_from_quat(quat_inv(robot.data.root_quat_w))
    jacobian_b[:, :3, :] = torch.bmm(root_rot_matrix, jacobian_b[:, :3, :])
    jacobian_b[:, 3:, :] = torch.bmm(root_rot_matrix, jacobian_b[:, 3:, :])

    # Compute current pose of the end-effector
    root_pos_w = robot.data.root_pos_w
    root_quat_w = robot.data.root_quat_w
    ee_pos_w = robot.data.body_pos_w[:, ee_frame_idx]
    ee_quat_w = robot.data.body_quat_w[:, ee_frame_idx]
    ee_pos_b, ee_quat_b = subtract_frame_transforms(root_pos_w, root_quat_w, ee_pos_w, ee_quat_w)
    root_pose_w = torch.cat([root_pos_w, root_quat_w], dim=-1)
    ee_pose_w = torch.cat([ee_pos_w, ee_quat_w], dim=-1)
    ee_pose_b = torch.cat([ee_pos_b, ee_quat_b], dim=-1)

    # Compute the current velocity of the end-effector
    ee_vel_w = robot.data.body_vel_w[:, ee_frame_idx, :]  # Extract end-effector velocity in the world frame
    root_vel_w = robot.data.root_vel_w  # Extract root velocity in the world frame
    relative_vel_w = ee_vel_w - root_vel_w  # Compute the relative velocity in the world frame
    ee_lin_vel_b = quat_rotate_inverse(robot.data.root_quat_w, relative_vel_w[:, 0:3])  # From world to root frame
    ee_ang_vel_b = quat_rotate_inverse(robot.data.root_quat_w, relative_vel_w[:, 3:6])
    ee_vel_b = torch.cat([ee_lin_vel_b, ee_ang_vel_b], dim=-1)

    # Calculate the contact force
    ee_force_w = torch.zeros(scene.num_envs, 3, device=sim.device)
    sim_dt = sim.get_physics_dt()
    contact_forces.update(sim_dt)  # update contact sensor
    # Calculate the contact force by averaging over last four time steps (i.e., to smoothen) and
    # taking the max of three surfaces as only one should be the contact of interest
    ee_force_w, _ = torch.max(torch.mean(contact_forces.data.net_forces_w_history, dim=1), dim=1)

    # This is a simplification, only for the sake of testing.
    ee_force_b = ee_force_w

    # Get joint positions and velocities
    joint_pos = robot.data.joint_pos[:, arm_joint_ids]
    joint_vel = robot.data.joint_vel[:, arm_joint_ids]

    return (
        jacobian_b,
        mass_matrix,
        gravity,
        ee_pose_b,
        ee_vel_b,
        root_pose_w,
        ee_pose_w,
        ee_force_b,
        joint_pos,
        joint_vel,
    )


# Update the target commands
def update_target(
    sim: sim_utils.SimulationContext,
    scene: InteractiveScene,
    osc: OperationalSpaceController,
    root_pose_w: torch.tensor,
    ee_target_set: torch.tensor,
    current_goal_idx: int,
):
    """Update the targets for the operational space controller.

    Args:
        sim: (SimulationContext) Simulation context.
        scene: (InteractiveScene) Interactive scene.
        osc: (OperationalSpaceController) Operational space controller.
        root_pose_w: (torch.tensor) Root pose in the world frame.
        ee_target_set: (torch.tensor) End-effector target set.
        current_goal_idx: (int) Current goal index.

    Returns:
        command (torch.tensor): Updated target command.
        ee_target_pose_b (torch.tensor): Updated target pose in the body frame.
        ee_target_pose_w (torch.tensor): Updated target pose in the world frame.
        next_goal_idx (int): Next goal index.

    Raises:
        ValueError: Undefined target_type.
    """

    # update the ee desired command
    command = torch.zeros(scene.num_envs, osc.action_dim, device=sim.device)
    command[:] = ee_target_set[current_goal_idx]

    # update the ee desired pose
    ee_target_pose_b = torch.zeros(scene.num_envs, 7, device=sim.device)
    for target_type in osc.cfg.target_types:
        if target_type == "pose_abs":
            ee_target_pose_b[:] = command[:, :7]
        elif target_type == "wrench_abs":
            pass  # ee_target_pose_b could stay at the root frame for force control, what matters is ee_target_b
        else:
            raise ValueError("Undefined target_type within update_target().")

    # update the target desired pose in world frame (for marker)
    ee_target_pos_w, ee_target_quat_w = combine_frame_transforms(
        root_pose_w[:, 0:3], root_pose_w[:, 3:7], ee_target_pose_b[:, 0:3], ee_target_pose_b[:, 3:7]
    )
    ee_target_pose_w = torch.cat([ee_target_pos_w, ee_target_quat_w], dim=-1)

    next_goal_idx = (current_goal_idx + 1) % len(ee_target_set)

    return command, ee_target_pose_b, ee_target_pose_w, next_goal_idx


# Convert the target commands to the task frame
def convert_to_task_frame(osc: OperationalSpaceController, command: torch.tensor, ee_target_pose_b: torch.tensor):
    """Converts the target commands to the task frame.

    Args:
        osc: OperationalSpaceController object.
        command: Command to be converted.
        ee_target_pose_b: Target pose in the body frame.

    Returns:
        command (torch.tensor): Target command in the task frame.
        task_frame_pose_b (torch.tensor): Target pose in the task frame.

    Raises:
        ValueError: Undefined target_type.
    """
    command = command.clone()
    task_frame_pose_b = ee_target_pose_b.clone()

    cmd_idx = 0
    for target_type in osc.cfg.target_types:
        if target_type == "pose_abs":
            command[:, :3], command[:, 3:7] = subtract_frame_transforms(
                task_frame_pose_b[:, :3], task_frame_pose_b[:, 3:], command[:, :3], command[:, 3:7]
            )
            cmd_idx += 7
        elif target_type == "wrench_abs":
            # These are already defined in target frame for ee_goal_wrench_set_tilted_task (since it is
            # easier), so not transforming
            cmd_idx += 6
        else:
            raise ValueError("Undefined target_type within _convert_to_task_frame().")

    return command, task_frame_pose_b


def main():
    """Main function."""
    # Load kit helper
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device=args_cli.device)
    sim = sim_utils.SimulationContext(sim_cfg)
    # Set main camera
    sim.set_camera_view([2.5, 2.5, 2.5], [0.0, 0.0, 0.0])
    # Design scene
    scene_cfg = SceneCfg(num_envs=args_cli.num_envs, env_spacing=2.0)
    scene = InteractiveScene(scene_cfg)
    # Play the simulator
    sim.reset()
    # Now we are ready!
    print("[INFO]: Setup complete...")
    # Run the simulator
    run_simulator(sim, scene)


if __name__ == "__main__":
    # run the main function
    main()
    # close sim app
    simulation_app.close()

```

---

## 4 构建自己的机器人

> 本部分从头开始在iasscLab中构建机器人

​		要在IsaacLab中构建自己的机器人，整体步骤为：

+ 使用Solidworks或者OnShape完成建模，机器人的结构尽可能简单。
+ 导入到USD格式，可以先导出为URDF或MJCF格式，再使用提供的脚本转为Usd格式，注意，如果网格以stl格式导出则没有表面材质信息，可以使用Blender进行处理。也可以先导出为glb格式，再导入到blender中导出为dae格式。
+ 在task中使用该机器人

```python
# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for a simple Cartpole robot."""


import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

##
# Configuration
##

CARTPOLE_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/Classic/Cartpole/cartpole.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            rigid_body_enabled=True,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=100.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 2.0), joint_pos={"slider_to_cart": 0.0, "cart_to_pole": 0.0}
    ),
    actuators={
        "cart_actuator": ImplicitActuatorCfg(
            joint_names_expr=["slider_to_cart"],
            effort_limit=400.0,
            velocity_limit=100.0,
            stiffness=0.0,
            damping=10.0,
        ),
        "pole_actuator": ImplicitActuatorCfg(
            joint_names_expr=["cart_to_pole"], effort_limit=400.0, velocity_limit=100.0, stiffness=0.0, damping=0.0
        ),
    },
)
"""Configuration for a simple Cartpole robot."""
```

​		导入时，需要配置机器人的驱动信息和一些参数。

​		此外，有关静态碰撞器、刚性物体、固定关节需要[额外配置](https://docs.robotsfan.com/isaaclab/source/how-to/make_fixed_prim.html)。

​		[可以一次复制生成多个刚性物体](https://docs.robotsfan.com/isaaclab/source/how-to/multi_asset_spawning.html)，或在一个prim路径下生成不同的资产，此功能使得可以随机产生多个有一定区别的机器人。

+ 

​		

---

## 5 API、库、类

|                      库名                      | 作用                                                 |
| :--------------------------------------------: | ---------------------------------------------------- |
|                    argparse                    | 处理命令行参数                                       |
|                  isaaclab.app                  | 仿真app相关的功能库                                  |
| import isaacsim.core.utils.prims as prim_utils | 新建、修改Prim及相关操作                             |
|        import isaaclab.sim as sim_utils        | 包含了导入、创建USD、mtl、URDF等文件到仿真器中的工具 |
|                isaaclab_assets                 | 包含机器人、传感器的预先配置文件                     |
|    import isaaclab.utils.math as math_utils    | 包含一些数学函数                                     |
|            import gymnasium as gym             | 注册环境的库                                         |
|                  isaaclab_rl                   | 由于环境兼容转换包装器的API库                        |
|               stable_baselines3                | SB3的学习库                                          |



|                             类名                             |                      作用                       |
| :----------------------------------------------------------: | :---------------------------------------------: |
|           from isaaclab.assets import Articulation           |        机器人关节控制使用的Articulation         |
|         from isaaclab.assets import ArticulationCfg          |       配置Articulation初始化的一个配置类        |
|          from isaaclab.sim import SimulationContext          |     控制仿真开始、步进、获取仿真参数等操作      |
|           from isaaclab.assets import AssetBaseCfg           |         将一个**不可交互的**prim spwan          |
| from isaaclab.assets import DeformableObject, DeformableObjectCfg |           软体对象、软体对象配置的类            |
|      from isaaclab.actuators import ImplicitActuatorCfg      |          Articulation内部的关节配置类           |
|                                                              |                                                 |
| from isaaclab.scene import InteractiveScene, InteractiveSceneCfg |               管理和配置Scene的类               |
| from isaaclab.envs import ManagerBasedEnv, ManagerBasedEnvCfg |       基于管理器构建环境的类及其配置的类        |
| from isaaclab.envs import ManagerBasedRLEnv，ManagerBasedRLEnvCfg |   基于管理器构建强化学习环境的类及其配置的类    |
|               import isaaclab.envs.mdp as mdp                |            ==一些配置管理器的函数==             |
| import isaaclab_tasks.manager_based.classic.cartpole.mdp as mdp | ==通常为特定任务实现特定的mdp==奖励函数在此实现 |
| from isaaclab.managers import ObservationGroupCfg as ObsGroup |                 设置观测配置组                  |
| from isaaclab.managers import ObservationTermCfg as ObsTerm  |                 设置观测配置项                  |
|         from isaaclab.managers import SceneEntityCfg         |     解析Scene中设置的entity，传给事件项参数     |
|   from isaaclab.managers import EventTermCfg as EventTerm    |                   设置事件项                    |
|    from isaaclab.managers import RewardTermCfg as RewTerm    |                设置奖励函数的类                 |
| from isaaclab.managers import TerminationTermCfg as DoneTerm |                设置终止条件的类                 |
|                                                              |                                                 |
| from isaaclab.sensors import CameraCfg, ContactSensorCfg, RayCasterCfg, patterns |                 传感器的配置类                  |
| from isaaclab.sensors import FrameTransformer, FrameTransformerCfg, OffsetCfg |                   tf配置的库                    |
|                                                              |                                                 |
| from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg |              创建可视化标记的工具               |
|                                                              |                                                 |
|                                                              |                                                 |
|                                                              |                                                 |
|                                                              |                                                 |
|                                                              |                                                 |
|                                                              |                                                 |
|                                                              |                                                 |
|                                                              |                                                 |
|                                                              |                                                 |
|                                                              |                                                 |



|        API        |                作用                |
| :---------------: | :--------------------------------: |
| configclass修饰器 | 使得类的对象可以通过键值的形式访问 |
|                   |                                    |
|                   |                                    |
|                   |                                    |
|                   |                                    |
|                   |                                    |
|                   |                                    |

## 6 问题汇总

> 使用过程中遇到的问题的解决方法

+ play时正常进入，但是没有机器人，点开prim显示丢失

​		由于网络问题无法加载到服务器上的网格文件，使得机器人的usd文件参考缺失。此时需要打开终端中报错的那个instanceable_meshes文件的地址，下载到本地。

+ 从本地加载机器人

​		如果直接在IsaacSim的资产列表中下载Robot，在IsaacLab中使用时会出现缺少相关关节的报错，这是由于IsaacLab和IsaacSim使用的虽然是同一个Robot但是其对应的usd设置可能不同，此时还是从浏览器上下载对应的Robot及Instanceable并存放在同一个文件夹下面比较好。

+ 查看任务的源文件地址

​		在开始仿真时，终端会显示调用的任务地址，具体如下：

![image-20250513204039760](/media/airporal/8333B1863791CF8A/learn/Notebook/IsaacLab.assets/image-20250513204039760.png)
