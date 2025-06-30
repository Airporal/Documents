# 💧HoloOcean For AUV Motion planning and Reinforcement Learning Simulation 
---
## 1. 📖开发备忘录

> ​	由于Holoocean仿真环境自身开发复杂（基于UE5的引擎开发），本项目并不打算建模新的水下机器人进而实现全Sim2Real流程。**本项目旨在借助提供的Hovering AUV搭建Sim端系统框架与算法验证**。


+ 开发环境
  
| 項目          | 版本                          |
| ------------- | ----------------------------- |
| 操作系統      | Ubuntu20.04                   |
| 显卡          | NVIDIA GeForce RTX 4080 super |
| Nvidia Driver | 550.144.03                    |
| Python        | 3.10.16(ros2使用系统的3.8.10) |
| PyTorch       | 2.6.0                         |
| CUDA          | 12.4                          |
| ROS2          | foxy                          |
| holoocean     | 2.0.0                         |

---

+ HoloOcean环境

​	使用的Agent：Hovering AUV(Holoocean中，HoveringAUV与BlueROV2拥有相同逻辑，因此也可使用BlueRov2)

​	搭载的传感器：坐标系满足右手法则，且一律x向前、y向左，z向上。

|      传感器       | 坐标系 |      数据格式      |                           备注                           |
| :---------------: | :----: | :----------------: | :------------------------------------------------------: |
|    PoseSensor     | n系下表示 |    4x4位姿矩阵     | 传感器安装的位置$R_{nb}(\Theta_{nb})$和$[x^n_{b,n},y^n_{b,n},z^n_{b,n}]$ |
|  VelocitySensor   | n系下表示 | $[v_x,v_y,v_z]^T$  |                       传感器安装的位置的速度m/s                       |
|     IMUSensor     | b西 | 4x3矩阵 |            加速度单位：m/s，角速度单位 rad/s             |
|    DepthSensor    | n系下表示 | number |                          深度m                           |
| RangeFinderSensor | b系下表示 | n维向量 |                         激光雷达m                         |
|     DVLSensor     | n系下表示 | 7维向量 | m/s |
|   ImagingSonar    |        |                    |                                                          |
| SingleBeam Sonar  | b系下表示 | n纬向量 | 测距用声纳m |
|     RGBCamera     | 相机坐标系 | 256x256x256x4的ndarray |                    左右各一个RGBA相机                    |
|  RotationSensor   | b系相对于n系，n系下表示 |  $\Theta_{nb}=[\phi,\theta,\psi]^T$  |传感器安装位置的欧拉角，角度单位|
|                   |        |                    |                                                          |

> 预配值的场景在env中定义。可参考Holoocean[文档](https://byu-holoocean.github.io/holoocean-docs/v2.0.0/holoocean/sensors.html#holoocean.sensors.DVLSensor)配置自己的场景。

  + holoocean 键盘与手柄的快捷键映射

| 键盘  |   手柄    |     备注     |
| :---: | :-------: | :----------: |
|   q   |    0/A    |   视角下降   |
|   v   |    1/B    | 切换观众模式 |
|   c   |    3/X    | 切换相机模式 |
|   e   |    4/A    |   视角上升   |
| shift |   6/LB    | 快速移动视角 |
|  esc  |   7/RB    |     退出     |
|   h   | 10/return |   切换Hub    |
|  tab  |   11/=    |   切换设备   |
|   w   |    Up     |   视角前移   |
|   s   |   Down    |   视角后移   |
|   a   |   Left    |   视角左移   |
|   d   |   Right   |   视角右移   |

---

+ ROS2
> 本项目Holoocean与ROS2独立运行不同的python环境。

​		Holoocean中单独完成仿真，但是为了使用ROS2中提供的规划、控制功能包，以及后续的接入Dave仿真器，有必要建立ROS2Bridge。

​		本项目使用Socket在Holoocean客户端与Ros2服务端之间通信。通信节点负责在二者之间转发数据。Ros并不是必需的，所有需要从客户端转发到服务端的数据都通过ros_bridge下client.py文件中定义的Ros2SocketClient类实现，同时在Ros服务端使用python和cpp分别实现（单独运行一个）的SocketServerNode类接收并转发为ROS2通信类型。

​		ROS2中使用到的相关的数据接口定义在holoocean_interface功能包中。


> [!IMPORTANT]
  >
  > 命名规定
  >
  > 本项目大多数情况下使用以下命名规则：
  >
  > + 类名/函数名/脚本名：全部小写，单词之间用下划线连接，如holoocean_env。
  > + 文件名/变量名：小写字母开头，连接的下一个单词首字母大写，如robotState。

---

## 2. 快速开始

+ 基本测试

​	在test文件夹下有项目相关的测试脚本，其中基础功能集成在environment.py中，运行以下命名快速开始：

```python
python ./test/environment.py --scenario SimpleUnderwater-Hovering --agent auv0 --controller 0 --showInTerminator 0 --joystick_index 0 --path_auto 0
```

​	其中controller提供三种控制模式，使用手柄控制时，需要先查看系统手柄编号（默认为0），运行以下命令查看：

```bash
sudo lsusb
```

​	使用自动跟踪路径时，支持三种轨迹，使用path_auto参数指定。

+ Ros2测试

	首先需要编译ros2_ws：
	
```bash
cd ros2_ws
colcon build
source install/setup.bash
ros2 run holoocean_bridge_py socket_server_py
```

​	然后就可以运行客户端以建立连接：

```bash
# 根目录下
python ./test/lqr_test.py --ros2 True --mode 1 --mini_error 0.8
```

​		之后建立会自动建立连接，可在另一终端查看话题转发：

```bash
ros2 topic echo /action_topic
```

## 3. 代办

- [ ] 🔥MPC PID ARQC控制器
- [ ] ⁉前端感知障碍物
- [ ] ⭕局部路径优化

## 4. 备注

| 变化方向   | P 增大           | I 增大             | D 增大         |
| ---------- | ---------------- | ------------------ | -------------- |
| 响应速度   | 提高             | 提高（缓慢）       | 减慢           |
| 稳态误差   | 存在（无法消除） | 减少               | 无影响         |
| 震荡风险   | 增加             | 增加（因积累过多） | 减少           |
| 抗干扰能力 | 较差             | 易积累扰动         | 抗短期扰动强   |
| 实时性     | 强               | 慢                 | 依赖前后状态差 |
