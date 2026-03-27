**<div align="center">HERO 战队 — 哈尔滨工业大学（威海）视觉组</div>**

# <div align="center">rm_buff — 能量机关三维运动学建模与预测系统🎯</div>

> **简介**：本项目是 Robomaster 比赛中能量机关（Rune）模块后端的运动学解算与通讯部分开源仓库。项目中剥离了前端二维视觉硬件及目标识别代码（如装甲板识别等），主要展示和分享包括多点三维圆拟合的非线性优化（基于 Ceres）、考虑空气阻力的弹道求解（基于 RK4），以及 ROS2 下的消息通信结构。

## 1. 项目情况

参考思路：[一种RM2024能量机关的识别与角度拟合方法](https://bbs.robomaster.com/article/371982)

本项目借鉴了该文章中的解算思路，并在 ROS2 Humble 环境下进行了工程化适配。主要内容包括：

- **三维建模与预测**：从视觉端获得2D/3D带噪观测数据后，通过 **RANSAC滤波** 剔除野值，进而使用 Ceres `ConstrainedCircle3DFittingCost` 进行**非线性约束拟合**，实现在空间中计算能量机关真实的三维旋转圆盘轨迹。
- **弹道补偿**：内建了基于 **四阶龙格-库塔 (RK4)** 的弹道补偿算法。考量了重力模型与飞行过程中的空气阻力衰减。
- **模块化设计**：系统按职责划分为 `interfaces` (通信消息), `usart` (上下位机通讯桥梁) 以及 `buff` (运动学拟合控制) 三大模块，以便于各模块独立迭代或者在其他子系统中复用。

各个子模块的详情阅读，请查阅其相应的 README：
- 👉 [buff/README.md](./buff/README.md)（核心运动学解算与预测，弹道解析）
- 👉 [usart/README.md](./usart/README.md)（ROS2 硬件串口通讯与时间同步机制）

## 2. 模块拓扑图 (Architecture)

```text
+-------------------+      (FanArmors Msg)         +--------------------------------+
|                   |  ====[3D/2D Obsv]====>       |    buff / rune_modeling Node   |
|   Vision Node     |                              |--------------------------------|
|  (Not in Repo)    |                              | - RANSAC Outlier Filter        |
|                   |                              | - Ceres 3D Plane & Circle Fit  |
+-------------------+                              +-------------+------------------+
                                                                 |
                                                          (RuneFire Srv)
                                                                 v
+-------------------+      (Readpack Msg)          +--------------------------------+
|   usart Node      |  ====[Gimbal Pose/Odom]==>   | buff / rune_fire_controller    |
|-------------------|                              |--------------------------------|
| - MCU Serial Com  |                              | - Time-Delay Compensation      |
| - Time Sync Match |  <===[Pitch/Yaw Target]===   | - RK4 Ballistic Solver         |
+-------------------+      (Sendpack Msg)          +--------------------------------+
```

## 3. 构建与运行（基于 ROS2 Humble）🔧

在已安装 ROS2 Humble 与 `colcon` 的环境中部署：

```bash
# 1. 编译所有模块
cd <your_ros2_workspace>
colcon build --symlink-install

# 2. Source 环境
source install/setup.bash

# 3. 运行完整闭环（使用虚拟串口提供硬件在环模拟）：
# 终端 1: 启动底层虚拟通信
ros2 run usart virtual_serialport

# 终端 2: 启动底层能量机关预测及虚拟测试流
ros2 launch buff virtual_rune_launch.py
# 或若已接入真车硬件: ros2 launch buff rune_launch.py
```

可视化：直接打开 `rviz2`，加载仓库根目录下的 `rune.rviz` 查看各项解算输出和观测点。

## 4. 核心代码导读💡

如果对算法具体实现或工程结构细节感兴趣，可以留意以下文件：
1. **`buff/include/kinematic/optimization/ceres_costfunc.hpp`**
   - 定义了 Ceres 损失函数 CostFunction。利用 Eigen 构建了限制点在平面、并测算投影后空间距离的残差方程。
2. **`buff/src/utils/ballistic_solver.cpp`**
   - 常微分方程 RK4 数值求函数的实现文件。
3. **整体 ROS2 节点流** (`rune_modeling.cpp` 和 `rune_fire_controller.cpp`)
   - 展现了在 ROS2 环境中结合 TF 树 (`tf2_ros`) 更新时序，处理并发布云台姿态的控制流。

## 5. 许可与致谢

- 本系统的运动学预测部分思路借鉴了该文：[一种RM2024能量机关的识别与角度拟合方法](https://bbs.robomaster.com/article/371982)，在此向原作者的开源分享表示由衷的感谢。
- 本项目旨在与 Robomaster 社区内的开发者共同交流学习。
