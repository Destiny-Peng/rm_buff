# <div align="center">rm_buff — 能量机关识别与角度预测🎯</div>


> **注：由于 RM2025 能量机关在装甲模块等方面有改动，本仓库中涉及装甲模块识别的部分可能需要适配或修改。**

### <div align="center">1. 简介📓</div>

参考：[一种RM2024能量机关的识别与角度拟合方法](https://bbs.robomaster.com/article/371982)。

本项目借鉴了该文在能量机关检测与角度拟合上的方法，并在工程化与可扩展性方面进行了改进。主要内容如下：

- 借鉴的方法：采用文中提出的目标识别与角度拟合思路作为算法基础（检测 → 装甲处理 → 角度拟合）；
- 工程化适配：将算法组织为 ROS2 包与节点，便于在 ROS2（Humble）环境中部署与调试；
- 模块化与可扩展性：按功能划分检测、装甲处理、角度拟合、弹道补偿等模块，便于替换或升级单独模块；
- 预留接口：在配置与接口设计上为后续性能优化与算法替换留出实现空间。

系统输入为时间戳、图像以及相机姿态（roll/pitch/yaw），输出为拟合后用于云台的 pitch 与 yaw 值。项目主要包含识别与坐标解算模块，不含车载串口通信或相机驱动（这部分可按实际平台接入）。
可视化配置 `rune.rviz` 已包含在仓库根目录，可在 `rviz2` 中直接加载查看运行结果。

### <div align="center">2. 构建与运行（ROS2 Humble）🔧</div>

本仓库以 ROS2 Humble 为目标环境，使用 `colcon` 构建。

构建示例：

```bash
# 在已安装 ROS2 Humble 的系统中，并已 source ROS 环境
cd <your_ros2_workspace>
# 将本仓库放入工作区（或以子模块形式加入）后构建
colcon build --symlink-install
source install/setup.bash
```

运行示例（在已 source 的终端中按顺序启动）：

```bash
ros2 run usart virtual_usart
ros2 launch buff virtual_rune_launch.py
ros2 launch buff rune_launch.py
```

随后打开 `rviz2` 并加载仓库根目录下的 `rune.rviz` 配置以查看可视化信息。

### <div align="center">3. 参考与致谢</div>

- 本仓库实现参考并借鉴了原始论文与开源实现，感谢原作者的公开贡献。
- 参考文献：一种RM2024能量机关的识别与角度拟合方法（原文链接保留）。

### <div align="center">4. 许可与贡献</div>

当前 README 保留了原实现的致谢与参考说明；如需在本仓库中指定许可证（例如 MIT/Apache），请在仓库根目录添加相应的 `LICENSE` 文件，我会同步更新 README 中的许可证说明。
