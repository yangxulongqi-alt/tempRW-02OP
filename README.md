# RW-02OP 双足轮足机器人

<p align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-blue.svg" alt="ROS2 Humble">
  <img src="https://img.shields.io/badge/C%2B%2B-17-blue.svg" alt="C++17">
  <img src="https://img.shields.io/badge/MuJoCo-Simulation-orange.svg" alt="MuJoCo">
  <img src="https://img.shields.io/badge/License-MIT-green.svg" alt="MIT License">
</p>

RW-02OP 是一款双足轮足复合移动机器人,集成 MuJoCo 仿真、LQR 最优控制与实时 EtherCAT 电机驱动,支持站立、坡道上下、台阶跨越、单边行进等多种运动模式。

## 功能演示

| 视频 | 说明 |
|------|------|
| [站立、上坡与下台阶](RW-02OP_OpenWBipedController/1.%20站立、上坡与下台阶.mp4) | 原地站立与坡道、台阶场景 |
| [单边与斜坡](RW-02OP_OpenWBipedController/2.%20单边与斜坡.mp4) | 单侧支撑与斜坡适应 |
| [水平快速运动](RW-02OP_OpenWBipedController/3.%20水平快速运动.mp4) | 平地高速移动 |

> **注意:** 视频文件较大,建议使用 Git LFS 管理(见 [存储库优化](#存储库优化))。

## 项目结构

```
RW-02OP/
├── RW-02OP_OpenWBipedController/     # 控制器源码 (ROS2 工作空间)
│   ├── src/
│   │   ├── encos_eth_master/         # EtherCAT 电机驱动 (因克斯电机)
│   │   ├── mjsim/                    # MuJoCo 仿真节点
│   │   ├── nezha_lqr_controller/     # LQR 控制器 + NN 策略
│   │   └── yesense_imu/              # YSF650 IMU 驱动
│   ├── assets/                       # MuJoCo 模型与 Mesh
│   └── Readme.md
├── 硬件/
│   └── RW-02OP轮足机器人硬件功能入门教程1.0.docx
├── 结构/
│   ├── RW-02OP.STEP                  # 整机 3D CAD 模型
│   └── RW-02OP_BOM清单_20260228.xlsx # 物料清单
├── .gitignore
└── README.md
```

## 快速开始

### 环境要求

- **操作系统:** Ubuntu 22.04 LTS
- **ROS2 版本:** Humble Hawksbill
- **C++ 标准:** C++17
- **依赖:** MuJoCo, ethercat-master (SOEM), onnxruntime

### 编译

```bash
# 克隆仓库并初始化子模块(如有)
git clone https://github.com/yangxulongqi-alt/tempRW-02OP.git
cd tempRW-02OP/RW-02OP_OpenWBipedController

# 安装依赖
sudo apt install ros-humble-* libeigen3-dev libcpp_common-dev

# 编译
colcon build --packages-select encos_eth_master mjsim nezha_lqr_controller yesense_imu
source install/setup.bash
```

### 运行

```bash
# 终端 1: 启动 MuJoCo 仿真
ros2 run mjsim sim_node --ros-args -p model:=nezhav1

# 终端 2: 启动 LQR 控制器
ros2 run nezha_lqr_controller wblqr_controller

# 终端 3: 启动手柄节点
ros2 run joy joy_node
```

## 模块说明

### encos_eth_master

基于 SOEM 的 EtherCAT 主站驱动,用于控制因克斯(Encos)无刷关节电机。

**配置项** (见 `config/motor_config.json`):

| 参数 | 说明 |
|------|------|
| `motor_mappings` | 电机 ID、从站索引、E-CAN 通道映射 |
| `motor_current_limits` | 各电机电流上限 |
| `motor_angle_limits` | 各电机角度范围 (rad) |
| `motor_heat_tables` | 电机热量-电流曲线表 |
| `accumulated_heat_map` | 初始累计热量值 |
| `kt_table` | 电机转矩常数表 |

### nezha_lqr_controller

Nezha 双足机器人 LQR 控制器,结合 Model Predictive Control 与神经网络速度策略。

- `config/lqr_paramsCD.json` — LQR 权重参数
- `config/nn_models/` — ONNX 策略网络 (平地/粗糙地形)
- `config/wb_motor_config.json` — 整车电机配置

### mjsim

MuJoCo 仿真节点,加载 `assets/nezha/models/nezhav1/` 下的 MJCF 模型与 STL 网格,提供高保真物理仿真环境。

### yesense_imu

YSF650 惯性测量单元驱动,提供六轴姿态数据,用于平衡控制闭环。

## 硬件参考

- **硬件入门教程:** [硬件/RW-02OP轮足机器人硬件功能入门教程1.0.docx](硬件/RW-02OP轮足机器人硬件功能入门教程1.0.docx)
- **3D 模型:** [结构/RW-02OP.STEP](结构/RW-02OP.STEP)
- **BOM 清单:** [结构/RW-02OP_BOM清单_20260228.xlsx](结构/RW-02OP_BOM清单_20260228.xlsx)

## 许可证

本项目的控制器软件代码采用 [MIT License](RW-02OP_OpenWBipedController/license.md)。
硬件文档及结构文件版权归作者所有。

## 存储库优化建议

当前仓库包含 4 个大文件(最大 83 MB),GitHub 推荐单文件不超过 50 MB。如需优化可安装 Git LFS:

```bash
git lfs install
git lfs track "*.mp4" "*.STEP"
git add .gitattributes
git commit -m "Enable Git LFS for large binary files"
git push
```

## 相关链接

- [MuJoCo 官方文档](https://mujoco.readthedocs.io/)
- [SOEM (Simple OpenEtherCAT Master)](https://github.com/OpenEtherCATsociety/SOEM)
- [onnxruntime](https://onnxruntime.ai/)
