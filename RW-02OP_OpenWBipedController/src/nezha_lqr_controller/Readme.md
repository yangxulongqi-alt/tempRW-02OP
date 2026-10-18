

### 环境

- **ROS 2 Humble**: 
- **Eigen3**: 矩阵运算
- **Boost**: C++常用库
- **nlohmann-json**： json文件解析

### 控制参数计算
1. 计算脚本文件：`./calculator/modelCalc_v1.ipynb`
2. 将生成参数文件`lqr_paramsCD.json`复制到config目录下，并build以安装到install目录


### 编译与运行

```bash
# 单独编译
colcon build --packages-select nezha_lqr_controller
# 加载安装文件路径
source install/setup.bash
# 运行控制器
ros2 run nezha_lqr_controller wblqr_controller

# 手柄节点
ros2 run joy joy_node
```


