

## 说明
需要最新的CMotorCommand模块



## 注意

**MJCF模型文件**
- USD文件所有的motor关节使用正向进行训练
- MJCF模型文件电机方向设置为与实际机器人相同
- 


控制器
- 

### 仿真运行
```shell
# 编译
colcon build --packages-select mjsim
# 软链接编译
colcon build --symlink-install --packages-select mjsim

# 运行sim_node
ros2 run mjsim sim_node --ros-args -p model:=nezhav1v1
# 0.2X倍速运行仿真
ros2 run mjsim sim_node --ros-args -p model:=nezhav1v1 -p sim_rate:=0.2

# 运行控制器（直接从python启动）
ros2 run mjsim nezhav1_controller2
```
**可能想修改**
- 相机跟踪距离：sim_node, 搜索`enable_camera_tracking`，修改跟踪主体与相机距离
- 

### policy测试

```shell
# 数据测试(data.json)
python3 src/mjsim/test/test_policy.py --frames 60 --frame 5
# 默认obs，切换action计算(前10帧参考意义不大)
python3 src/mjsim/test/test_policy.py --frames 60 --test action
```

### 工具
```shell
# 模型基础信息
python3 src/mjsim/test/test_model.py nezhav1

# 可视化
ros2 run mjsim model_viewer --ros-args -p model:=nezhav1
# Web可视化（viser，WebGL，可以在手机上查看）
ros2 run mjsim viser_model_viewer --ros-args -p model:=nezhav1 -p port:=8080

# 计算器
tools.ipynb
# 键盘Joy指令发布器
ros2 run mjsim kb_joy

# Web状态监测器
ros2 run mjsim web_monitor
```

**数据记录与分析**

```shell
# 记录数据
ros2 run mjsim log_node --ros-args -p model_type:=nezhav1 -p log_rate:=100
# 数据分析
cd /home/shiro/wBipedController/logs/nezhav1
python3 plot_logs.py 

```






