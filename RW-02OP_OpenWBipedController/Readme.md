
### 说明

- [mjSim使用](./src/mjsim/Readme.md)
- [nezha_lqr_controller使用](./src/nezha_lqr_controller/Readme.md)

### run

```shell
# 运行仿真节点
ros2 run mjsim sim_node --ros-args -p model:=nezhav1

# 运行控制器
ros2 run nezha_lqr_controller wblqr_controller

# 手柄节点
ros2 run joy joy_node
```


