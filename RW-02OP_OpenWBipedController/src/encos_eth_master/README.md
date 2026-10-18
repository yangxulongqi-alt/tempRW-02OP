# 因克斯电机使用说明
### 电机参数配置

在电机运行时，会读取config文件夹下电机的参数运用在程序之中，有如下参数需要配置：

1. motor_mappings

    `"motor_id": 11, "slave_index": 0, "passage": 1`

	分别是电机的ID，电机所属的从站以及电机在E-CAN板上所属的通道

2. motor_current_limits

    `"11": 180,`

 	分别是电机的ID以及电机的电流限制

3. motor_angle_limits

    ` "11": [-3.14, 3.14]`

    分别是电机的ID以及电机对应的角度限制
4. `"motor_heat_tables":`

    ```bash
    [55.0, 222.0],
    [75.0, 63.0]
    ```

    分别是电机的ID以及电机本身的热量表格
5. `"accumulated_heat_map"`

    ` "11": 0.0,`

    分别是电机的id,积累的热量初始值
6. `"kt_table"`

    `{"motor_id": 11, "kt": 2.1},`

    分别是电机的id，以及电机对应的kt值
7. ` "init_commands"`

    ```bash
      {
          "motor_id": 11,
          "slave_index": 0,
          "passage": 1,
          "control_mode": 0,
          "position": 0.0,
          "speed": 10,
          "torque": 0,
          "kp": 0,
          "kd": 0
        },
    ```

       一般是使用位置模式，postion单位为度,speed的为rpm*10，torque为电流的阈值

8. `"motor_params"`

    ```bash
     {
        "motor_id": 11,
        "max_current": 220.0,
        "min_current": -220.0,
        "max_torque": 400.0,
        "min_torque": -400.0
       },
    ```

	电机本身的属性，电流的阈值以及力矩的阈值

### 驱动使用说明

1. 构建类

    `EtherCATController ethercat_controller;`

    电机便会运动到初始位姿
2. 构建运动指令

    ```cpp
    // 构造控制命令
    typedef struct ControllerCommand {
        uint16_t motor_id;
        uint8_t control_mode;   // 控制模式 0：位置模式 1：速度模式 2：力矩模式 3：力位混合模式
        float position;         // 位置值    单位：rad
        float speed;            // 速度值    单位：rad/s
        float torque;           // 力矩值    单位：N
        float kp;               // 参数KP
        float kd;               // 参数KD
    } ControllerCommand;

            ControllerCommand cmd;
            cmd.motor_id = motor_id;
            cmd.control_mode = mode;
            cmd.position = pos_cmd;
            cmd.speed = speed_cmd;
            cmd.torque = torque.cmd;

            std::vector<ControllerCommand> cmds = {cmd};
            ethercat_controller.control_and_ethercat_task(cmds);

    ```

3. 获得反馈

```cpp
typedef struct MotorBack {
    uint16_t motor_id;
    double position;    // 单位：rad
    double speed;       // 单位：rad/s
    float torque;       // 单位：N
    uint8_t error_code; // 电机没有错误时会上报0
} MotorBack;
std::vector<MotorBack> motor_back_data;
motor_back_data = ethercat_controller.get_all_motor_back();
```

4. 反馈分析

    输入的命令和反馈的log文件会存在log文件夹中


## ✅使用示例：对指定电机绘图

```bash
python3 analyze_motor_log.py \
  --cmd_csv ../log/command_log_2025-06-10_15-30-21.csv \
  --fbk_csv ../log/feedback_log_2025-06-10_15-30-21.csv \
  --motor_id 5
```

### ✅ 示例：对所有电机绘图

```bash
python3 analyze_motor_log.py \
  --cmd_csv ../log/command_log_2025-06-10_15-30-21.csv \
  --fbk_csv ../log/feedback_log_2025-06-10_15-30-21.csv
```

---

### 💾 示例：保存所有图像到指定目录

```bash
python3 analyze_motor_log.py \
  --cmd_csv ../log/command_log_2025-07-25_13-30-42.csv \
  --fbk_csv ../log/feedback_log_2025-07-25_13-30-42.csv \
  --save_dir output_figs/
```

这将在 `output_figs/` 下生成：

```
motor_1_position.png
motor_1_speed.png
motor_1_torque.png
...
```