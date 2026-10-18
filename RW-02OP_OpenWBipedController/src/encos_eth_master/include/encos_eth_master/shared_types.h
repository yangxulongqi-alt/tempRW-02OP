#ifndef SHARED_TYPES_H
#define SHARED_TYPES_H

#include <stdint.h>   // 标准整数类型
#include <stdbool.h>  // bool 类型

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ControllerCommand {
    uint16_t motor_id;
    uint8_t control_mode;   // 控制模式
    float position;         // 位置值    单位：rad
    float speed;            // 速度值    单位：rad/s
    float torque;           // 力矩值    单位：N
    float kp;               // 参数KP
    float kd;               // 参数KD
} ControllerCommand;

typedef struct MotorCommand {
    uint16_t motor_id;
    int slave_index;        // EtherCAT slave index
    int passage;            // EtherCAT passage
    uint8_t control_mode;   // 控制模式
    float position;         // 位置值
    float speed ;            // 速度值
    float torque ;           // 力矩值
    float kp ;               // 参数KP
    float kd ;               // 参数KD
} MotorCommand;

typedef struct MotorBack {
    uint16_t motor_id;
    double position;
    double speed;
    float torque;
    uint8_t error_code;
} MotorBack;

typedef struct MotorParams {
    uint16_t motor_id;
    float max_current;
    float min_current;
    float max_torque;
    float min_torque;
    // 可扩展更多参数
} MotorParams;


// C接口函数声明
void Ethercat_Motor_Run(MotorCommand* commands, int num_commands);
void process_command(MotorCommand* motor_commands, int num_commands);
void EtherCATmotor_Data_Get(void);
int get_motor_params(uint16_t motor_id, MotorParams* out_params);

#ifdef __cplusplus
}
#endif
#endif // SHARED_TYPES_H