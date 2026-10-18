#ifndef ETHERCAT_CONTROLLER_H
#define ETHERCAT_CONTROLLER_H

#include <vector>
#include <mutex>
#include <algorithm>
#include <cstdint>
#include <optional>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <map>
#include <deque>
#include <thread>
#include <atomic>
#include <iostream>
#include <cstddef>
#include <limits>
#include <memory>
#include <stdexcept>
#include "time.h"
#include <deque> 
#include <chrono>  // 引入计时库
#include <iomanip> // 格式化输出
#include <math.h>
#include <queue>
#include <condition_variable>
extern "C" {
#include "ethercat.h"
#include "motor_control.h"
}
#include "transmit.h"
#include "shared_types.h"
#include "shared_types_cpp.h"
#include <nlohmann/json.hpp>
#include <filesystem> 
#include <unistd.h>

struct MotorMapping {
    uint16_t motor_id;
    int slave_index; // EtherCAT slave index
    int passage;     // EtherCAT passage
};
// 电流-时间数据表
struct MotorHeatData {
    double current;
    double time;
};

struct CommandLog {
    uint64_t timestamp; 
    int motor_id;
    double position;
    double speed;
    double torque;
    double kp;
    double kd;
};

struct FeedbackLog {
    uint64_t timestamp; 
    int motor_id;
    double position;
    double speed;
    double torque;
    uint8_t error_code_;
};

extern std::vector<MotorMapping> motor_mappings;
extern std::map<int, double> motor_current_limits;
extern std::map<int, std::pair<double, double>> motor_angle_limits;
extern std::map<int, std::vector<MotorHeatData>> motor_heat_tables;
extern std::map<int, double> accumulated_heat_map;
extern MotorBack motor_back_data[30];

class EtherCATController
{
public:
    // 启动 EtherCAT
    explicit EtherCATController(const std::string& config_file = "../config/motor_config.json",std::optional<uint16_t> init_motor_id = std::nullopt);

    // 停止 EtherCAT
    ~EtherCATController();

    // 控制电机并执行 EtherCAT 任务
    void control_and_ethercat_task(const std::vector<ControllerCommand>& controller_commands);

    /// 读接口：获取当前所有电机反馈数据（线程安全）
    std::vector<MotorBack> get_all_motor_back();

    /// 写接口：由维护线程调用，更新所有电机反馈数据（线程安全）
    void set_all_motor_back(const std::vector<MotorBack>& new_back);

    // 初始化默认控制命令
    void initialize_default_commands(std::optional<uint16_t> filter_id);

    static constexpr double heat_threshold = 0.95;

private:
    // 配置相关
    std::vector<MotorMapping> motor_mappings;
    std::map<int, double> motor_current_limits;
    std::map<int, std::pair<double, double>> motor_angle_limits;
    std::map<int, std::vector<MotorHeatData>> motor_heat_tables;
    std::map<int, double> accumulated_heat_map;
    std::vector<MotorCommand> init_commands;
    std::map<uint16_t, double> kt_table_;
    std::atomic<uint64_t> loop_counter_{0};

    // 控制与线程
    bool isConfig[SLAVE_NUMBER]{false};
    std::thread ethercat_thread_;
    bool stop_ethercat_thread_ = false;
    std::mutex mapping_mutex;
    std::optional<std::vector<MotorCommand>> last_command_;

    // 状态估计与滤波
    std::map<int, std::deque<double>> position_window_map;
    std::map<int, double> last_position_map;
    std::chrono::steady_clock::time_point last_time = std::chrono::steady_clock::now();
    std::unordered_map<int, std::pair<double, double>> motor_estimates;
    std::mutex cmd_mutex_;                         // protect last_command_
    size_t window_size_ = 500;

    // 卡尔曼滤波参数
    const double Q = 0.001;
    const double R = 0.025;
    const double A = 1.0;
    const double H = 1.0;
    const double B = 0.0;

    // 配置文件加载
    void load_motor_config(const std::string& filename);

    // 线程任务
    void ethercat_task();

    
    void calculate_motor_heat(int motor_id, double current);
    void check_motor_angle(int motor_id, double position);
    double low_pass_filter_or_window(int motor_id, double current_position);
    double get_kt(uint16_t motor_id);
    double calculate_speed(int motor_id, double current_position, double last_position, double elapsed_time);
    void kalman_filter(int motor_id, double z_position_diff, double z_speed_feedback, double elapsed_time);

    std::queue<CommandLog> command_log_queue_;
    std::queue<FeedbackLog> feedback_log_queue_;
    std::mutex log_mutex_;
    std::condition_variable log_cv_;
    std::atomic<bool> stop_log_thread_{false};
    std::thread log_thread_;

    void log_task();

    // 新增维护反馈线程
    std::thread maintain_thread_;
    std::atomic<bool> stop_maintain_thread_{false};

    std::array<MotorBack, 30> motor_back_data_;         // 缓存区
    std::mutex motor_back_mutex_;                       // 保护缓存区的互斥量

    /// 老的 update_motor_data 改名为内部实现
    void update_motor_data_impl(std::vector<MotorBack>& out_data);

    // 维护线程入口，不变
    void maintain_motor_back_task();

        // 安全模式标志，一旦有错误则置位
    std::atomic<bool> safety_mode_{false};
};


#endif // ETHERCAT_CONTROLLER_H
