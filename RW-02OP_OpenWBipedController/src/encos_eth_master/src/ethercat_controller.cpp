#include "ethercat_controller.h"
#include <fstream>
#include <iostream>
#include <cmath>
#include <limits>
#include <sys/timerfd.h>

using json = nlohmann::json;

MotorBack motor_back_data[30];
EtherCATController::EtherCATController(const std::string& config_file,std::optional<uint16_t> init_motor_id)
    : last_command_(std::nullopt), 
      stop_ethercat_thread_(false),
      stop_log_thread_(false),
      stop_maintain_thread_(false),
      safety_mode_(false)
{
    load_motor_config(config_file);
    // 这里填自己电脑上的网卡
    // printf("初始化网口");
    EtherCAT_Init("enP8p1s0");

    if (ec_slavecount <= 0)
    {
        printf("未找到从站, 程序退出！");
    }
    else
        printf("从站数量： %d\r\n", ec_slavecount);
    
    initialize_default_commands(init_motor_id);
     // 启动 EtherCAT 指令发送线程
    ethercat_thread_ = std::thread(&EtherCATController::ethercat_task, this);

    log_thread_ = std::thread(&EtherCATController::log_task, this);

    maintain_thread_ = std::thread(&EtherCATController::maintain_motor_back_task,this);

}

EtherCATController::~EtherCATController()
{
    stop_ethercat_thread_ = true;
    if (ethercat_thread_.joinable()) {
        ethercat_thread_.join();
    }
    stop_log_thread_ = true;
    log_cv_.notify_all();
    if (log_thread_.joinable()) log_thread_.join();

    stop_maintain_thread_ = true;
    if (maintain_thread_.joinable()) maintain_thread_.join();
}


void EtherCATController::control_and_ethercat_task(const std::vector<ControllerCommand>& controller_commands)
{
    std::vector<MotorCommand> motor_commands;
    if (controller_commands.empty()) {
    std::cerr << "Error: No controller commands provided.\n";
    return;
    }

    // 遍历 controller_commands 中的每个电机控制命令
    for (const auto& controller_command : controller_commands) {
        int motor_id = controller_command.motor_id;
        bool found = false;
        MotorCommand cmd{};
		// 在 motor_mappings 中查找对应的 motor_id
        for (const auto& mapping : motor_mappings) {
            if (mapping.motor_id == motor_id) {
                cmd.slave_index = mapping.slave_index;
                cmd.passage = mapping.passage;
                found = true;
                break;
            }
        }

        if (!found) {
            printf("Motor ID %d not found in mappings\n", motor_id);
            continue;
        }

        // 设置控制参数
        cmd.motor_id = controller_command.motor_id;
        cmd.control_mode = controller_command.control_mode;
        cmd.position = controller_command.position;
        cmd.speed = controller_command.speed;
        cmd.torque = controller_command.torque;
        cmd.kp = controller_command.kp;
        cmd.kd = controller_command.kd;
        // 如果控制模式为0，位置模式，输出端期望位置数值解释：数值即为实际角度，单位为度，例：120.5 表示 120.5°。 输出端期望速度数值解释：0~32767 对应 0~3276.7rpm，比例为 10。
        if (cmd.control_mode == 0) {
            // 转换速度为 RPM
            double rpm = controller_command.speed * 10 * 60 / (2 * M_PI);  // 速度从 rad/s 转换为 RPM
            double rounded_rpm = std::round(rpm);  // 对转换后的 RPM 进行四舍五入

            // 将 RPM 转换为正值，确保其为绝对值
            rounded_rpm = std::abs(rounded_rpm);

            // 如果结果超过 uint16_t 最大值，则将其限制为最大值
            if (rounded_rpm > std::numeric_limits<uint16_t>::max()) {
                rounded_rpm = std::numeric_limits<uint16_t>::max();
            }

            // 强制转换为 uint16_t 并赋值给 cmd.speed
            cmd.speed = static_cast<uint16_t>(rounded_rpm);
            cmd.position = controller_command.position * 180 / (M_PI);
        }
        //如果控制模式为2.力矩模式，期望力矩：-32768~32767 对应-327.68~327.67Nm
        if (cmd.control_mode == 2) {
            cmd.torque = controller_command.torque * 100;
        }

        motor_commands.push_back(cmd);
    }

    // 安全地更新 last_command_
    {
        std::lock_guard<std::mutex> lk(cmd_mutex_);
        last_command_ = std::move(motor_commands);
    }

    // printf("Motor commands updated and EtherCAT task executed.\n");
}

void EtherCATController::ethercat_task() {
    using namespace std::chrono;
    auto next = steady_clock::now();
    const auto period = microseconds(1000);  // 1 ms

    while (!stop_ethercat_thread_) {
        next += period;
        // 在每次循环开始时自增 loop_counter_
        uint64_t ts = loop_counter_.fetch_add(1, std::memory_order_relaxed);

        if (safety_mode_) exit(EXIT_FAILURE);

        std::optional<std::vector<MotorCommand>> cmds_copy;
        {
            std::lock_guard<std::mutex> lk(cmd_mutex_);
            cmds_copy = last_command_;
        }
        if (cmds_copy) {
            Ethercat_Motor_Run(cmds_copy->data(), cmds_copy->size());

            std::lock_guard<std::mutex> lk(log_mutex_);
            for (const auto& cmd : *cmds_copy) {
                if (cmd.motor_id < 1 || cmd.motor_id > 30) continue;

                double log_pos, log_speed, log_torque;
                if (cmd.control_mode == 0) {
                    log_pos   = cmd.position / 180.0 * M_PI;
                    log_speed = cmd.speed   / 60.0      * (2.0 * M_PI);
                    log_torque = cmd.torque;
                } else if (cmd.control_mode == 2) {
                    log_pos    = cmd.position;
                    log_speed  = cmd.speed;
                    log_torque = cmd.torque / 100.0;
                } else {
                    log_pos    = cmd.position;
                    log_speed  = cmd.speed;
                    log_torque = cmd.torque;
                }

                // push 带 timestamp 的命令日志
                command_log_queue_.push(CommandLog{
                    ts,
                    cmd.motor_id,
                    log_pos, log_speed, log_torque,
                    cmd.kp, cmd.kd
                });

                // push 带 timestamp 的反馈日志
                auto& fb = rv_motor_msg[cmd.motor_id - 1];
                feedback_log_queue_.push(FeedbackLog{
                    ts,
                    cmd.motor_id,
                    fb.angle_actual_rad,
                    fb.speed_actual_rad,
                    fb.current_actual_float * get_kt(cmd.motor_id),
                    fb.error
                });
            }
        }

        log_cv_.notify_one();
        std::this_thread::sleep_until(next);
    }
}


// 根据 filter_id 从 init_commands 提取到 last_command_
void EtherCATController::initialize_default_commands(std::optional<uint16_t> filter_id) {
    std::vector<MotorCommand> result;
    if (filter_id.has_value()) {
        for (const auto& cmd : init_commands) {
            if (cmd.motor_id == filter_id.value()) {
                result.push_back(cmd);
                break; // 只取第一个匹配
            }
        }
    } else {
        result = init_commands;
    }
    // 将结果赋予 optional
    last_command_ = std::move(result);
}

    // 滑动窗口或低通滤波处理
double EtherCATController::low_pass_filter_or_window(int motor_id, double current_position) {
    // 滑动窗口的大小
    static size_t window_size = 500;

    // 获取当前电机的窗口
    std::deque<double>& window = position_window_map[motor_id];

    // 使用滑动窗口更新
    if (window.size() >= window_size) {
        window.pop_front();  // 窗口已满，移除最旧的值
    }
    window.push_back(current_position);  // 将当前值加入窗口

    // 计算窗口内的平均值
    double sum = 0.0;
    for (double pos : window) {
        sum += pos;
    }
    double average_position = sum / window.size();

    return average_position;
}
    
double EtherCATController::get_kt(uint16_t motor_id) {
    auto it = kt_table_.find(motor_id);
    if (it != kt_table_.end()) {
        return it->second;
    }
    fprintf(stderr, "Error: get kt failed, no parameters for motor_id=%u\n", motor_id);
    fflush(stderr);
    exit(EXIT_FAILURE);
}

void EtherCATController::calculate_motor_heat(int motor_id, double current) {
    auto it = motor_heat_tables.find(motor_id);
    if (it != motor_heat_tables.end()) {
        auto& accumulated_heat = accumulated_heat_map[motor_id]; // 获取或创建该电机的累积热量
        // 获取最小电流值
        double min_current = it->second.front().current;
        if (current >= min_current) {
            for (const auto& data : it->second) {
                if (current <= data.current) {
                    accumulated_heat += 1.0 / data.time * 0.01;
                    break;
                }
            }
        }
    }
}
    
void EtherCATController::check_motor_angle(int motor_id, double position) {
    auto it = motor_angle_limits.find(motor_id);
    if (it != motor_angle_limits.end()) {
        if (position < it->second.first || position > it->second.second) {
            printf( "电机 %d 角度超出限制，立即停止程序！", motor_id);
            safety_mode_ = true;;
        }
    }
}
void EtherCATController::kalman_filter(int motor_id, double z_position_diff, double z_speed_feedback, double elapsed_time) {
    // 获取当前电机的状态估计
    double& estimated_position = motor_estimates[motor_id].first;
    double& estimated_velocity = motor_estimates[motor_id].second;

    // 初始化卡尔曼滤波器状态
    static std::unordered_map<int, double> P_map; // 状态协方差矩阵，存储每个电机的协方差
    static std::unordered_map<int, double> last_velocity_map; // 存储上次计算的速度
    
    double& P = P_map[motor_id];
    
    if (P == 0) {
        P = 1.0; // 初始协方差
        estimated_position = 0.0; // 初始化位置
        estimated_velocity = 0.0; // 初始化速度
    }

    // 预测阶段
    // 预测位置和速度
    double predicted_position = estimated_position + estimated_velocity * elapsed_time;
    double predicted_velocity = estimated_velocity;

    // 更新协方差
    P += Q;

    // 更新阶段（卡尔曼滤波公式）
    double innovation_position = z_position_diff - predicted_position;
    double innovation_speed = z_speed_feedback - predicted_velocity;

    // 计算卡尔曼增益
    double K_position = P / (P + R);
    double K_speed = P / (P + R);

    // 更新状态估计
    estimated_position += K_position * innovation_position;
    estimated_velocity += K_speed * innovation_speed;

    // 更新协方差
    P = (1 - K_position) * P;
}
double EtherCATController::calculate_speed(int motor_id, double current_position, double last_position, double elapsed_time) {
    return (current_position - last_position) / elapsed_time;
}

void EtherCATController::load_motor_config(const std::string& filename) {
    std::ifstream in(filename);
    if (!in.is_open()) {
        throw std::runtime_error("Cannot open config file: " + filename);
    }
    json j;
    in >> j;

    motor_mappings.clear();
    for (const auto& item : j["motor_mappings"]) {
        motor_mappings.push_back({
            static_cast<uint16_t>(item["motor_id"]),
            item["slave_index"],
            item["passage"]
        });
    }

    motor_current_limits.clear();
    for (auto& [k, v] : j["motor_current_limits"].items()) {
        motor_current_limits[std::stoi(k)] = v;
    }

    motor_angle_limits.clear();
    for (auto& [k, v] : j["motor_angle_limits"].items()) {
        motor_angle_limits[std::stoi(k)] = {v[0], v[1]};
    }

    motor_heat_tables.clear();
    for (auto& [k, v] : j["motor_heat_tables"].items()) {
        std::vector<MotorHeatData> table;
        for (const auto& arr : v) {
            table.push_back({arr[0], arr[1]});
        }
        motor_heat_tables[std::stoi(k)] = table;
    }

    accumulated_heat_map.clear();
    for (auto& [k, v] : j["accumulated_heat_map"].items()) {
        accumulated_heat_map[std::stoi(k)] = v;
    }

    init_commands.clear();
    for (const auto& item : j["init_commands"]) {
        init_commands.push_back({
            static_cast<uint16_t>(item["motor_id"]),
            item["slave_index"],
            item["passage"],
            static_cast<uint8_t>(item["control_mode"]),
            static_cast<float>(item["position"]),
            static_cast<uint16_t>(item["speed"]),
            static_cast<int16_t>(item["torque"]),
            static_cast<float>(item["kp"]),
            static_cast<float>(item["kd"]),
        });
    }
    kt_table_.clear();
    for (const auto& item : j["kt_table"]) {
        uint16_t id = item["motor_id"];
        double kt = item["kt"];
        kt_table_[id] = kt;
    }

    for (const auto& item : j["motor_params"]) {
        MotorParams params;
        params.motor_id = item.at("motor_id").get<uint16_t>();
        params.max_current = item.at("max_current").get<float>();
        params.min_current = item.at("min_current").get<float>();
        params.max_torque = item.at("max_torque").get<float>();
        params.min_torque = item.at("min_torque").get<float>();
        set_motor_params(params.motor_id, params);
    }

}

void EtherCATController::log_task() {
    // —— 文件准备（同之前） —— 
    std::time_t now = std::time(nullptr);
    std::tm* lt = std::localtime(&now);
    std::ostringstream ts;
    ts << std::put_time(lt, "%Y-%m-%d_%H-%M-%S");
    auto log_dir = std::filesystem::current_path().parent_path() / "log";
    if (!std::filesystem::exists(log_dir))
        std::filesystem::create_directories(log_dir);

    std::ofstream cmd_log((log_dir / ("command_log_" + ts.str() + ".csv")).string());
    std::ofstream fbk_log((log_dir / ("feedback_log_"  + ts.str() + ".csv")).string());
    cmd_log << "timestamp,motor_id,position,speed,torque,kp,kd\n";
    fbk_log << "timestamp,motor_id,position,speed,torque,error_code_\n";

    // —— timerfd 驱动 1 ms 事件 —— 
    int tfd = timerfd_create(CLOCK_MONOTONIC, 0);
    itimerspec its{};
    its.it_value.tv_nsec = 1'000'000;
    its.it_interval = its.it_value;
    timerfd_settime(tfd, 0, &its, nullptr);

    uint64_t expirations = 0;
    while (!stop_log_thread_) {
        if (::read(tfd, &expirations, sizeof(expirations)) != sizeof(expirations))
            break;

        // 对于每个过期周期，都尝试把一条日志写出
        for (uint64_t i = 0; i < expirations; ++i) {
            bool wrote = false;
            {
                std::lock_guard<std::mutex> lk(log_mutex_);
                if (!command_log_queue_.empty()) {
                    auto lg = std::move(command_log_queue_.front());
                    command_log_queue_.pop();
                    cmd_log
                      << lg.timestamp   << ','
                      << lg.motor_id    << ','
                      << lg.position    << ','
                      << lg.speed       << ','
                      << lg.torque      << ','
                      << lg.kp          << ','
                      << lg.kd          << '\n';
                    wrote = true;
                }
            }
            if (wrote) continue;

            {
                std::lock_guard<std::mutex> lk(log_mutex_);
                if (!feedback_log_queue_.empty()) {
                    auto lg = std::move(feedback_log_queue_.front());
                    feedback_log_queue_.pop();
                    fbk_log
                      << lg.timestamp        << ','
                      << lg.motor_id         << ','
                      << lg.position         << ','
                      << lg.speed            << ','
                      << lg.torque           << ','
                      << static_cast<int>(lg.error_code_) << '\n';
                    wrote = true;
                }
            }
            // 若两队列空，则跳到下一个 expiration
        }
        cmd_log.flush();
        fbk_log.flush();
    }
    ::close(tfd);

    // —— 退出前写完所有剩余 —— 
    std::lock_guard<std::mutex> lk(log_mutex_);
    while (!command_log_queue_.empty()) {
        auto lg = std::move(command_log_queue_.front());
        command_log_queue_.pop();
        cmd_log
          << lg.timestamp   << ','
          << lg.motor_id    << ','
          << lg.position    << ','
          << lg.speed       << ','
          << lg.torque      << ','
          << lg.kp          << ','
          << lg.kd          << '\n';
    }
    while (!feedback_log_queue_.empty()) {
        auto lg = std::move(feedback_log_queue_.front());
        feedback_log_queue_.pop();
        fbk_log
          << lg.timestamp        << ','
          << lg.motor_id         << ','
          << lg.position         << ','
          << lg.speed            << ','
          << lg.torque           << ','
          << static_cast<int>(lg.error_code_) << '\n';
    }
}

void EtherCATController::set_all_motor_back(const std::vector<MotorBack>& new_back) {
    std::lock_guard<std::mutex> lk(motor_back_mutex_);
    // 清空旧数据
    for (auto& mb : motor_back_data_) {
        mb.motor_id = 0; // 或者其它无效值
    }
    // 拷贝新数据到数组
    for (const auto& mb : new_back) {
        if (mb.motor_id >= 1 && mb.motor_id <= 30) {
            motor_back_data_[mb.motor_id - 1] = mb;
        }
    }
}

// 读接口：给调用者返回一个 vector 拷贝
std::vector<MotorBack> EtherCATController::get_all_motor_back() {
    std::lock_guard<std::mutex> lk(motor_back_mutex_);
    std::vector<MotorBack> snapshot;
    snapshot.reserve(30);
    for (const auto& mb : motor_back_data_) {
        if (mb.motor_id != 0)  // 只加有效条目
            snapshot.push_back(mb);
    }
    return snapshot;
}

/// 原来的 update_motor_data 重命名为内部实现，供维护线程调用
void EtherCATController::update_motor_data_impl(std::vector<MotorBack>& out_data) {
    out_data.clear();
    out_data.reserve(30);
    for (int i = 0; i < 30; ++i) {
        if (rv_motor_msg[i].motor_id != 0) {
            MotorBack mb;
            mb.motor_id = rv_motor_msg[i].motor_id;
            mb.position = rv_motor_msg[i].angle_actual_rad;
            mb.error_code = rv_motor_msg[i].error;
            // // 计算速度
            // double speed = 0.0;
            // // 获取当前位置
            double current_position = rv_motor_msg[i].angle_actual_rad;
            // // 计算位置差分速度
            // std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();
            // std::chrono::duration<double> elapsed_time = current_time - last_time;
            // last_time = current_time;
            
            // double last_position = motor_estimates[rv_motor_msg[i].motor_id].first;
            // double position_diff_speed = calculate_speed(rv_motor_msg[i].motor_id, current_position, last_position, elapsed_time.count());

            // // 获取电机反馈的速度
            // double speed_feedback = rv_motor_msg[i].speed_actual_rad;

            // // 进行卡尔曼滤波，融合反馈的速度和位置差分速度
            // kalman_filter(rv_motor_msg[i].motor_id, position_diff_speed, speed_feedback, elapsed_time.count());

            // // 获取融合后的速度
            // speed = motor_estimates[rv_motor_msg[i].motor_id].second;
    
            mb.speed = rv_motor_msg[i].speed_actual_rad;
            double kt = get_kt(mb.motor_id);
            mb.torque = rv_motor_msg[i].current_actual_float * kt;

            out_data.push_back(mb);

            double current = rv_motor_msg[i].current_actual_float;
            if (motor_current_limits.find(rv_motor_msg[i].motor_id) != motor_current_limits.end() && current >= motor_current_limits[rv_motor_msg[i].motor_id]) {
                printf("%d 电流过高，立即关闭电机！", rv_motor_msg[i].motor_id);
                safety_mode_ = true;;
            }

            // 计算电机热量
            calculate_motor_heat(rv_motor_msg[i].motor_id, current);

            // 检查电机热量是否超过阈值
            if (accumulated_heat_map[rv_motor_msg[i].motor_id] > heat_threshold) {
                printf("警告：电机 %d 发热量过高，程序即将退出！", rv_motor_msg[i].motor_id);
                safety_mode_ = true;;
            }

            if (mb.error_code != 0) {
                std::cerr << "[错误] 电机ID: " << rv_motor_msg[i].motor_id 
                        << " 错误码: " << static_cast<int>(mb.error_code); 
                safety_mode_ = true;
            }

            check_motor_angle(rv_motor_msg[i].motor_id,current_position);
        }
    }
}

/// 维护线程：调用 impl 生成最新数据，然后通过 set_all 写入共享缓存
void EtherCATController::maintain_motor_back_task() {
    constexpr auto period = std::chrono::milliseconds(1);
    while (!stop_maintain_thread_) {
        std::vector<MotorBack> tmp;
        update_motor_data_impl(tmp);
        set_all_motor_back(tmp);
        std::this_thread::sleep_for(period);
    }
}