#include "ethercat_controller.h"
#include <iostream>
#include <vector>
#include <thread>
#include <chrono>
#include <cmath>
#include <fstream>
#include <ament_index_cpp/get_package_share_directory.hpp>


void test1(int motor_id);
void test2(int motor_id);
int get_input_args(int argc, char* argv[]);

std::string package_share_dir = ament_index_cpp::get_package_share_directory("encos_eth_master");
std::string config_file_path = package_share_dir + "/config/motor_config.json";
// std::string config_file_path = "../../share/encos_eth_master/config/motor_config.json";

// MARK: Main
// 使用：`ros2 run encos_eth_master motor_test_basic`
int main(int argc, char* argv[]) {
    int motor_id = get_input_args(argc, argv);
    test2(motor_id);
    return 0;
}


int get_input_args(int argc, char* argv[]) {
    int motor_id = 19;
    if (argc < 2) {
        std::cerr << "请传入id参数" << std::endl;
        return 1; // 返回非零值表示程序异常退出
    }
    try {
        // 将参数转换为int类型
        motor_id = std::stoi(argv[1]);
        std::cout << "\x1b[32m > motor_id = \x1b[0m" << motor_id << std::endl;
    } catch (const std::invalid_argument& e) {
        std::cerr << "错误: 传入的参数不是有效的整数" << std::endl;
        return 1;
    } catch (const std::out_of_range& e) {
        std::cerr << "错误: 传入的整数超出范围" << std::endl;
        return 1;
    }
    return motor_id;
}


// 电机位置控制模式（固定位置）
void test1(int motor_id) {
    EtherCATController ethercat_controller(config_file_path);
    // const int motor_id = 20;
    const int mode = 0; // 控制模式 0：位置模式 1：速度模式 2：力矩模式 3：力位混合模式
    const double amplitude = 0.0;//0.2;  // 正弦波幅值（单位：弧度）
    const double freq = 0.2;       // 频率 Hz
    const double duration = 100.0; // 测试时长（秒）
    const double dt = 0.02;        // 控制周期（秒）
    int steps = static_cast<int>(duration / dt);
    std::cout << "\x1b[32m ---------- Start Loop ---------- \x1b[0m" << std::endl;
    for (int i = 0; i < steps; ++i) {
        double t = i * dt;
        double pos_cmd = amplitude * std::sin(2 * M_PI * freq * t);
        double speed_cmd = amplitude * 2 * M_PI * freq * std::cos(2 * M_PI * freq * t);

        // 构造控制命令
        ControllerCommand cmd;
        cmd.motor_id = motor_id;
        cmd.control_mode = mode;    // 0-位置模式，1-速度模式，2-扭矩模式
        cmd.position = 0.87; //pos_cmd;
        cmd.speed = 0.1; // speed_cmd;
        cmd.torque = 10.0;
        cmd.kp = 0.0;
        cmd.kd = 0.0;

        std::vector<ControllerCommand> cmds = {cmd};
        ethercat_controller.control_and_ethercat_task(cmds);

        //获取反馈
        std::vector<MotorBack> motor_back_data;
        motor_back_data = ethercat_controller.get_all_motor_back();
        // std::cout << "\x1b[32m motor pos =  \x1b[0m" << motor_back_data[19].position << std::endl;
        // std::cout << "\x1b[31m motor data len =  \x1b[0m" << motor_back_data.size() << std::endl;
        if(motor_back_data.size()==1){
            std::cout << "\x1b[32m motor pos =  \x1b[0m" << motor_back_data[0].position << ", tq = " << motor_back_data[0].torque << std::endl;
        }else {
            // for(int i=0; i<motor_back_data.size(); i++){
            //     std::cout << i << "\x1b[31m motor_id =  \x1b[0m" << motor_back_data[i].motor_id <<std::endl;
            // }
            std::cout << "\x1b[31m motor data len =  \x1b[0m" << motor_back_data.size() << std::endl;
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000)));
    }
}


// 单电机，MIT模式测试
void test2(int motor_id) {
    EtherCATController ethercat_controller(config_file_path);
    const int mode = 3; // 控制模式 0：位置模式 1：速度模式 2：力矩模式 3：力位混合模式
    const double amplitude = 0.2;//0.2;  // 正弦波幅值（单位：弧度）
    const double freq = 0.2;       // 频率 Hz
    const double duration = 100.0; // 测试时长（秒）
    const double dt = 0.02;        // 控制周期（秒）
    int steps = static_cast<int>(duration / dt);
    std::cout << "\x1b[32m ---------- Start Loop ---------- \x1b[0m" << std::endl;
    for (int i = 0; i < steps; ++i) {
        double t = i * dt;
        double pos_cmd = amplitude * std::sin(2 * M_PI * freq * t);
        double speed_cmd = amplitude * 2 * M_PI * freq * std::cos(2 * M_PI * freq * t);

        // 构造控制命令
        ControllerCommand cmd;
        cmd.motor_id = motor_id;
        cmd.control_mode = mode;    // 0-位置模式，1-速度模式，2-扭矩模式
        cmd.position = 0.0;
        cmd.speed = 2;
        cmd.torque = 0.0;
        cmd.kp = 0.0;
        cmd.kd = 0.8;

        std::vector<ControllerCommand> cmds = {cmd};
        ethercat_controller.control_and_ethercat_task(cmds);

        //获取反馈
        std::vector<MotorBack> motor_back_data;
        motor_back_data = ethercat_controller.get_all_motor_back();
        for(int i=0; i<motor_back_data.size(); i++){
            if(motor_back_data[i].motor_id==19) {
                std::cout << "\x1b[32m speed =  \x1b[0m" << motor_back_data[i].speed << ", tq = " << motor_back_data[i].torque << ", pos= " << motor_back_data[i].position << std::endl;    
            }
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(int(dt * 1000)));
    }
}