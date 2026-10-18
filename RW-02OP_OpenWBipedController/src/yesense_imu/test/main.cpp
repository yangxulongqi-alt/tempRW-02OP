// 文件名称: main.cpp
#include "imu_driver.h"   // 包含 IMU 数据处理模块头文件
#include <iostream>
#include <thread>
#include <chrono>

using namespace imu;

int main() {
    // 初始化 ImuDriver，使用默认串口 "/dev/imu" 和波特率 921600
    ImuDriver imu_driver("/dev/imu", 921600);
    
    while (true) {
        // 调用获取最新 IMU 数据的函数
        ImuData data = imu_driver.getImuData();
        
        // 打印 IMU 数据
        std::cout << "Frame ID: " << data.frame_id << std::endl;
        std::cout << "Angular Velocity: x=" << data.angular_velocity.x 
                  << ", y=" << data.angular_velocity.y 
                  << ", z=" << data.angular_velocity.z << std::endl;
        std::cout << "Linear Acceleration: x=" << data.linear_acceleration.x 
                  << ", y=" << data.linear_acceleration.y 
                  << ", z=" << data.linear_acceleration.z << std::endl;
        std::cout << "Orientation: x=" << data.orientation.x 
                  << ", y=" << data.orientation.y 
                  << ", z=" << data.orientation.z 
                  << ", w=" << data.orientation.w << std::endl;
        std::cout << "--------------------------------------------------" << std::endl;
        
        // 休眠 500 毫秒后再读取下一次数据，避免占用过高 CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    
    return 0;
}
