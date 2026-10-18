// 文件名称: imu_driver.h
// 功能描述: 定义 ImuDriver 类及相关数据结构，用于处理和解析 IMU 数据
// 作者: TobbyChen (modified)
// 创建日期: 2025-04-09

#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <cstdint>   // 标准整型头文件
#include <iostream>
#include <string>
#include <vector>
#include <chrono>
#include <cstring>
#include <map>
#include <memory>
#include <mutex>
#include <thread>
#include <deque>
#include <numeric>
#include <cmath>
#include <atomic>

#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/case_conv.hpp>

#include "analysis_data.h"  // 定义 yis_cmd_response_t 等结构体

#define ENBALE_DEBUG_OUTPUT 1
#define ENABLE_SERIAL_INPUT 1

#define DATA_BUF_SIZE 8192

namespace imu {

// IMU Data protocal
const uint8_t MODE_HEADER1            = 0;
const uint8_t MODE_HEADER2            = 1;
const uint8_t MODE_TID_L              = 2;
const uint8_t MODE_TID_H              = 3;
const uint8_t MODE_LENGTH             = 4;
const uint8_t MODE_MESSAGE            = 5;
const uint8_t MODE_CHECKSUM_L         = 6;
const uint8_t MODE_CHECKSUM_H         = 7;

const uint8_t MODE_GPS_RAW            = 10;

// IMU Param protocal
const uint8_t    PORDUCTION_INFO   = 0x00;
const uint8_t    BAUDRATE          = 0x02;
const uint8_t    OUTPUT_FREEQUENCY = 0x03;
const uint8_t    OUTPUT_CONTENT    = 0x04;
const uint8_t    STANDARD_PARAM    = 0x05;
const uint8_t    MODE_SETTING      = 0x4D;

typedef enum _Id {
    QUERY_STATUS  = 0x00,
    CONFIG_MEMERY = 0x01,
    CONFIG_FLASH  = 0x02
} ID;

#pragma pack(1)
typedef struct {
    uint8_t type;
    uint8_t len;
} yis_tlv_header_t;
#pragma pack()

// 定义一个结构体，类似 sensor_msgs::msg::Imu
struct ImuData {
    std::chrono::steady_clock::time_point timestamp;
    std::string frame_id;
    struct {
        double x;
        double y;
        double z;
    } angular_velocity;
    struct {
        double x;
        double y;
        double z;
    } linear_acceleration;
    struct {
        double x;
        double y;
        double z;
        double w;
    } orientation;
};


/* 
 * 类名: ImuDriver
 * 功能  : 实现 IMU 数据的接收、解析和内部存储，
 *         并提供线程安全的接口来获取最新数据。
 */
class ImuDriver {
    public:
        // 构造函数
        ImuDriver(const std::string &port = "/dev/imu", int baudrate = 460800);
        // 析构函数
        ~ImuDriver();
    
        /*
         * 函数名   : getImuData
         * 功能     : 返回最新的 IMU 数据（线程安全）。
         * 参数     : 无
         * 返回值   : ImuData 结构体
         */
        ImuData getImuData();
    
    private:
        boost::asio::io_service io_service_;
        boost::asio::serial_port serial_port_;
        std::string port_;
        int         baudrate_;
        size_t      buffer_size_;
        bool        configured_;
        std::atomic<size_t> bytes_;
        bool        first_frame_;
        uint32_t    prev_tid_;
        double      prev_yaw_;
        double      prev_angle_x_;
        double      prev_angle_y_;
        double      prev_angle_z_;
        double      prev_accel_x_;
        double      prev_accel_y_;
        double      prev_accel_z_;
        double      initial_yaw_;
        bool        first_yaw_;

        yis_cmd_response_t response;      // 响应数据
        protocol_info_t yesense_out;
    
        // 数据缓冲区
        std::shared_ptr<std::vector<char>> data_buffer_ptr_;
        uint8_t message_in_[DATA_BUF_SIZE];
        char async_read_buffer_[1024];
    
        // 互斥锁：m_mutex_ 用于保护串口接收缓冲区，
        //          imu_mutex_ 用于保护 imu_data_
        std::mutex m_mutex_;
        std::mutex imu_mutex_;
    
        // 保存上次串口接收数据的时间
        std::chrono::steady_clock::time_point last_serial_time_;
    
        // 异步读取及数据处理线程
        std::thread read_thread_;
        std::thread process_thread_;
        std::atomic<bool> running_;
    
        // 异步读取函数
        void startAsyncRead();
    
        /*
         * 函数名   : handleRead
         * 功能     : 异步读取回调函数，处理串口数据
         * 参数     : error - 错误码
         *            bytes_transferred - 实际读取字节数
         * 返回值   : void
         */
        void handleRead(const boost::system::error_code &error,
                        std::size_t bytes_transferred);
    
        /*
         * 函数名   : processBuffer
         * 功能     : 从缓冲区中提取数据并调用数据解析逻辑
         * 参数     : void
         * 返回值   : void
         */
        void processBuffer();
    
        /*
         * 函数名   : updateImuData
         * 功能     : 解析后的数据更新到 imu_data_ 中
         * 参数     : imu_data - 解析后的 protocol_info_t 数据
         * 返回值   : void
         */
        void updateImuData(const protocol_info_t &imu_data);
        /*
         * 函数名   : lowPassFilter
         * 功能     : 低通滤波器，实现简单滤波
         * 参数     : new_value - 新输入值
         *            prev_value- 前一滤波值
         *            alpha     - 低通滤波系数（默认 0.05）
         * 返回值   : double 滤波结果
         */
        double lowPassFilter(double new_value, double prev_value,
                             double alpha = 0.05);
    
        /*
         * 函数名   : normalizeYaw
         * 功能     : yaw 角归一化，确保角度连续
         * 参数     : yaw - 原始 yaw 角
         * 返回值   : double 归一化后的 yaw 角
         */
        double normalizeYaw(double yaw);
    
        /*
         * 函数名   : toQuaternion
         * 功能     : 将 roll、pitch、yaw（单位：度）转换为四元数
         * 参数     : roll  - 横滚角
         *            pitch - 俯仰角
         *            yaw   - 偏航角
         * 返回值   : Quaternion 四元数结构
         */
        struct Quaternion {
            double x;
            double y;
            double z;
            double w;
        };
        Quaternion toQuaternion(double roll, double pitch, double yaw);
    
        // 针对角速度的滑动平均滤波器：为每个轴维护一个缓冲区
        std::deque<double> angular_velocity_buffer_x_;
        std::deque<double> angular_velocity_buffer_y_;
        std::deque<double> angular_velocity_buffer_z_;
        size_t moving_average_window_size_;
        double movingAverageFilter(std::deque<double> &buffer, double new_value,
                                   size_t window_size);
    
        /*
         * 函数名   : processAngularVelocity
         * 功能     : 对角速度先低通滤波后滑动平均滤波
         * 参数     : new_value  - 新角速度值
         *            prev_value - 前一滤波值
         *            first_flag - 首次标识
         *            bias       - 角速度偏置
         *            alpha      - 低通滤波系数
         *            buffer     - 滑动平均滤波缓冲区
         * 返回值   : double 处理后的角速度值
         */
        double processAngularVelocity(double new_value, double &prev_value,
                                      bool &first_flag, double &bias, double alpha,
                                      std::deque<double> &buffer);
    
        // 用于角速度滤波的偏置和首次标志
        double bias_angle_x_;
        double bias_angle_y_;
        double bias_angle_z_;
        bool   first_angle_x_;
        bool   first_angle_y_;
        bool   first_angle_z_;
    
        // 内部保存的最新 IMU 数据
        ImuData imu_data_;

        // Shiro: Extra
        bool err_flag = false;
        long err_cnt = 0;
        boost::system::error_code last_err;
        void handle_async_read_error(const boost::system::error_code &error);
        // boost::asio::io_context& io_context_;
        // boost::asio::steady_timer timer_;  // 声明计时器
        // void scheduleRetry(int milliseconds);
    };

}  // namespace imu

#endif  // IMU_DRIVER_H
