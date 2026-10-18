// 文件名称: imu_driver.cpp
// 功能描述: 实现 ImuDriver 类方法，用于处理和解析 IMU 数据
// 作者: John Doe
// 创建日期: 2025-03-31

#include "imu_driver.h"
#include <boost/asio.hpp>
#include <iostream>
#include <cstring>
#include <cmath>
#include <numeric>
#include <thread>
#include <chrono>

namespace imu {

/*
 * 函数名   : ImuDriver
 * 功能     : 构造函数，初始化串口及启动异步读写线程
 */
ImuDriver::ImuDriver(const std::string &port, int baudrate)
    : io_service_(),
      serial_port_(io_service_),
      port_(port),
      baudrate_(baudrate),
      buffer_size_(8192),
      configured_(false),
      bytes_(0),
      first_frame_(true),
      prev_tid_(0),
      prev_yaw_(0.0),
      prev_angle_x_(0.0),
      prev_angle_y_(0.0),
      prev_angle_z_(0.0),
      prev_accel_x_(0.0),
      prev_accel_y_(0.0),
      prev_accel_z_(0.0),
      initial_yaw_(0.0),
      first_yaw_(true),
      moving_average_window_size_(10),
      bias_angle_x_(0.0),
      bias_angle_y_(0.0),
      bias_angle_z_(0.0),
      first_angle_x_(true),
      first_angle_y_(true),
      first_angle_z_(true),
      running_(true)
{
    data_buffer_ptr_ = std::make_shared<std::vector<char>>();
    data_buffer_ptr_->reserve(buffer_size_);

    try {
        serial_port_.open(port_);
        serial_port_.set_option(
            boost::asio::serial_port_base::baud_rate(baudrate_));
        std::cout << "Serial port " << port_ 
                  << " opened successfully." << std::endl;
        configured_ = true;
    } catch (const std::exception &e) {
        std::cerr << "Error opening serial port: " << e.what() 
                  << std::endl;
    }

    memset(async_read_buffer_, 0, sizeof(async_read_buffer_));
    memset(message_in_, 0, DATA_BUF_SIZE);

    last_serial_time_ = std::chrono::steady_clock::now();

    // 启动异步读取
    startAsyncRead();

    // 启动 io_service 线程
    read_thread_ = std::thread([this]() {
        io_service_.run();
    });

    // 启动处理缓冲区线程（模拟定时器，每 2 毫秒调用一次 processBuffer）
    process_thread_ = std::thread([this]() {
        while (running_) {
            processBuffer();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    });
}

/*
 * 函数名   : ~ImuDriver
 * 功能     : 析构函数，停止线程并关闭串口
 */
ImuDriver::~ImuDriver()
{
    running_ = false;
    if (process_thread_.joinable()) {
        process_thread_.join();
    }
    io_service_.stop();
    if (read_thread_.joinable()) {
        read_thread_.join();
    }
    if (serial_port_.is_open()) {
        serial_port_.close();
    }
}

/*
 * 函数名   : startAsyncRead
 * 功能     : 启动异步读取数据
 */
void ImuDriver::startAsyncRead()
{
    serial_port_.async_read_some(
        boost::asio::buffer(async_read_buffer_,
            sizeof(async_read_buffer_)),
        std::bind(&ImuDriver::handleRead, this,
                  std::placeholders::_1, std::placeholders::_2));
}

/*
 * 函数名   : handleRead
 * 功能     : 异步读取回调，处理串口数据并重新启动异步读取
 */
void ImuDriver::handleRead(const boost::system::error_code &error,
                               std::size_t bytes_transferred)
{
    if (!error) {
        {
            std::lock_guard<std::mutex> lock(m_mutex_);
            last_serial_time_ = std::chrono::steady_clock::now();
            if (data_buffer_ptr_->size() + bytes_transferred <=
                data_buffer_ptr_->capacity())
            {
                data_buffer_ptr_->insert(data_buffer_ptr_->end(),
                    async_read_buffer_,
                    async_read_buffer_ + bytes_transferred);
            } else {
                std::cerr << "Buffer overflow prevented during async read!"
                          << std::endl;
            }
        }
        startAsyncRead();
    } else {
        // std::cerr << "Async read error: " << error.message() 
        //           << std::endl;
        handle_async_read_error(error);
        startAsyncRead();
    }
}

void ImuDriver::handle_async_read_error(const boost::system::error_code &error) {
    err_cnt ++;
    if(err_cnt%1000==0 || error.category()!=last_err.category()) {
        std::cerr << "Async read error: " << error.message() << ", ErrorCnt = " << err_cnt
                  << std::endl;
    }
    last_err = error;
    // std::this_thread::sleep_for(std::chrono::milliseconds(2));
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
}

// void ImuDriver::scheduleRetry(int milliseconds) {
//     // 使用deadline_timer延迟重试
//     timer_.expires_from_now(std::chrono::milliseconds(milliseconds));
//     timer_.async_wait([this](const boost::system::error_code& ec) {
//         if (!ec) {
//             startAsyncRead();
//         }
//     });
// }

/*
 * 函数名   : processBuffer
 * 功能     : 从缓冲区中提取数据并调用数据解析逻辑
 */
void ImuDriver::processBuffer()
{
    size_t available = 0;
    {
        std::lock_guard<std::mutex> lock(m_mutex_);
        available = data_buffer_ptr_->size();
        if (available > 0) {
            if (bytes_ + available > DATA_BUF_SIZE) {
                std::cerr << "message_in_ buffer overflow, resetting buffer!"
                          << std::endl;
                bytes_ = 0;
            } else {
                std::copy(data_buffer_ptr_->begin(),
                          data_buffer_ptr_->end(),
                          message_in_ + bytes_);
                bytes_ += available;
            }
            data_buffer_ptr_->clear();
        }
    }

    if (bytes_ >= PROTOCOL_MIN_LEN) {
        int cnt = bytes_;
        int pos = 0;
        while (cnt > 0) {
            if (cnt < PROTOCOL_MIN_LEN) {
                break;
            }
            protocol_info_t imu_out;
            int ret = analysis_data(message_in_ + pos, cnt,
                                   &imu_out, &response);
            if (analysis_done == ret) {
                pos++;
                cnt--;
            } else if (data_len_err == ret) {
                break;
            } else if (crc_err == ret || analysis_ok == ret) {
                output_data_header_t *header = reinterpret_cast<output_data_header_t *>(message_in_ + pos);
                unsigned int frame_len = header->len + PROTOCOL_MIN_LEN;
                if (response.response_recv_done) {
                    frame_len = response.len + PROTOCOL_MIN_LEN;
                }
                if (pos + frame_len > static_cast<int>(bytes_)) {
                    // std::cerr << "Incomplete frame detected, waiting for more "
                    //           << "data." << std::endl;
                    break;
                }
                if (analysis_ok == ret) {
                    if (!first_frame_) {
                        if (imu_out.tid != prev_tid_ + 1) {
                            std::cerr << "Frame drop detected: current tid = "
                                      << imu_out.tid
                                      << ", previous tid = " 
                                      << prev_tid_ << std::endl;
                        }
                    } else {
                        first_frame_ = false;
                    }
                    prev_tid_ = imu_out.tid;
                    // 更新内部 IMU 数据，不再发布 ROS 消息
                    updateImuData(imu_out);
                }
                pos += frame_len;
                cnt -= frame_len;
            } else {
                break;
            }
        }
        if (pos > 0 && cnt > 0) {
            if (static_cast<size_t>(pos) < DATA_BUF_SIZE &&
                static_cast<size_t>(pos) + static_cast<size_t>(cnt) <=
                DATA_BUF_SIZE)
            {
                memmove(message_in_, message_in_ + pos, cnt);
            } else {
                std::cerr << "Invalid buffer parameters: pos=" << pos 
                          << ", cnt=" << cnt 
                          << " exceed buffer limit of "
                          << DATA_BUF_SIZE << std::endl;
                cnt = 0;
            }
        }
        bytes_ = cnt;
    }
}

/*
 * 函数名   : updateImuData
 * 功能     : 将解析后的数据经过滤波处理后更新到内部 IMU 数据结构
 */
void ImuDriver::updateImuData(const protocol_info_t &imu_data)
{
    double alpha = 0.05;  // 低通滤波系数
    // double ang_vel_x = processAngularVelocity(imu_data.angle_x,
    //                   prev_angle_x_, first_angle_x_, bias_angle_x_,
    //                   alpha, angular_velocity_buffer_x_);
    // double ang_vel_y = processAngularVelocity(imu_data.angle_y,
    //                   prev_angle_y_, first_angle_y_, bias_angle_y_,
    //                   alpha, angular_velocity_buffer_y_);
    // double ang_vel_z = processAngularVelocity(imu_data.angle_z,
    //                   prev_angle_z_, first_angle_z_, bias_angle_z_,
    //                   alpha, angular_velocity_buffer_z_);
    
    // 单位转换：角度转弧度
    double ang_vel_x = imu_data.angle_x * M_PI / 180.0;
    double ang_vel_y = imu_data.angle_y * M_PI / 180.0;
    double ang_vel_z = imu_data.angle_z * M_PI / 180.0;

    double lin_acc_x = lowPassFilter(imu_data.accel_x, prev_accel_x_);
    double lin_acc_y = lowPassFilter(imu_data.accel_y, prev_accel_y_);
    double lin_acc_z = lowPassFilter(imu_data.accel_z, prev_accel_z_);

    double yaw = imu_data.yaw;
    if (first_yaw_) {
        initial_yaw_ = yaw;
        first_yaw_   = false;
    }
    yaw -= initial_yaw_;
    yaw  = normalizeYaw(yaw);
    // 将 roll, pitch, yaw 转换为四元数
    auto q = toQuaternion(imu_data.roll, imu_data.pitch, yaw);

    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        imu_data_.timestamp = last_serial_time_;
        imu_data_.frame_id  = "imu_link";
        imu_data_.angular_velocity.x = ang_vel_x;
        imu_data_.angular_velocity.y = ang_vel_y;
        imu_data_.angular_velocity.z = ang_vel_z;
        imu_data_.linear_acceleration.x = lin_acc_x;
        imu_data_.linear_acceleration.y = lin_acc_y;
        imu_data_.linear_acceleration.z = lin_acc_z;
        imu_data_.orientation.x = q.x;
        imu_data_.orientation.y = q.y;
        imu_data_.orientation.z = q.z;
        imu_data_.orientation.w = q.w;
    }

    // 更新加速度的滤波前值
    prev_accel_x_ = lin_acc_x;
    prev_accel_y_ = lin_acc_y;
    prev_accel_z_ = lin_acc_z;
}

/*
 * 函数名   : lowPassFilter
 * 功能     : 实现简单的低通滤波
 */
double ImuDriver::lowPassFilter(double new_value,
                                    double prev_value,
                                    double alpha)
{
    return alpha * new_value + (1.0 - alpha) * prev_value;
}

/*
 * 函数名   : normalizeYaw
 * 功能     : yaw 角归一化，确保角度连续
 */
double ImuDriver::normalizeYaw(double yaw)
{
    if (yaw - prev_yaw_ > 180) {
        yaw -= 360;
    } else if (prev_yaw_ - yaw > 180) {
        yaw += 360;
    }
    prev_yaw_ = yaw;
    return yaw;
}

/*
 * 函数名   : toQuaternion
 * 功能     : 将 roll、pitch、yaw（单位：度）转换为四元数
 */
ImuDriver::Quaternion ImuDriver::toQuaternion(double roll,
                                                      double pitch,
                                                      double yaw)
{
    double r = roll  * M_PI / 180.0;  // 转换为弧度
    double p = pitch * M_PI / 180.0;
    double y = yaw   * M_PI / 180.0;

    Quaternion q;
    double cy = cos(y * 0.5);
    double sy = sin(y * 0.5);
    double cp = cos(p * 0.5);
    double sp = sin(p * 0.5);
    double cr = cos(r * 0.5);
    double sr = sin(r * 0.5);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

/*
 * 函数名   : movingAverageFilter
 * 功能     : 实现滑动平均滤波器
 */
double ImuDriver::movingAverageFilter(std::deque<double> &buffer,
                                            double new_value,
                                            size_t window_size)
{
    buffer.push_back(new_value);
    if (buffer.size() > window_size) {
        buffer.pop_front();
    }
    double sum = std::accumulate(buffer.begin(),
                                 buffer.end(), 0.0);
    return sum / static_cast<double>(buffer.size());
}

/*
 * 函数名   : processAngularVelocity
 * 功能     : 对角速度先低通滤波后滑动平均滤波处理
 */
double ImuDriver::processAngularVelocity(double new_value,
                                               double &prev_value,
                                               bool &first_flag,
                                               double &bias,
                                               double alpha,
                                               std::deque<double> &buffer)
{
    if (first_flag) {
        bias = new_value;
        first_flag = false;
    }
    double corrected_value = new_value - bias;
    double filtered_value = lowPassFilter(corrected_value,
                                          prev_value, alpha);
    prev_value = filtered_value;
    return movingAverageFilter(buffer, filtered_value,
                               moving_average_window_size_);
}

/*
 * 函数名   : getImuData
 * 功能     : 返回当前最新的 IMU 数据（线程安全）
 */
ImuData ImuDriver::getImuData()
{
    std::lock_guard<std::mutex> lock(imu_mutex_);
    return imu_data_;
}

}  // namespace imu
