#include "nezha_lqr_controller/lqrController.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <unordered_map>

using json = nlohmann::json;


const std::unordered_map<WBStatus, std::string> statusToStringMap = {
    {WBStatus::WB_REST, "REST"},
    {WBStatus::WB_STANDING, "STANDING"},
    {WBStatus::WB_FALL, "FALL"},
    {WBStatus::WB_RUNNING, "RUNNING"},
    {WBStatus::WB_CALIBRATING, "CALIBRATING"}
};

std::string WBStatusToString(WBStatus status) {
    auto it = statusToStringMap.find(status);
    return it != statusToStringMap.end() ? it->second : "UNKNOWN_STATUS";
}

// MARK: Init
lqrController::lqrController() : Node("wblqr_controller"), motor_cmds(8)
{
    // 1. 控制器配置参数
    ini_parameters();

    // 2. 硬编码变量初始化
    Eigen::Matrix<double, 4, 1> kp_joints, kd_joints;
    kp_joints << 200,  20,  20,   0;     // for real, 又是LQR又是PD的
    kd_joints << 20,   10,   6,    0;
    quat.setW(1.0); quat.setX(0.0); quat.setY(0.0); quat.setZ(0.0);
    lqr_k_table_ = std::make_shared<lqrKTable>();
    vmc_left_  = std::make_unique<vmc::VMC>(lqr_k_table_, vmc::LEFT,  Kpd_joints_free);
    vmc_right_ = std::make_unique<vmc::VMC>(lqr_k_table_, vmc::RIGHT, Kpd_joints_free);
    leg_len_cmd = 0.5;
    vmc_left_->set_height(leg_len_cmd);         // 初始状态
    vmc_right_->set_height(leg_len_cmd);
    x_unified_cmd = 0.3;
    vx_unified_cmd = 0.0;

    // nn_init();
    
    

    // 3. 节点通信初始化
    if(platform==PLATFORM_REAL) {
        // 传感器与驱动器相关初始化
        real_setup_communication();
    } else {
        sim_setup_communication();
    }
    if(shouldLog){
        DataLog_all_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/DataLog_all", 10);
    }

    // 4. 控制循环线程setup
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / control_frequency),
        std::bind(&lqrController::controlLoop, this));
    
    // 5. 初始化状态报告
    LOG_INFO("Platform: " << param_platform);
    LOG_INFO("Motion controller initialized with " << control_frequency << " Hz frequency");
};

void lqrController::ini_parameters(){
    this->declare_parameter("control_frequency", 1000.0);         // 申明参数
    // this->declare_parameter("Platform", "real");     // 仿真平台
    this->declare_parameter("Platform", "mujoco");     // 仿真平台
    this->declare_parameter("kp_roll", 300.0);
    this->declare_parameter("kd_roll", 100.0);
    this->declare_parameter("kp_yaw", 20.0);
    this->declare_parameter("kd_yaw", 10.0);
    this->declare_parameter("kp_align", 100.0);
    this->declare_parameter("kd_align", 50.0);
    this->declare_parameter("shouldLog", false);
    param_platform = this->get_parameter("Platform").as_string();

    if(param_platform=="real") { platform = PLATFORM_REAL; } else {
        platform = PLATFORM_SIM;
    }
    control_frequency = this->get_parameter("control_frequency").as_double();
    kp_roll = this->get_parameter("kp_roll").as_double();
    kd_roll = this->get_parameter("kd_roll").as_double();
    kp_yaw = this->get_parameter("kp_yaw").as_double();
    kd_yaw = this->get_parameter("kd_yaw").as_double();
    kp_align = this->get_parameter("kp_align").as_double();
    kd_align = this->get_parameter("kd_align").as_double();
    shouldLog = this->get_parameter("shouldLog").as_bool();
}




/* ********************************************************************************
 *                      _______   _________   _______    _       
 *                     (  ____ \  \__   __/  (  ____ )  ( \      
 *                     | (    \/     ) (     | (    )|  | (      
 *                     | |           | |     | (____)|  | |      
 *                     | |           | |     |     __)  | |      
 *                     | |           | |     | (\ (     | |      
 *                     | (____/\     | |     | ) \ \__  | (____/\
 *                     (_______/     )_(     |/   \__/  (_______/
********************************************************************************** */ 
// MARK:  Control
void lqrController::controlLoop() {
    // if(loopCnt==0) {
    //     std::cout << "\x1b[32m ============== Loop Begin ============== \x1b[0m" << std::endl;
    // }
    loopCnt++;

    // 1. 状态采集与估计
    if(platform==PLATFORM_REAL) {  // 直接从读写线程读取（数据可能来自1ms前，也可能来自）
        real_get_eth_motor_data();
        real_get_IMU_data();
    } else {} // 仿真环境从callback中更新
    this->basic_state_transform();
    // autoCommand();

    // 2. 控制与输出
    switch(status) {
        case WB_REST:
            tau_all.setZero();
            break;
        case WB_STANDING:
            lqr_process_tranditional();
            planning_standing();
            // 限幅
            for(int i=0; i<8; i++) {
                if(tau_all(i)>tq_limit[i])  { tau_all(i) = tq_limit[i]; }         // 手动限制
                if(tau_all(i)<-tq_limit[i]) { tau_all(i) = -tq_limit[i]; }
            }
            if(t_platform - status_switch_time > 3.0) {
                x_unified_cmd = x_unified;
                status_trans(WB_RUNNING);
            }
            break;
        case WB_RUNNING:
            lqr_process_tranditional();
            if(vmc_left_->is_fall() || vmc_right_->is_fall()) {
                status_trans(WB_FALL);
                tau_all.setZero();
            }
            break;
        case WB_FALL:
            tau_all.setZero();
            break;
        case WB_CALIBRATING:
            real_calibrating_motor();
            break;
        default:
            tau_all.setZero();
    }
    

    if(platform==PLATFORM_REAL)  real_apply_torque();
    else {
        sim_apply_torque();
    }
    

    // 控制循环信息输出
    if(loopCnt%1000==0) {
        // LOG_INFO("LoopCnt = " << loopCnt << ", t_platform = " << t_platform <<", dt_tmp =" << dt_tmp << ", roll comp = " 
        //                 << ", motor err cnt: " << err_motor_msg_cnt);
        std::cout << C_G << "================== " << loopCnt << " Loops ==================" << C_End << std::endl;
        mj_q_ang = mj_q*180/mPI;
        std::cout << "mj_q_ang: " << mj_q_ang.transpose()<< std::endl;
        // 输出IMURPY
        std::cout << "RPY     : " << roll*180/mPI << ", " << pitch*180/mPI << ", " << yaw*180/mPI << std::endl;
    }
    if(shouldLog && status!=WB_REST && dt_tmp> 0.0002) {
        // dataLog_publish();
        // DataLog_all_contact_publish();
        // DataLog_all_RL_publish();
        // DataLog_all_stable_following_publish();
    }
    
    

};

// lqr基本计算流程(陈年古方)
void lqrController::lqr_process_tranditional() {
    // 1. 足端压力分配计算(只要是简化模型，无论如何估计都是有误差的)
    double yL = vmc_left_->get_wheel_pos_y();
    double yR = vmc_right_->get_wheel_pos_y();
    double m_on_hip = lqr_k_table_->m_head + lqr_k_table_->m_abad*2;
    double m_on_knee = m_on_hip + lqr_k_table_->m_thigh*2;
    double FnR = m_on_knee*9.81*yL/(yL-yR);   // 粗略估计足端期望的支撑力
    double FnL = m_on_knee*9.81*yR/(yR-yL);
    double FfL = 0.0;
    double FfR = 0.0;
    if(jump_stage==2 && platform==PLATFORM_SIM) {
        if(vmc_left_->leg_len>leg_len_cmd || vmc_right_->leg_len>leg_len_cmd) {
            // 开始认为离地
            jump_stage=3;
            LOG_INFO(C_G << "Jump Stage" << jump_stage << C_End);
            vmc_left_->set_kp_kd(Kpd_joints_normal);
            vmc_right_->set_kp_kd(Kpd_joints_normal);
            t_jump_LO = t_platform;        // 离地时间
        } else {
            FnL += 400;
            FnR += 400;
            FfL -= 210;
            FfR -= 210;
        }
        // 需要重置底层关节的PD
    }

    // 2. tau_ff, 足端力对roll的调控
    double tau_roll = PD_for_roll();
    double tau_align = PD_for_align();
    Eigen::Matrix<double, 3, 1> FnLVec, FnRVec;
    FnLVec << FfL+tau_align, 0, -FnL-tau_roll;
    FnRVec << FfR-tau_align, 0, -FnR+tau_roll;
    Eigen::Matrix<double, 1, 3> FjL = FnLVec.transpose()*vmc_left_->get_jacobian();
    Eigen::Matrix<double, 1, 3> FjR = FnRVec.transpose()*vmc_right_->get_jacobian();
    //         abad      hip    knee     wheel
    tau_ff <<  FjL(0),  FjL(1), FjL(2),   0.0,
               FjR(0),  FjR(1), FjR(2),   0.0;

    // 3. tau_lqr, 计算lqr输出并合并
    vmc_left_->lqr_calc( x_unified, x_unified_cmd, vx_unified_cmd);
    vmc_right_->lqr_calc(x_unified, x_unified_cmd, vx_unified_cmd);
    Eigen::Matrix<double, 3, 1> lqr_tauL = vmc_left_->get_lqr_tau();
    Eigen::Matrix<double, 3, 1> lqr_tauR = vmc_right_->get_lqr_tau();
    //         abad      hip           knee         wheel
    tau_lqr << 0.0,  -lqr_tauL(2), -lqr_tauL(1), -lqr_tauL(0),
               0.0,  -lqr_tauR(2), -lqr_tauR(1), -lqr_tauR(0);
    // if(tau_lqr(2) > 0) tau_lqr(2) = tau_lqr(2)*0.3;   // 限制LQR对膝关节的力(缩短)
    // if(tau_lqr(6) > 0) tau_lqr(6) = tau_lqr(6)*0.3;   // 这部分力有利于限制过冲

    // 离地扭矩设置   
    // if(!vmc_left_->get_contact_status())  {tau_ff(2) = 0; tau_lqr(3) = 0;}
    // if(!vmc_right_->get_contact_status()) {tau_ff(6) = 0; tau_lqr(7) = 0;}

    tau_all = tau_lqr + tau_ff;

    // 5. yaw, 基于车轮
    tau_wheel_yaw = PD_for_yaw();
    tau_all(3) = tau_all(3) - tau_wheel_yaw;
    tau_all(7) = tau_all(7) + tau_wheel_yaw;


    if(jump_stage==3) {
        // 跳跃阶段3: 离地过程
        tau_all.setZero();
        if(t_platform-t_jump_LO > 0.4) {
            jump_stage = 0;
            LOG_INFO(C_R << "开始着地检测" << C_End);
        }
        
    } else {
        // if(jump_stage!=2)
            deal_with_off_ground_normal();
    }
}

double lqrController::PD_for_roll() {
    // 调节左右腿的支撑力以实现roll调节，返回单腿的delta_tau
    adaptive_len_adjust2();
    static double last_droll = 0.0;
    const double alpha = 0.3;
    double droll_filtered = droll*alpha + last_droll*(1-alpha);
    last_droll = droll_filtered;
    // return kp_roll*(0.0 - roll) + kd_roll*(0.0 - droll);
    return kp_roll*(0.0 - roll) + kd_roll*(0.0 - droll_filtered);
}

double lqrController::PD_for_yaw() {
    // 偏航/转向控制
    static double last_dyaw = 0.0;
    const double alpha = 0.3;
    double dyaw_filtered = dyaw*alpha + last_dyaw*(1-alpha);
    last_dyaw = dyaw_filtered;

    static double t_worked = 0.0;         // 记录上次运行时间(处理积分项)
    static double err_int = 0.0;
    const double MAX_INT_TQ = 3.0;
    if(t_platform - t_worked > 0.03) err_int = 0.0;   // 如果运行时间间隔太长,认为积分项需要重置
    err_int += (t_platform - t_worked) * (yaw_cmd - yaw);
    tau_yaw_err_int = err_int * 10.0;      // KI = 7.0, 最大提供±5N
    if(tau_yaw_err_int > MAX_INT_TQ) tau_yaw_err_int = MAX_INT_TQ;          // 积分饱和
    if(tau_yaw_err_int < -MAX_INT_TQ) tau_yaw_err_int = -MAX_INT_TQ;
    t_worked = t_platform;

    // return kp_yaw* (yaw_cmd - yaw) + kd_yaw*(0.0 - dyaw);
    return kp_yaw* (yaw_cmd - yaw) + kd_yaw*(0.0 - dyaw_filtered) + tau_yaw_err_int;
}

double lqrController::PD_for_align() {
    // 腿分叉抑制(在进行旋转的时候，机身会发生扭转，分叉控制就是增加扭转刚度，而不是通过抑制车轮相对滑动实现)
    double alphaL = vmc_left_->get_task_alpha();
    double alphaR = vmc_right_->get_task_alpha();
    double dalphaL = vmc_left_->get_task_dalpha();
    double dalphaR = vmc_right_->get_task_dalpha();

    return kp_align*(alphaL- alphaR) + kd_align*(dalphaL - dalphaR);
}





/* ********************************************************************************
 *                       _______   _________   _______   _________   _______ 
 *                      (  ____ \  \__   __/  (  ___  )  \__   __/  (  ____ \
 *                      | (    \/     ) (     | (   ) |     ) (     | (    \/
 *                      | (_____      | |     | (___) |     | |     | (__    
 *                      (_____  )     | |     |  ___  |     | |     |  __)   
 *                            ) |     | |     | (   ) |     | |     | (      
 *                      /\____) |     | |     | )   ( |     | |     | (____/\
 *                      \_______)     )_(     |/     \|     )_(     (_______/
********************************************************************************** */ 
// 先这样吧
void lqrController::basic_state_transform() {
    dt_tmp = t_platform - tlast_platform;
    // 1. rpy及其导
    imu_data_trans();
    
    // 2. 更新两侧vmc状态
    vmc_left_->set_state( mj_q.head(4), mj_dq.head(4), pitch, dpitch, acc_n[2], mj_tau.head(4), dt_tmp);
    vmc_right_->set_state(mj_q.tail(4), mj_dq.tail(4), pitch, dpitch, acc_n[2], mj_tau.tail(4), dt_tmp);

    // 3. x_unified
    double vL = vmc_left_->get_wheel_vel_x();
    double vR = vmc_right_->get_wheel_vel_x();
    double yL = vmc_left_->get_wheel_pos_y();
    double yR = vmc_right_->get_wheel_pos_y();
    // double w_wheel = (vL - vR) / (yL-yR);    // 车轮差速估计的旋转角速度
    vx_unified = (vR*yL - vL*yR)/(yL - yR);
    if(dt_tmp < 0.0002 || dt_tmp>3.0) {
        // 仿真状态几乎没有更新, 什么都不需要做
    } else {
        x_unified += vx_unified * dt_tmp;
        tlast_platform = t_platform;
        // 4. 运动指令更新(TODO: 平滑、限幅、设计操纵特性)
        if(status==WB_RUNNING) {
            vx_unified_cmd = abs(vx_unified_cmd)>0.01 ? vx_unified_cmd : 0.0;
            dyaw_cmd = abs(dyaw_cmd_raw)>0.01 ? dyaw_cmd_raw : 0.0;
            // x_unified_cmd += vx_unified_cmd * dt_tmp;
            if(abs(vx_unified)>0.1 && abs(vx_unified_cmd)>0.1)
                x_unified_cmd = x_unified + vx_unified_cmd * dt_tmp*10;    // 需要
            else
                x_unified_cmd += vx_unified_cmd * dt_tmp;      // 低目标速度情况下, 直接累加
            yaw_cmd += dyaw_cmd * dt_tmp;
        } else if(status==WB_STANDING) {} else {}
    }
    
}

void lqrController::status_trans(WBStatus target_status) {
    std::string status_str_from = WBStatusToString(status);
    std::string status_str_to = WBStatusToString(target_status);
    LOG_INFO(C_G<<" 状态切换: " << status_str_from << "-->" << status_str_to << C_End);
    status = target_status;
    if(target_status==WB_REST || target_status==WB_CALIBRATING) {
        setup_motor_command_free();
    } else {
        setup_motor_command_MIT_low();
        vmc_left_->set_kp_kd(Kpd_joints_normal);
        vmc_right_->set_kp_kd(Kpd_joints_normal);
    }
    status_switch_time = t_platform;       // 状态切换时间
}



// MARK: Sim
void lqrController::sim_setup_communication() {
    // 接收与控制
    sim_js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, 
            std::bind(&lqrController::sim_js_callback, this, std::placeholders::_1));
    sim_imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10, 
            std::bind(&lqrController::sim_imu_callback, this, std::placeholders::_1));
    sim_jc_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "/mujoco_action", 10);
    joy_subs_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&lqrController::joy_callback, this, std::placeholders::_1));
    
}

void lqrController::sim_js_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // for (size_t i = 0; i < msg->name.size(); i++) {
    for (size_t i = 0; i < 8; i++) {
        mj_q(i) = msg->position[i] * mj_dir[i];
        mj_dq(i) = msg->velocity[i] * mj_dir[i];
        mj_tau(i) = msg->effort[i] * mj_dir[i];
    }
    // update_status_usd();
    t_platform = msg->header.stamp.sec + msg->header.stamp.nanosec/1e9;
}

void lqrController::sim_imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    quat.setW(msg->orientation.w);      // quat
    quat.setX(msg->orientation.x);
    quat.setY(msg->orientation.y);
    quat.setZ(msg->orientation.z);
    quat.normalize();
    gyro_imu(0) = msg->angular_velocity.x;    // gyro
    gyro_imu(1) = msg->angular_velocity.y;
    gyro_imu(2) = msg->angular_velocity.z;
    acc_imu[0] = msg->linear_acceleration.x;
    acc_imu[1] = msg->linear_acceleration.y;
    acc_imu[2] = msg->linear_acceleration.z;
}

void lqrController::sim_apply_torque() {
    // 4. tau_pd_eq, 维持平衡态的PD计算
    Eigen::Matrix<double, 4, 1> tau_pd_eqL = vmc_left_->calc_pd_for_eq();
    Eigen::Matrix<double, 4, 1> tau_pd_eqR = vmc_right_->calc_pd_for_eq();
    tau_eq_pd << tau_pd_eqL, tau_pd_eqR;

    auto message = std_msgs::msg::Float32MultiArray();
    message.data.resize(8);
    for (int i = 0; i < 8; ++i) {
        message.data[i] = (tau_all(i) + tau_eq_pd(i)) * mj_dir[i];
    }
    sim_jc_pub_->publish(message);
    
}


// MARK: Real
void lqrController::real_setup_communication() {
    for(int i=0; i<8; i++) {
        mj_q(i) = 0.0;
    }
    // EtherCAT
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("nezha_lqr_controller");
    std::string config_file_path = package_share_dir + "/config/wb_motor_config.json";
    eth_controller_ = std::make_unique<EtherCATController>(config_file_path);
    // IMU（创建解析线程与管理）
    yesense_imu_ = std::make_shared<imu::ImuDriver>("/dev/imu", 921600);
    // Joy手柄
    joy_subs_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "wb_joy_topic", 10, std::bind(&lqrController::joy_callback, this, std::placeholders::_1));
    
    // 初始化，后续不更改
    setup_motor_command_free();

    t_platform_start_ = this->now();
}




// 电机消息缓冲区(len=1)读取(通常需要在切换之后稳定)
void lqrController::real_get_eth_motor_data() {
    // 
    motor_back_data = eth_controller_->get_all_motor_back();
    if(motor_back_data.size() == 8) {
        // 正常size的电机数量
    } else {
        err_motor_msg_cnt ++;
        if(err_motor_msg_cnt%1000==0) {
            LOG_INFO("Error msg size: " << motor_back_data.size())
        }
    }
    // Read motor info
    for (size_t i = 0; i < motor_back_data.size(); i++) {
        for(int j=0; j<8; j++) {
            if(mjid_map[0][j]==motor_back_data[i].motor_id) {
                int jid = mjid_map[1][j];
                motor_pos_raw[jid] = motor_back_data[i].position * real_dir[jid];
                motor_vel_raw[jid] = motor_back_data[i].speed * real_dir[jid];
                motor_tq_raw[jid] = motor_back_data[i].torque * real_dir[jid];
                mj_q[jid] = motor_pos_raw[jid] + mjzero_offset[jid];
                mj_dq[jid] = motor_vel_raw[jid];
                mj_tau[jid] = motor_tq_raw[jid];
                break;
            }
        }
        // 如果没有在list中找到ID
        // LOG_INFO("illegal motor id: " << motor_back_data[i].motor_id);
    }

    t_platform = (this->now() - t_platform_start_).seconds();
}

void lqrController::real_apply_torque() {
    for(int i=0; i<8; i++) {
        motor_cmds[i].torque = tau_all(i) * real_dir[i];            // 电机端前馈扭矩设置
    }
    
    // 站立过程中限制膝关节扭矩输出变化率,
    lim_knee_tq_rate();

    eth_controller_->control_and_ethercat_task(motor_cmds);
}

void lqrController::real_get_IMU_data() {
    imu::ImuData data = yesense_imu_->getImuData();
    double quat_norm = data.orientation.w * data.orientation.w + 
                       data.orientation.x * data.orientation.x +
                       data.orientation.y * data.orientation.y +
                       data.orientation.z * data.orientation.z;
    if(quat_norm >1.2 || quat_norm <0.5) {
        // 错误的数据
        quat.setW(1);     // quat
        quat.setX(0);
        quat.setY(0);
        quat.setZ(0);
        // std::cout<< "\x1b[32m INF quat\x1b[0m" << std::endl;
    } else {
        quat.setW(data.orientation.w);     // quat
        quat.setX(data.orientation.x);
        quat.setY(data.orientation.y);
        quat.setZ(data.orientation.z);
    }
    quat.normalize();
    gyro_imu(0) = data.angular_velocity.x;   // gyro
    gyro_imu(1) = data.angular_velocity.y;
    gyro_imu(2) = data.angular_velocity.z;
    acc_imu[0] = data.linear_acceleration.x;
    acc_imu[1] = data.linear_acceleration.y;
    acc_imu[2] = data.linear_acceleration.z;
}

// MARK: Joy Logic
void lqrController::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if(joy_action_protect_flag) {
        if(t_platform - t_joy_action_start >t_protect)
            joy_action_protect_flag = false;
        return;
    }
    // 1. 基础状态切换
    switch(status){
        case WB_REST:
            // 切换到站立
            if( msg->buttons[ButtonLB] && (!msg->buttons[ButtonRB]) && msg->buttons[ButtonA]) {
                status_trans(WB_STANDING);
                reset_wheel_offset();          // 重置此时车轮的角度
                joy_action_protect();
            }
            // 长按进入标零状态
            if(msg->buttons[ButtonRB] && msg->buttons[ButtonLB] && msg->buttons[ButtonY]) {
                if(t_platform-long_press_t_button_cali > 3.0){
                    status_trans(WB_CALIBRATING);
                    joy_action_protect();
                    long_press_t_button_cali = t_platform;
                }
            } else long_press_t_button_cali = t_platform;     // 重置长按计时器
            break;
        case WB_CALIBRATING:
            // 记录一次标定数据
            if(msg->buttons[ButtonLB] && (!msg->buttons[ButtonRB]) && msg->buttons[ButtonA]) {
                real_record_motor_zero();
                joy_action_protect(1.0);
            }
            // 长按退出标零状态
            if(msg->buttons[ButtonRB] && msg->buttons[ButtonLB] && msg->buttons[ButtonY] && t_platform-long_press_t_button_cali > 3.0) {
                if(t_platform-long_press_t_button_cali > 3.0){
                    status_trans(WB_REST);
                    joy_action_protect();
                    long_press_t_button_cali = t_platform;
                }
            } else long_press_t_button_cali = t_platform;     // 重置长按计时器
            break;
        case WB_RUNNING:
            // 3. 运动指令，左摇杆：前后axes[1]，值[后-1, 前1], 左右axes[0], 值[左1，右-1]
            make_joy_cmd_desired(msg->axes[1], msg->axes[0]);
            // 3. 急停回到REST(表示X掉程序)
            if(msg->buttons[ButtonX])  status_trans(WB_REST);

            // 跳跃过程控制, 与Running同时运行
            switch(jump_stage) {
                case 0:
                    if(msg->buttons[ButtonLB] && msg->buttons[ButtonB]) {
                        jump_stage=1;
                        LOG_INFO(C_G << "Jump Stage: " << jump_stage << C_End << ", Leg Len cmd: " << leg_len_cmd);
                        leg_len_cmd_restore = leg_len_cmd;
                        leg_len_cmd = jump_prepare_height;
                        set_heights(leg_len_cmd, leg_len_cmd);
                        joy_action_protect(0.5);
                    }
                    break;
                case 1:
                    if(msg->buttons[ButtonLB] && msg->buttons[ButtonB]) {
                        jump_stage=2;
                        LOG_INFO(C_G << "Jump Stage: " << jump_stage << C_End << ", Leg Len cmd: " << leg_len_cmd);
                        leg_len_cmd = leg_len_cmd_restore;
                        set_heights(leg_len_cmd, leg_len_cmd);
                        joy_action_protect(0.5);
                        // 设置腿部刚度
                        vmc_left_->set_kp_kd(Kpd_joints_LO);
                        vmc_right_->set_kp_kd(Kpd_joints_LO);
                    }
                    break;
                case 2:
                    // 跳跃过程
                    break;
            }
            break;
        case WB_STANDING:
            // 3. 急停回到REST(表示X掉程序)
            if(msg->buttons[ButtonX])  status_trans(WB_REST);
            break;
        default:
            break;
    }
    
    // SharedPtr
    latest_joy_msg_ = *msg;
}


void lqrController::real_calibrating_motor() {
    // 1. 零位确定，动态记录关节电机范围, 车轮的在开机确定
    for(int i=0; i<8; i++) {
        if(motor_pos_raw[i] > motor_cali_max[i]){ motor_cali_max[i] = motor_pos_raw[i]; }
        if(motor_pos_raw[i] < motor_cali_min[i]){ motor_cali_min[i] = motor_pos_raw[i]; }
    }
}

void lqrController::real_record_motor_zero() {
    // 记录当前的零点，可以查看对应rviz显示
    for(int i=0; i<8; i++) {
        // 求解offset
        if(motor_cali_pos[i] > 0) {
            mjzero_offset[i] = motor_cali_pos[i] - motor_cali_max[i];
        } else {
            mjzero_offset[i] = motor_cali_pos[i] - motor_cali_min[i];
        }
    }
    // TODO：持久化到参数文件，现在先这样
    LOG_INFO(C_G<<" mjzero: {" << mjzero_offset[0] << ", " << mjzero_offset[1] << ", " << mjzero_offset[2] << ", " << mjzero_offset[3] << ", " << mjzero_offset[4] << ", " << mjzero_offset[5] << ", " << mjzero_offset[6] << ", " << mjzero_offset[7] << C_End)
}



/* ********************************************************************************
 *                      _          _______    _______ 
 *                     ( \        (  ___  )  (  ____ \
 *                     | (        | (   ) |  | (    \/
 *                     | |        | |   | |  | |      
 *                     | |        | |   | |  | | ____ 
 *                     | |        | |   | |  | | \_  )
 *                     | (____/\  | (___) |  | (___) |
 *                     (_______/  (_______)  (_______)
********************************************************************************** */ 
// 二阶段: 调试站立这一动态过程(数据处理在:rosbag_ana_contact2.ipynb)
void lqrController::DataLog_all_contact_publish() {
    // 对特定调试组统一发布状态
    auto msg_log_all = std_msgs::msg::Float32MultiArray();
    msg_log_all.data.resize(100);
    
    // 加载数据
    for(int i=0; i<8; i++){
        msg_log_all.data[i]    = mj_q(i);      // 0~8,   mj_q
        msg_log_all.data[i+8]  = mj_dq(i);     // 8~16,  mj_dq
        msg_log_all.data[i+16] = mj_tau(i);    // 16~24, mj_tau, 回传的电机扭矩
        msg_log_all.data[i+24] = motor_cmds[i].torque * real_dir[i];   // 24~32, 指令扭矩
        msg_log_all.data[i+32] = vmc_left_->get_lqr_err()(i);     // LQR误差, left
        msg_log_all.data[i+40] = vmc_right_->get_lqr_err()(i);     // LQR误差, right
        msg_log_all.data[i+52] = tau_ff(i);   // 前馈扭矩记录
    }
    msg_log_all.data[48] = vmc_left_->get_Fn_est();         // 左右腿的足端估计力, 决定tau_FF存在
    msg_log_all.data[49] = vmc_right_->get_Fn_est();
    msg_log_all.data[50] = pitch;
    msg_log_all.data[51] = dpitch;
    msg_log_all.data[60] = plan_h;    // 规划的高度
    msg_log_all.data[61] = vmc_left_->leg_len;
    msg_log_all.data[62] = vmc_right_->leg_len;
    msg_log_all.data[63] = x_unified;           // 位置, 估计
    msg_log_all.data[64] = vx_unified;          // 速度, 估计
    msg_log_all.data[65] = x_unified_cmd;
    msg_log_all.data[66] = vx_unified_cmd;      // 速度, 手柄输入值
    msg_log_all.data[67] = dyaw_cmd_raw;        // 旋转, 手柄输入值
    msg_log_all.data[68] = PD_for_roll();       // PD参数
    msg_log_all.data[69] = PD_for_align();
    msg_log_all.data[70] = PD_for_yaw();
    msg_log_all.data[71] = vmc_left_->get_h_target_now();  // 指令高度
    msg_log_all.data[72] = vmc_right_->get_h_target_now();
    msg_log_all.data[73] = roll;
    msg_log_all.data[74] = yaw;
    msg_log_all.data[75] = dyaw;
    msg_log_all.data[76] = tau_yaw_err_int;
    msg_log_all.data[77] = jump_stage;

    DataLog_all_pub_->publish(msg_log_all);

}

// 三阶段: 运动指令跟踪性能 & 劈叉稳定性 & 旋转稳定性 etc, 基本还是参数调试
void lqrController::DataLog_all_stable_following_publish() {
    auto msg_log_all = std_msgs::msg::Float32MultiArray();
    msg_log_all.data.resize(30);
    
    msg_log_all.data[0] = x_unified;           // 位置, 估计
    msg_log_all.data[1] = vx_unified;          // 速度, 估计
    msg_log_all.data[2] = x_unified_cmd;
    msg_log_all.data[3] = vx_unified_cmd;      // 速度, 手柄输入值
    msg_log_all.data[4] = dyaw_cmd_raw;        // 旋转, 手柄输入值
    msg_log_all.data[5] = mj_q(3);             // 左轮位置
    msg_log_all.data[6] = mj_q(7);             // 右轮位置
    msg_log_all.data[7] = mj_dq(3);            // 左轮速度
    msg_log_all.data[8] = mj_dq(7);            // 右轮速度
    msg_log_all.data[9] = PD_for_roll();       // PD参数
    msg_log_all.data[10] = PD_for_align();
    msg_log_all.data[11] = PD_for_yaw();
    msg_log_all.data[12] = vmc_left_->get_lqr_err()(0);    // 左腿 lqr err x
    msg_log_all.data[13] = vmc_left_->get_lqr_err()(4);    // 左腿 lqr err vx
    msg_log_all.data[14] = vmc_right_->get_lqr_err()(0);   // 右腿 lqr err x
    msg_log_all.data[15] = vmc_right_->get_lqr_err()(4);   // 右腿 lqr err vx
    msg_log_all.data[16] = yaw_raw;
    msg_log_all.data[17] = yaw;
    msg_log_all.data[18] = vmc_left_->leg_len;             // 实时腿长
    msg_log_all.data[19] = vmc_right_->leg_len;
    msg_log_all.data[20] = vmc_left_->get_h_target_now();  // 指令高度
    msg_log_all.data[21] = vmc_right_->get_h_target_now();
    msg_log_all.data[22] = roll;

    DataLog_all_pub_->publish(msg_log_all);
}







/* ********************************************************************************
 *                    _______    _______   _________   _______            
 *                   (  ____ )  (  ___  )  \__   __/  (  ____ \  |\     /|
 *                   | (    )|  | (   ) |     ) (     | (    \/  | )   ( |
 *                   | (____)|  | (___) |     | |     | |        | (___) |
 *                   |  _____)  |  ___  |     | |     | |        |  ___  |
 *                   | (        | (   ) |     | |     | |        | (   ) |
 *                   | )        | )   ( |     | |     | (____/\  | )   ( |
 *                   |/         |/     \|     )_(     (_______/  |/     \|
********************************************************************************** */ 
void lqrController::setup_motor_command_free() {
    for(int i=0; i<8; i++) {
        motor_cmds[i].motor_id = mjid_map[0][i];
        motor_cmds[i].position = 0.0;
        motor_cmds[i].speed = 0.0;
        motor_cmds[i].kp = 0.0;
        motor_cmds[i].kd = 0.0;
        motor_cmds[i].control_mode = 3;    // 2-扭矩模式.  3-MIT模式
        motor_cmds[i].torque = 0.0;
    }
}

void lqrController::setup_motor_command_MIT_low() {
    if(platform == PLATFORM_SIM) return;

    for(int i=0; i<8; i++) {
        motor_cmds[i].motor_id = mjid_map[0][i];
        motor_cmds[i].position = 0.0;
        motor_cmds[i].speed = 0.0;
        motor_cmds[i].control_mode = 3;    // 2-扭矩模式.  3-MIT模式
        motor_cmds[i].torque = 0.0;
    }
    for(int i=0; i<4; i++) {
        motor_cmds[i].kp   = vmc_left_->kp_joints[i];
        motor_cmds[i+4].kp = vmc_right_->kp_joints[i];
        motor_cmds[i].kd   = vmc_left_->kd_joints[i];
        motor_cmds[i+4].kd = vmc_right_->kd_joints[i];
    }
    // 设置平衡状态(左右腿不同高度可能对应不同参数)
    for(int i=0; i<3; i++) {                                        // 给电机端设置期望位置(除了车轮), 也可以在set height中设置一次
        motor_cmds[i].position   = (vmc_left_->mjcf_q_eq_des(i) - mjzero_offset[i]) * real_dir[i]; 
        motor_cmds[4+i].position = (vmc_right_->mjcf_q_eq_des(i) - mjzero_offset[4+i])* real_dir[i+4];
    }
}

void lqrController::planning_standing() {
    // 在standing中调用, 动态调整期望高度, 以实现平稳上升
    double h0 = 0.27;   // 0.3
    double h1 = 0.50;
    double T = 0.6;     // 0.4
    if(!on_stand_planning) {
        if(vmc_left_->leg_len > h0 || vmc_right_->leg_len > h0) {
            stand_plan_t_start = t_platform;
            if(h0 < vmc_left_->leg_len) h0 = vmc_left_->leg_len;
            if(h0 < vmc_right_->leg_len) h0 = vmc_right_->leg_len;
            double v0 = vmc_left_->dleg_len;
            if(v0 < vmc_right_->dleg_len) v0 = vmc_right_->dleg_len;
            // 计算规划多项式曲线系数
            p_a = (3*v0*T + 6*h0 - 6*h1)/(3*T*T*T);
            p_b = (3*h1 - 3*h0 - 2*v0*T)/(T*T);
            p_c = v0;
            p_d = h0;

            on_stand_planning = true;  
        }
    } else {
        double t = t_platform - stand_plan_t_start;
        if(t>T) t=T;  
        plan_h = p_a * t*t*t + p_b*t*t + p_c*t + p_d;
        if(plan_h > h1) plan_h = h1;
        if(plan_h < h0) plan_h = h0;
        set_heights(plan_h, plan_h);
    }
}

// 动态计算HIP的期望位置, 让电机的PD层和LQR的目标一致
void lqrController::hip_PD_pos_calc() {
    // 在eq状态下, 添加一个gamma的bias

}

void lqrController::lim_knee_tq_rate() {
    const int L_id = 2;
    const int R_id = 6;  // 6
    double tau_kneeL_last = last_tau_applied[L_id];
    double tau_kneeR_last = last_tau_applied[R_id];

    if(status==WB_STANDING) {
        // motor_cmds[7].torque = 3.0;
        // 限制一下膝关节的输出变化率！
        // double dtau_lim = tau_knee_rate_lim*dt_tmp;
        double dtau_lim = tau_knee_rate_lim*0.001;
        // double dtau_lim = tau_knee_rate_lim*(t_platform-tlast_platform);
        if(motor_cmds[L_id].torque - tau_kneeL_last > dtau_lim) {   
            motor_cmds[L_id].torque = tau_kneeL_last + dtau_lim;
        }else if(motor_cmds[L_id].torque - tau_kneeL_last< -dtau_lim) {
            motor_cmds[L_id].torque = tau_kneeL_last - dtau_lim;
        }
        if(motor_cmds[R_id].torque - tau_kneeR_last > dtau_lim) {
            motor_cmds[R_id].torque = tau_kneeR_last + dtau_lim;
        }else if(motor_cmds[R_id].torque - tau_kneeR_last< -dtau_lim) {
            motor_cmds[R_id].torque = tau_kneeR_last - dtau_lim;
        }
    }
    // 记录输出扭矩
    for(int i=0; i<8; i++) {
        last_tau_applied[i] = motor_cmds[i].torque;
    }
}


void lqrController::imu_data_trans() {
    // 1. 计算连续的yaw角度
    static double last_yaw_raw = 0.0;
    static int yaw_cnt = 0;      // 旋转的圈数
    tf2::Matrix3x3 mat(quat);
    mat.getRPY(roll, pitch, yaw_raw);    // 计算RPY值
    if(yaw_raw-last_yaw_raw > mPI) { yaw_cnt--;
    }else if(yaw_raw-last_yaw_raw < -mPI) { yaw_cnt++; }
    last_yaw_raw = yaw_raw;
    yaw = 2*mPI * yaw_cnt + yaw_raw;
    
    // 2. 加速度计处理
    droll = gyro_imu(0); dpitch = gyro_imu(1); dyaw = gyro_imu(2);

    // 3. imu-acc到世界坐标系acc
    tf2::Vector3 gravity(0.0, 0.0, -9.8);
    gravity_b = mat.transpose() * gravity;    // 世界坐标系-->机身坐标系; 计算机身坐标系下的重力分量

    // 加速度计滤波
    const double accLPF = 0.008;    // LowPass Filter 系数
    double alpha = 0.0025 / (accLPF + 0.0025);
    // 物体的绝对加速度 = 加速度计读数（比力） + 重力加速度（在同一坐标系下）
    acc_b = (acc_imu + gravity_b) * alpha + acc_b*(1-alpha);  // 新值占权重0.23, 上一个占权重0.23*0.77
    acc_n = mat * acc_b;
}


// 根据roll角, 动态计算腿长
void lqrController::adaptive_len_adjust() {
    static long delta_hFF_cnt = 0;
    static bool change_left = false;      // 标记这条腿是否正在调整中(当腿目标恢复原长可以取消)
    static bool change_right = false;
    delta_hFF_cnt ++;     // 100ms调整一次
    if (status==WB_RUNNING) {
        // 0.4*tan(0.05) = 0.02，对应角度2.86度
        if(abs(roll)>0.01) {
            double h_left, h_right;            // 左右腿目标高度
            double delta_hFF = 0.4*tan(roll);
            double leg_len_left_now = vmc_left_->leg_len;
            double leg_len_right_now = vmc_right_->leg_len;
            if(!change_left && !change_right) {
                if(delta_hFF > 0) {
                    change_left = true;
                } else {
                    change_right = true;
                }
            }
            if(change_left) {
                h_right = leg_len_cmd;
                // if(leg_len_left_now - delta_hFF > leg_len_cmd) {
                if(leg_len_left_now  > leg_len_right_now) {
                    change_left = false;
                    h_left = leg_len_cmd;
                } else {
                    h_left = leg_len_left_now - delta_hFF;
                    if(h_left > leg_len_cmd) h_left = leg_len_cmd;
                    if(h_left < 0.3) h_left = 0.3;
                }
                LOG_INFO(C_G << " Change left leg:  " << C_End << leg_len_left_now - delta_hFF);
            }else if(change_right) {
                h_left = leg_len_cmd;
                // if(leg_len_right_now + delta_hFF > leg_len_cmd) {
                if(leg_len_right_now  > leg_len_left_now) {
                    change_right = false;
                    h_right = leg_len_cmd;
                } else {
                    h_right = leg_len_right_now + delta_hFF;
                    if(h_right > leg_len_cmd) h_right = leg_len_cmd;
                    if(h_right < leg_len_cmd) h_right = 0.3;
                }
                LOG_INFO(C_G<<" Change right leg: " << C_End << leg_len_right_now + delta_hFF);
            }
            set_heights(h_left, h_right);
        }
    }
}

// 分别设置左右腿高度, 并设置电机目标状态
void lqrController::set_heights(double h_left, double h_right) {
    vmc_left_->set_height(h_left);
    vmc_right_->set_height(h_right);
    setup_motor_command_MIT_low();
}


void lqrController::autoCommand() {
    // 
    if(use_autoCommand && platform==PLATFORM_SIM) {
        if(t_platform>3.0 && status==WB_REST) {
            status_trans(WB_STANDING);
            reset_wheel_offset();          // 重置此时车轮的角度
        }
        if(t_platform>6.0 && t_platform<10.5 && status==WB_RUNNING) {
            // 指定运动
            make_joy_cmd_desired(0.6, 0.0);
        }
        if(t_platform>10.5 && status==WB_RUNNING) {
            // 指定运动
            make_joy_cmd_desired(0.0, 0.0);
        }
        // 恢复手柄控制
        if(t_platform>20) {
            use_autoCommand = false;
            LOG_INFO(C_R << "恢复手柄控制" << C_End);
        } 
    }
}


void lqrController::make_joy_cmd_desired(double vx_cmd_in, double dyaw_cmd_in){
    const double alpha = 0.2;    // LPF
    vx_unified_cmd_raw = vx_cmd_in *alpha + vx_unified_cmd_raw*(1-alpha);  // [0, 1] m/s
    dyaw_cmd_raw = dyaw_cmd_in*alpha + dyaw_cmd_raw*(1-alpha);             // [0, 1] rad/s
    vx_unified_cmd = vx_unified_cmd_raw;
}


// 离地策略(覆盖腿长主动调节)
void lqrController::deal_with_off_ground_normal() {
    // 任意一条腿离地, [LQR+前馈]完全置零
    if(!vmc_left_->get_contact_status() || !vmc_right_->get_contact_status()) {
        off_gnd_status = true;
        tau_all.setZero();    // 任意一方离地, 相关控制量全部置零,全部交给关节PD
        if(!vmc_left_->get_contact_status() && !vmc_right_->get_contact_status()) return;  // 双腿均离地
        // 长腿设置低PD, 短腿设置高PD
        set_heights(leg_len_cmd, leg_len_cmd);
        if(roll<0) {   // roll小于零, 左侧下落比较多, 左侧希望尽快着地
            vmc_left_->set_kp_kd(Kpd_joints_airH);
            vmc_right_->set_kp_kd(Kpd_joints_airL);
        } else {
            vmc_left_->set_kp_kd(Kpd_joints_airL);
            vmc_right_->set_kp_kd(Kpd_joints_airH);
        }
        setup_motor_command_MIT_low();
    } else {    // 双腿全部着地
        if(off_gnd_status) {
            off_gnd_status = false;
            vmc_left_->set_kp_kd(Kpd_joints_normal);
            vmc_right_->set_kp_kd(Kpd_joints_normal);
            setup_motor_command_MIT_low();
        }
    }

}

// 腿长主动调节策略(想要完全主动适应还是比较困难的)
void lqrController::adaptive_len_adjust2() {
    static bool change_left = false;      // 标记主要调节腿
    static bool change_right = false;
    if (status==WB_RUNNING) {
        // 0.4*tan(0.05) = 0.02，对应角度2.86度
        if(abs(roll)>0.01) {
            double h_left, h_right;            // 左右腿目标高度, tan在零位置导数为1
            double delta_hFF = 0.4*tan(roll) + 0.02*(droll);
            double leg_len_left_now = vmc_left_->leg_len;
            double leg_len_right_now = vmc_right_->leg_len;
            

            // 1. roll角偏差大-->触发主动调整
            if(!change_left && !change_right) {
                if(delta_hFF > 0) {
                    change_left = true;
                } else {
                    change_right = true;
                }
            }
            // 2. 主要调整左腿
            if(change_left) {
                h_right = leg_len_cmd;
                if(leg_len_left_now  > leg_len_right_now) {
                    change_left = false;
                    h_left = leg_len_cmd;
                } else {
                    // 分配delta_hFF,
                    // if(abs(delta_hFF)>0.08)
                    h_left = leg_len_left_now - delta_hFF;
                    if(h_left > leg_len_cmd) h_left = leg_len_cmd;
                    if(h_left < 0.3) h_left = 0.3;
                }
            }else if(change_right) {
                h_left = leg_len_cmd;
                // if(leg_len_right_now + delta_hFF > leg_len_cmd) {
                if(leg_len_right_now  > leg_len_left_now) {
                    change_right = false;
                    h_right = leg_len_cmd;
                } else {
                    h_right = leg_len_right_now + delta_hFF;
                    if(h_right > leg_len_cmd) h_right = leg_len_cmd;
                    if(h_right < 0.3) h_right = 0.3;
                }
            }
            set_heights(h_left, h_right);
        }
    }
}
