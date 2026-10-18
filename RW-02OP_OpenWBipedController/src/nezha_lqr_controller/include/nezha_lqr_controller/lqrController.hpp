#pragma once

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

#include "encos_eth_master/ethercat_controller.h"
#include "yesense_imu/imu_driver.h"

#include "nezha_lqr_controller/lqrKTable.hpp"
#include "nezha_lqr_controller/vmc.hpp"
#include <nezha_lqr_controller/utils.h>



#define LOG_INFO(...)  RCLCPP_INFO_STREAM(this->get_logger(), __VA_ARGS__);


#define mPI  3.141592653589793
#define CALI_abad_pos (80.0 * mPI/180.0)
#define CALI_hip_pos  (60.0 * mPI/180.0)
#define CALI_knee_pos (38.0 * mPI/180.0)

using F64MsgArray = std_msgs::msg::Float64MultiArray;

// 仅代表Logitech手柄X-Input模式，灭灯mode
typedef enum {
  ButtonA = 0,
  ButtonB = 1,
  ButtonX = 2,
  ButtonY = 3,
  ButtonLB = 4,
  ButtonRB = 5,
  ButtonBack = 6,
  ButtonStart = 7,
  ButtonLS = 9,   // 左摇杆按钮
  ButtonRS = 10,
} JoyButtonID;


typedef enum{
  PLATFORM_SIM,
  PLATFORM_REAL,
}PlatformType;

typedef enum{
  WB_REST,          // 上电初始化
  WB_STANDING,      // 站立动作保护阶段
  WB_FALL,          // 摔倒
  WB_RUNNING,       // 常规运行状态
  WB_CALIBRATING,   // 标零（如果可以一起标的话）
  WB_AIR,           // 悬空状态
  WB_JUMP,          // 跳跃(运行时, 自动流程管理)
} WBStatus;



class lqrController : public rclcpp::Node
{
public:
  lqrController();
  void ini_parameters();

private:
  rclcpp::TimerBase::SharedPtr timer_;


// lqr-json参数
private:
  std::shared_ptr<lqrKTable>  lqr_k_table_;

private:
  void controlLoop();
  long loopCnt = 0;

// ----------------- 补丁 -----------------
private:
  double stand_plan_t_start;
  bool on_stand_planning = false;
  double p_a, p_b, p_c, p_d;   // plan poly4 系数
  double plan_h;
  void planning_standing();    // 有一定规划的站立
  void hip_PD_pos_calc();
  double last_tau_applied[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  void lim_knee_tq_rate();    // 限制膝关节输出扭矩
  void autoCommand();         // 仿真中自主指令, 保证测试流程的一致性
  bool use_autoCommand = true;
  void make_joy_cmd_desired(double vx_cmd_in, double dyaw_cmd_in);   // 从joy中获取规整后的指令
  void deal_with_off_ground_normal();       // 离地相关策略
  bool off_gnd_status = false;
  double jump_prepare_height = 0.27;
  double leg_len_cmd_restore = jump_prepare_height;
  int jump_stage = 0;
  double t_jump_LO = 0.0;
  bool joy_action_protect_flag = false;
  double t_joy_action_start;
  double t_protect;
  void joy_action_protect(double t=0.5) {
    t_joy_action_start = t_platform; 
    joy_action_protect_flag = true;
    t_protect = t;
  }


// Hardwares(motor & IMU)
private:
  void real_setup_communication();
  void real_get_eth_motor_data();
  void real_get_IMU_data();
  void real_apply_torque();
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  // uint16_t motor_id[8] = {11, 13, 15, 19, 12, 16, 14, 20};  // 对应joint顺序
  uint16_t mjid_map[2][8] = {{11, 13, 15, 19, 12, 16, 14, 20}, 
                             {0, 1, 2, 3, 4, 5, 6, 7}};
  std::vector<MotorBack> motor_back_data;
  std::unique_ptr<EtherCATController> eth_controller_;
  std::shared_ptr<imu::ImuDriver> yesense_imu_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subs_;
  sensor_msgs::msg::Joy latest_joy_msg_; 
  uint32_t err_motor_msg_cnt = 0;
  std::vector<ControllerCommand> motor_cmds;
  void real_calibrating_motor();
  void real_record_motor_zero();
  

  double motor_cali_max[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double motor_cali_min[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double motor_pos_raw[8];
  double motor_vel_raw[8];
  double motor_tq_raw[8];
  // double mjzero_offset[8] = {-0.0821454, 0.633105, -0.592843, 0, -0.618626, 1.0472, 1.14363, 0.618563};
  // double mjzero_offset[8] = {-0.0813825, 0.644932, -0.595895, 9.26242, -0.623585, 1.0472, 1.14173, 11.0538};
  // double mjzero_offset[8] = {-0.0848157, 0.643025, -0.588265, 1.96441, -0.614047, 1.0472, 1.14936, 0};
  double mjzero_offset[8] = {-0.0848157, 0.643025, -0.588265, 1.96441, -0.614047, 1.1392, 1.14936, 0};     // 手动增加5.3度, 右腿hip, 倒数第三个
  void reset_wheel_offset() {mjzero_offset[3]= motor_pos_raw[3]; mjzero_offset[7]= motor_pos_raw[7]; }
  // double mjzero_offset[8] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};   //  读数 + offset = 实际角度
  double motor_cali_pos[8] = {CALI_abad_pos, CALI_hip_pos, -CALI_knee_pos, 0.0, 
                              -CALI_abad_pos, CALI_hip_pos, -CALI_knee_pos, 0.0 }; // 限位块所在位置

// simulation
private:
  void sim_setup_communication();
  void sim_imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
  void sim_js_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void sim_apply_torque();

  float mj_dir[8] = {1, -1, -1, 1,   -1, -1, -1, 1};    // Mujoco mjcf模型电机方向
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sim_imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sim_js_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr sim_jc_pub_;
  float real_dir[8] = {1, -1, 1, 1, 1, 1, -1, -1};

private:
  double kp_roll = 300, kd_roll = 50;
  double kp_yaw = 10, kd_yaw = 3;
  double kp_align = 20, kd_align = 5;
  void setup_motor_command_free();      // 设置电机默认指令, 保证可以实现free
  void setup_motor_command_MIT_low();
  void set_heights(double h_left, double h_right);
  // 单腿PD参数定义
  double Kpd_joints_normal[8] = {200,  20,  20,   0,
                                20,   10,   6,    0};
  double Kpd_joints_airH[8] =   {200,  20,  50,   0,
                                 20,   10,   10,    0};      // 空中膝关节高PD值
  double Kpd_joints_airL[8] =   {200,  20,  0,   0,
                                 20,   10,   0,    0};     // 
  double Kpd_joints_free[8] =   {0,  0,  0,   0,
                                 0,  0,  0,   0};     // 
  double Kpd_joints_LO[8]   =   {200,  20,  20,   0,
                                 20,   10,   1,   0};     // lift off(跳跃的抬升阶段)

  // RollNew
  double tau_roll_max = 20.0;           // 力矩补偿饱和值
  double roll_threshold_for_leg = 0.08; // 触发腿长调节的roll阈值 (~4.5°)
  double roll_persist_time = 0.0;       // roll超阈值的持续时间
  double last_roll_check_time = 0.0;
  void adaptive_len_adjust_smooth();

// -------------------------- Data Log --------------------------
private:
  void setup_data_log_publisher();
  void dataLog_publish();
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr DataLog_mj_q_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr DataLog_mj_dq_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr DataLog_mj_tau_pub_;
  void DataLog_joints_basic_publish();
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr DataLog_mj_tau_lqr_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr DataLog_lqr_slot_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr DataLog_mj_tau_lqr_err_pub_;
  void DataLog_lqr_publish();
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr DataLog_Fn_est_pub_;
  void DataLog_Fn_publish();
  void DataLog_Status_publish();    // 一些离散的状态变化发布
  bool shouldLog = true;
 
  // 统一数据发布(针对特定调试目标进行的发布)
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr DataLog_all_pub_;
  void DataLog_all_contact_publish();               // 接触相关的消息发布
  void DataLog_all_stable_following_publish();      // 稳定运行后, 跟踪相关的调试消息发布
  // void DataLog_all_RL_publish();
  


// -------------------------- 全局状态 --------------------------
private:
  void lqr_process_tranditional();
  double PD_for_roll();
  double PD_for_yaw();
  double PD_for_align();
  void basic_state_transform();
  PlatformType platform;
  Eigen::Matrix<double, 8, 1> mj_q;       // mujoco模型下的关节角度(视为标准中转)
  Eigen::Matrix<double, 8, 1> mj_q_ang;   // 角度表示的
  Eigen::Matrix<double, 8, 1> mj_dq;
  Eigen::Matrix<double, 8, 1> mj_tau;
  Eigen::Matrix<double, 8, 1> tau_ff;     // 提供平衡态的前馈扭矩
  Eigen::Matrix<double, 8, 1> tau_lqr;
  Eigen::Matrix<double, 8, 1> tau_eq_pd;
  Eigen::Matrix<double, 8, 1> tau_all;     // 最终输出控制扭矩
  double tau_knee_rate_lim = 2000;          // 60Nm/s, 限制扭矩的变化率，计算出来比较大，但慢慢给，避免地面支撑力瞬变，参数
  double tq_limit[8] = {60, 60, 80, 40, 60, 60, 80, 40};  // for FF

  tf2::Quaternion quat;                    // IMU & 姿态
  Eigen::Matrix<double, 3, 1> gyro_imu;
  tf2::Vector3 acc_imu;  // IMU传感器的原始加速度值
  tf2::Vector3 acc_b;    // 机身Acc(减去gravity分量,并滤波的)
  tf2::Vector3 acc_n;    // 机身, 世界坐标系(或者导航坐标系)acc
  double roll, pitch, yaw, yaw_raw;
  double droll, dpitch, dyaw;
  Eigen::Matrix<double, 3, 1> gravity;
  void imu_data_trans();
  void adaptive_len_adjust();
  void adaptive_len_adjust2();

  double t_platform;          // 目标平台的时间
  double dt_tmp;              // 当前控制周期的时间间隔
  double tlast_platform = -1.0;      // 控制器记录的上次平台时间
  rclcpp::Time t_platform_start_;     // 记录控制器起始时平台的时间
  double long_press_t_button_cali = 0.0;
  WBStatus status = WB_REST;
  void status_trans(WBStatus target_status);
  double status_switch_time = 0.0;
  double x_unified;         // united x，左右两侧统一估计的x，通过积分获得
  double vx_unified;

  double x_unified_cmd;     // 控制指令（需要限error幅度）
  double yaw_cmd;
  double vx_unified_cmd;
  double dyaw_cmd;
  double vx_unified_cmd_raw; // 手接收的原始指令数据
  double dyaw_cmd_raw;
  double leg_len_cmd = 0.5;  // 目标腿长
  double dyna_leg_len_cmd = 0.5;
  std::unique_ptr<vmc::VMC> vmc_left_;
  std::unique_ptr<vmc::VMC> vmc_right_;
  double control_frequency;
  std::string param_platform;
  double tau_wheel_yaw;
  double tau_yaw_err_int;       // yaw的误差积分
  tf2::Vector3 gravity_b;

// // ------------------------- RL Co -----------------------------
// // MARK: RL
// private:
//   void nn_init();
//   // PolicyFlat policy_flat;
//   std::unique_ptr<PolicyFlat> p_policy_flat;
//   void policy_flat_refer(double t_now);     // 平地数据收集与推理
//   // URDF(usd)电机方向
//   // abad_L, hip_L, knee_L, wheel_L, abad_R, hip_R, knee_R, wheel_R
//   //  +x      -y      +y      +y       +x     +y      -y      -y
//   float usd_dir[8] = {1, -1, 1, 1,   1, 1, -1, -1};
//   std::vector<double> obs_mj_pos{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//   std::vector<double> last_action;
//   // Isaac Out: [abad_L, abad_R, hip_L, hip_R, knee_L, knee_R, wheel_L, wheel_R]
//   std::vector<double> action = std::vector<double>(8);
//   torch::DeviceType device;
//   int jid_map_usd[8] = {0, 2, 4, 6, 1, 3, 5, 7};
//   std::vector<double> usd_q = std::vector<double>(8);      // 按照usd文件的顺序以及方向处理后的实际
//   std::vector<double> usd_dq = std::vector<double>(8);
//   void update_status_usd();
  
//   void rl_sim_apply_torque();
//   void rl_sim_apply_MIT();
//   void rl_real_apply_MIT();
//   void rl_get_std_pos_vel_des();     // 
//   double usd_kp_act[8] = {40, 40, 40, 40, 40, 40, 0.0, 0.0};          // 位置与速度的PD参数(usd顺序)，控制器端直接计算输出扭矩
//   double usd_kd_act[8] = {2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 0.8, 0.8};
//   // double act_scales[8] = {0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 1.0, 1.0};   // aciton scale
//   double act_scales[8] = {0.25, 0.25, 0.25, 0.25, 0.25, 0.25, 2.0, 2.0};
//   bool using_RL_flag = false;
//   int rl_warmup_cnt = 0;
//   double rl_t_last_compute = 0.0;
//   // double mj_kp_act[8] = {40,  40,  40,  0.0, 40,  40,  40,  0.0};          // 位置与速度的PD参数(默认顺序)
//   // double mj_kd_act[8] = {2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5, 2.5};
//   double mj_kp_act[8] = {60,  40,  40,  0.0, 60,  40,  40,  0.0};          // 位置与速度的PD参数(默认顺序)
//   double mj_kd_act[8] = {4.0, 4.5, 4.5, 4.5, 4.0, 4.5, 4.5, 4.5};
//   std::vector<double> rl_pos_des_std = std::vector<double>(8);
//   std::vector<double> rl_vel_des_std = std::vector<double>(8);
};
