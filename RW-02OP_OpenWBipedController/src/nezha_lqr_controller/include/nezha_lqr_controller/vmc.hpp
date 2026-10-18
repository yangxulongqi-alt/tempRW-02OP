#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <memory>
#include "nezha_lqr_controller/lqrKTable.hpp"

namespace vmc {
using Vec7d = Eigen::Matrix<double, 7, 1>;
using Vec4d = Eigen::Matrix<double, 4, 1>;
using Vec8d = Eigen::Matrix<double, 8, 1>;
using Vec3d = Eigen::Matrix<double, 3, 1>;

typedef enum{
    LEFT,
    RIGHT,
}LegSide;

class VMC {
public:
    VMC(std::shared_ptr<lqrKTable> lqr_k_table, LegSide side, Vec4d kp, Vec4d kd);
    VMC(std::shared_ptr<lqrKTable> lqr_k_table, LegSide side, double* Kpd);
public:
    void set_state(Vec4d mjcf_q, Vec4d dq_mjcf, double gamma, double dgamma, double acc_nz, Vec4d mj_tau, double dt);
    void lqr_calc(double x_unified, double x_des, double vx_des);
    void set_height(double h_target);
    Vec3d get_lqr_tau() { return lqr_tau; }
    Vec4d get_tau_pd_eq() {return tau_pd_eq; }
    Vec8d get_lqr_err() {return lqr_err;}
    void reset_kp_kd(Vec4d kp, Vec4d kd){kp_joints = kp; kd_joints = kd;}    // 重置Kp, Kd, 主要用于仿真
    void set_kp_kd(double* kpd);
    
public:
    double get_wheel_vel_x(){ return dq_task(0); };   // 车轮中心速度
    double get_wheel_pos_y(){ return wheel_pos(1); }
    double get_task_alpha() {return q_task(1); }
    double get_task_dalpha() {return dq_task(1); }
    Eigen::Matrix3d get_jacobian() {return jacMat; }
    bool is_fall();
    void on_off_ground_detect(double acc_nz, Vec4d mj_tau);
private:
    void _jacobian_calc();
    void _leg_fwd();
    void _trans1_mjcf2pin(Vec4d& q_src_mjcf, Vec7d& q_des_pin, double& gamma_in);
    void _trans2_pin2task(Vec7d& q_src_pin,  Vec4d& q_des_task, double& gamma_in);
    void _calc_pin_from_mjcf();
    void _calc_task_from_pin();
    void _calc_for_gnd_detect(double dt);
    
public:
    Vec4d calc_pd_for_eq();

private:
    std::shared_ptr<lqrKTable> lqr_k_table;   // 控制相关参数
    double l;             // shank与thigh长度
public:
    double leg_len;       // 支撑的等效长度(LQR参数使用的这个)
    double dleg_len;
private:
    double Fz_des;        // 目标z向支撑力
    double Fx_des;        // 目标x方向力
    
private:
    Vec4d theta;      // abad, hip, knee, wheel
    Vec4d theta_ini;  // 
    Vec4d q_mjcf, dq_mjcf;      // 
    Vec7d q_pin, dq_pin;        // pinocchio建模模型表达：["x_wheel", "z_wheel", "wheelRot", "shank_ang", "thigh_ang", "abad_ang", "head_ang"]
    Vec4d q_task, dq_task;      // ["x_wheel", "alpha", "kneeAng", "gamma"]
    Vec4d q_err_task, dq_err_task;
    Vec8d lqr_err;
    double gamma, dgamma;       // pitch角度
    double xBias;     // 车轮中心相对质心在x方向的偏移（考虑pitch）
    Eigen::Matrix3d jacMat;
    LegSide side;
    Vec3d wheel_pos;
    Vec3d lqr_tau;
    Vec4d tau_pd_eq;
public:
    Vec4d mjcf_q_eq_des;      // 平衡状态期望q in mj



private:
    Eigen::Matrix<double, 3, 8> lqrK;                  // 指定平衡态系数矩阵
    Eigen::Matrix<double, 4, 1> lqr_q_eq_task;
    Eigen::Matrix<double, 4, 1> lqr_dq_eq_task;
    Eigen::Matrix<double, 7, 1> lqr_q_eq_pin;
    Eigen::Matrix<double, 7, 1> lqr_dq_eq_pin;
    Eigen::Matrix<double, 7, 1> lqr_tau_eq_pin;

public:
    Vec4d kp_joints;
    Vec4d kd_joints;

private:
    double zf, zf1;     // z_foot
    double dzf, dzf1;
    double ddzf;
    double ddzf_n;      // 世界坐标系下车轮加速度估计
    double ddzf_n_buffer[4] = {0.0, 0.0, 0.0, 0.0};
    long ddz_update_idx = 0;
    double Fn_est;
    double ddzf_n_filtered;
    Vec3d F_foot;    // 足端支反力计算(暂存)
    bool contact_flag = true;
    int off_ground_cnt = 0;              // 连续离地计数
    static constexpr int OFF_GROUND_CNT_THRESHOLD = 5;  // 连续N个周期才判定离地
    double h_target_now;

public:
    double get_ddzf_filtered() {return ddzf_n_filtered; }
    double get_Fn_est() {return Fn_est; }
    double get_F_foot_z() {return F_foot(2); }
    bool get_contact_status() {return contact_flag; }
    double get_zf() {return ddzf_n; }
    double get_h_target_now() {return h_target_now; }

};

}