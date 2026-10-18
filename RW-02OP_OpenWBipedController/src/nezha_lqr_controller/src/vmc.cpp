#include "nezha_lqr_controller/vmc.hpp"
#include <cmath>
#include <iostream>
// 虚拟模型控制，对单腿的出力控制
// VMC是有极限的，我不做VMC啦，出来吧WBC
using namespace vmc;


VMC::VMC(std::shared_ptr<lqrKTable> lqr_k_table, LegSide side, Vec4d kp, Vec4d kd)
    : lqr_k_table(lqr_k_table), 
      side(side){
    l = lqr_k_table->l1;        // 目前的机器人l1==l2

    // VMC中将thigh，shank竖直认为零位,
    // 即当mjcf角度为零时，vmc角度为[0, -ang0, 2*ang0]
    double ang0 = atan(0.15/0.25981);  // TODO:也许应该在参数文件中
    theta_ini << 0.0, -ang0, 2*ang0, 0.0;
    kp_joints = kp;
    kd_joints = kd;
}
VMC::VMC(std::shared_ptr<lqrKTable> lqr_k_table, LegSide side, double* Kpd)
    : lqr_k_table(lqr_k_table), 
      side(side) {
    l = lqr_k_table->l1;        // 目前的机器人l1==l2

    // VMC中将thigh，shank竖直认为零位,
    // 即当mjcf角度为零时，vmc角度为[0, -ang0, 2*ang0]
    double ang0 = atan(0.15/0.25981);  // TODO:也许应该在参数文件中
    theta_ini << 0.0, -ang0, 2*ang0, 0.0;
    for(int i=0; i<4; i++) {
        kp_joints(i) = Kpd[i];
        kd_joints(i) = Kpd[i+4];
    }
}

void VMC::set_kp_kd(double* kpd) {
    for(int i=0; i<4; i++) {
        kp_joints(i) = kpd[i];
        kd_joints(i) = kpd[i+4];
    }
}


void VMC::set_state(Vec4d q_mjcf, 
                    Vec4d dq_mjcf, 
                    double gamma, 
                    double dgamma,
                    double acc_nz,
                    Vec4d mj_tau, double dt) {
    this->q_mjcf  = q_mjcf;
    this->dq_mjcf = dq_mjcf;
    this->gamma = gamma;
    this->dgamma = dgamma;
    theta = q_mjcf + theta_ini;
    
    _leg_fwd();                 // 基础计算
    xBias = cos(gamma)*wheel_pos(0) + sin(gamma)*wheel_pos(2);  // 已验证
    _jacobian_calc();
    _calc_pin_from_mjcf();
    _calc_task_from_pin();
    // calc_pd_for_eq();
    _calc_for_gnd_detect(dt);
    on_off_ground_detect(acc_nz, mj_tau);
    leg_len = 2 * l * cos(theta(2)/2);
    dleg_len = -l * sin(theta(2)/2)*dq_mjcf(2);
}

// 计算需要的参数
void VMC::set_height(double h_target) {
    h_target_now = h_target;
    lqr_k_table->poly_calc_at(h_target, lqrK, lqr_q_eq_task, lqr_dq_eq_task, 
        lqr_q_eq_pin, lqr_dq_eq_pin, lqr_tau_eq_pin);
    //              abad       hip              knee          wheel(无意义)
    mjcf_q_eq_des << 0, -lqr_q_eq_pin(5), -lqr_q_eq_pin(4), -lqr_q_eq_pin(3);
    
    // std::cout<< "\x1b[33m---------  设置高度 ---------\x1b[0m" << std::endl;
    // std::cout<<"\x1b[33m mjcf_q_eq_des\x1b[0m = [ 0.0, " << -lqr_q_eq_pin(5) << ", " << -lqr_q_eq_pin(4) << ", " << -lqr_q_eq_pin(3) << "]" << std::endl;
}

void VMC::lqr_calc(double x_unified, double x_des, double vx_des) {
    // 计算lqr输出
    q_task(0) = x_unified + xBias;   // 重置状态中的 x 位置
    lqr_q_eq_task(0) = x_des;
    lqr_dq_eq_task(0) = vx_des;
    q_err_task  = lqr_q_eq_task  - q_task;
    dq_err_task = lqr_dq_eq_task - dq_task;
    lqr_err.head<4>() = q_err_task;
    lqr_err.tail<4>() = dq_err_task;
    lqr_tau = lqrK * lqr_err;
}

// TODO: 离地检测，离地状态下应该锁住对应的车轮












void VMC::_jacobian_calc() {
    // 更新当前腿对机身平行坐标系的雅可比矩阵
    double theta1_wg = theta(1) + gamma;
    double s0 = sin(theta(0)), c0 = cos(theta(0));
    double s1 = sin(theta1_wg), c1 = cos(theta1_wg);
    double s1_2 = sin(theta1_wg + theta(2));
    double c1_2 = cos(theta1_wg + theta(2));
    jacMat <<         0,       -l*(c1 + c1_2),   -l*c1_2,
              l*(c1+c1_2)*c0, -l*(s1+s1_2)*s0, -l*s1_2*s0,
              l*(c1+c1_2)*s0,  l*(s1+s1_2)*c0,  l*s1_2*c0;
}

void VMC::_leg_fwd() {
    // 计算足端相对机身坐标系的位置, 达妙开源的没有侧摆关节
    double s0 = sin(theta(0)), c0 = cos(theta(0));
    double s1 = sin(theta(1)), c1 = cos(theta(1));
    double s1_2 = sin(theta(1) + theta(2));
    double c1_2 = cos(theta(1) + theta(2));
    wheel_pos << l*(-s1 - s1_2), l*(c1 + c1_2)*s0, -l*(c1 + c1_2)*c0;
    if(side==LEFT){
        wheel_pos = wheel_pos + lqr_k_table->vec_1l + lqr_k_table->vec_2l;
    } else {
        wheel_pos = wheel_pos + lqr_k_table->vec_1r + lqr_k_table->vec_2r;
    }
}

// 计算速度以及角速度，求导计算流程不变
void VMC::_trans1_mjcf2pin(Vec4d& q_src_mjcf, Vec7d& q_des_pin, double& gamma_in) {
    //                               hip                 knee               wheel
    q_des_pin(2) = gamma_in - (- q_src_mjcf(1)) - (- q_src_mjcf(2)) - (- q_src_mjcf(3));   // 滚过的转角，留给编译器优化
    q_des_pin(0) = q_des_pin(2) * lqr_k_table->R_wheel;    // 滚过的距离
    q_des_pin(1) = 0.0;
    q_des_pin(3) = - q_src_mjcf(3);    // 车轮转角
    q_des_pin(4) = - q_src_mjcf(2);    // knee转角
    q_des_pin(5) = - q_src_mjcf(1);    // hip转角
    q_des_pin(6) = - q_src_mjcf(0);    // abad转角
}
void VMC::_trans2_pin2task(Vec7d& q_src_pin, Vec4d& q_des_task, double& gamma_in) {
    double R = lqr_k_table->R_wheel;
    q_des_task(0) = q_src_pin(0);       // x
    q_des_task(1) = q_src_pin(0)/R + q_src_pin(3) + 0.5*q_src_pin(4);  // 机身转角，TODO：如果腿长不一致，不能如此计算
    q_des_task(2) = q_src_pin(4);
    // q_des_task(3) = q_src_pin(0)/R + q_src_pin[3] + q_src_pin[4] + q_src_pin[5];
    q_des_task(3) = gamma_in;    // 等效的
}


void VMC::_calc_pin_from_mjcf() {
    // 更新计算pinocchio下坐标表达
    _trans1_mjcf2pin(q_mjcf, q_pin, gamma);
    _trans1_mjcf2pin(dq_mjcf, dq_pin, dgamma);
}

void VMC::_calc_task_from_pin() {
    _trans2_pin2task(q_pin, q_task, gamma);
    _trans2_pin2task(dq_pin, dq_task, dgamma);
}

bool VMC::is_fall() {
    if(abs(q_err_task(1)>0.5 || q_err_task(3)>0.5)) {
        return true;
    } else {return false; }
}

Vec4d VMC::calc_pd_for_eq() {
    for(int i=0; i<4; i++) {
        tau_pd_eq(i) = kp_joints(i) * (mjcf_q_eq_des(i) - q_mjcf(i)) +
                       kd_joints(i) * (0 - dq_mjcf(i));
    }
    return tau_pd_eq;
}

// 计算着地检测相关量, 主要包括连杆长度二阶导, 等
// 如果要微分,仿真不同步有点麻烦, 
void VMC::_calc_for_gnd_detect(double dt) {
    if(dt>0.0005) {
        zf = -wheel_pos(0)*sin(gamma) + wheel_pos(2)*cos(gamma);
        dzf = 0.3*dzf + 0.7*(zf - zf1)/dt;    // 新数据权重0.7
        ddzf = 0.2*ddzf + 0.8*(dzf - dzf1)/dt;   // 足端加速度
        dzf1 = dzf; zf1 = zf;
        ddz_update_idx ++;
    }
}

// 着地检测: 当判定离地, 只保持关节PD
void VMC::on_off_ground_detect(double acc_nz, Vec4d mj_tau) {
    // acc_nz, 机身的平行于世界坐标系加速度
    ddzf_n = acc_nz + ddzf;         // 铝箔
    ddzf_n_buffer[ddz_update_idx%4] = ddzf_n;
    // 要滤一下
    ddzf_n_filtered = (ddzf_n_buffer[0] + ddzf_n_buffer[1] + ddzf_n_buffer[2] + ddzf_n_buffer[3])/4;

    
    // 计算实际关节出力到地面接触力的映射
    F_foot = mj_tau.head(3).transpose() * jacMat.inverse();
    double P = F_foot(2);  // 腿部对车轮向下的压力（z方向, 这里直接计算通常为负数）
    double M_use = lqr_k_table->m_wheel + lqr_k_table->m_shank;
    Fn_est = M_use*(ddzf_n_filtered + 9.81) - P;

    // 连续N个周期低于阈值才判定离地，避免误判
    if(Fn_est < 30) {
        off_ground_cnt++;
        if(off_ground_cnt >= OFF_GROUND_CNT_THRESHOLD) {
            contact_flag = false;
            off_ground_cnt = OFF_GROUND_CNT_THRESHOLD;  // 防止溢出
        }
    } else {
        off_ground_cnt = 0;   // 着地后立即重置计数
        contact_flag = true;
    }
}





