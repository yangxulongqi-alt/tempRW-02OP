#pragma once
#include <Eigen/Core>


class lqrKTable{
    // lqr-json参数读取
private:
    double lqrK_coeffs[24][5];                         // poly5拟合系数
    double lqr_q_eq_task_coeffs[4][5];
    double lqr_dq_eq_task_coeffs[4][5];
    double lqr_q_eq_pin_coeffs[7][5];
    double lqr_dq_eq_pin_coeffs[7][5];
    double lqr_tau_eq_pin_coeffs[7][5];

    // Eigen::Matrix<double, 3, 8> lqrK;                  // 指定平衡态系数矩阵
    // Eigen::Matrix<double, 8, 1> lqr_q_eq_task;
    // Eigen::Matrix<double, 8, 1> lqr_dq_eq_task;
    // Eigen::Matrix<double, 8, 1> lqr_q_eq_pin;
    // Eigen::Matrix<double, 8, 1> lqr_dq_eq_pin;
    // Eigen::Matrix<double, 8, 1> lqr_tau_eq_pin;

public:
    double m_head, m_abad, m_thigh, m_shank, m_wheel;
    double R_wheel;
    double l1, l2;
    Eigen::Matrix<double, 3, 1> vec_1l, vec_2l;
    Eigen::Matrix<double, 3, 1> vec_1r, vec_2r;
    


public:
    lqrKTable();
    // void poly_calc_at(double h);
    void poly_calc_at(double h, Eigen::Matrix<double, 3, 8>& lqrK,
                             Eigen::Matrix<double, 4, 1>& lqr_q_eq_task,
                             Eigen::Matrix<double, 4, 1>& lqr_dq_eq_task,
                             Eigen::Matrix<double, 7, 1>& lqr_q_eq_pin,
                             Eigen::Matrix<double, 7, 1>& lqr_dq_eq_pin,
                             Eigen::Matrix<double, 7, 1>& lqr_tau_eq_pin);
private:
    void loadJsonParameters();
    std::string matrixToString(const double* matrix, int rows, int cols);
    void print_params(std::string string_in);
};

