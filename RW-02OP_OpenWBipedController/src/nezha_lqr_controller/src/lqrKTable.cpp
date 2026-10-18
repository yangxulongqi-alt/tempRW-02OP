#include "nezha_lqr_controller/lqrKTable.hpp"
#include <nlohmann/json.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>
#include <iostream>
using json = nlohmann::json;


lqrKTable::lqrKTable() {
    // 构造函数
    loadJsonParameters();   // 加载LQR模型参数
}


/* ********************************************************************************
 *                     _________   _______    _______    _       
 *                     \__    _/  (  ____ \  (  ___  )  ( (    /|
 *                        )  (    | (    \/  | (   ) |  |  \  ( |
 *                        |  |    | (_____   | |   | |  |   \ | |
 *                        |  |    (_____  )  | |   | |  | (\ \) |
 *                        |  |          ) |  | |   | |  | | \   |
 *                     |\_)  )    /\____) |  | (___) |  | )  \  |
 *                     (____/     \_______)  (_______)  |/    )_)
********************************************************************************** */ 
// 从指定路径读取
void readMatrixFrom(
    const nlohmann::json& root, 
    const std::string& path, 
    double* matrix, 
    int rows, 
    int cols
) {
    try {
        // 使用JSON指针访问指定路径
        const auto& matrixJson = root.at(nlohmann::json::json_pointer(path));
        
        // 验证数组维度
        if (!matrixJson.is_array() || matrixJson.size() != static_cast<size_t>(rows)) {
            throw std::runtime_error(
                "Invalid matrix format (expected " + std::to_string(rows) + " rows)"
            );
        }
        
        // 填充矩阵
        for (int i = 0; i < rows; ++i) {
            const auto& row = matrixJson[i];
            
            if (!row.is_array() || row.size() != static_cast<size_t>(cols)) {
                throw std::runtime_error(
                    "Row " + std::to_string(i) + " has invalid size (expected " + 
                    std::to_string(cols) + " columns)"
                );
            }
            
            for (int j = 0; j < cols; ++j) {
                // 计算一维数组中的位置：i*cols + j
                matrix[i * cols + j] = row[j].get<double>();
            }
        }
        
    } catch (const nlohmann::json::out_of_range& e) {
        throw std::runtime_error("JSON path does not exist: " + path);
    } catch (const std::exception& e) {
        throw std::runtime_error("Error reading matrix: " + std::string(e.what()));
    }
}

void lqrKTable::loadJsonParameters() {
    // 获取json文件路径
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("nezha_lqr_controller");
    std::string config_file_path = package_share_dir + "/config/lqr_paramsCD.json";

    std::ifstream json_in(config_file_path);
    if (!json_in.is_open()) {
        throw std::runtime_error("无法打开配置文件: " + config_file_path);
    }

    // 2. 参数读取
    json j;
    json_in >> j;     // j["R"], j["mass"]["head"]

    readMatrixFrom(j, "/fit_coeffs/Ks_coeffs", &lqrK_coeffs[0][0], 24, 5);
    readMatrixFrom(j, "/fit_coeffs/lqr_q_eq_task_coeffs", &lqr_q_eq_task_coeffs[0][0], 4, 5);
    readMatrixFrom(j, "/fit_coeffs/lqr_dq_eq_task_coeffs", &lqr_dq_eq_task_coeffs[0][0], 4, 5);
    readMatrixFrom(j, "/fit_coeffs/lqr_q_eq_pin_coeffs", &lqr_q_eq_pin_coeffs[0][0], 7, 5);
    readMatrixFrom(j, "/fit_coeffs/lqr_dq_eq_pin_coeffs", &lqr_dq_eq_pin_coeffs[0][0], 7, 5);
    readMatrixFrom(j, "/fit_coeffs/lqr_tau_eq_pin_coeffs", &lqr_tau_eq_pin_coeffs[0][0], 7, 5);
    R_wheel = j["R"];
    m_head  = j["mass"]["head"];
    m_abad  = j["mass"]["abad"];
    m_thigh = j["mass"]["thigh"];
    m_shank = j["mass"]["shank"];
    m_wheel = j["mass"]["wheel"];
    l1 = j["l1"];
    l2 = j["l2"];
    for(int i=0; i<3; i++){
        vec_1l(i, 0) = j["vec_1l"][i].get<double>();
        vec_2l(i, 0) = j["vec_2l"][i].get<double>();
        vec_1r(i, 0) = j["vec_1r"][i].get<double>();
        vec_2r(i, 0) = j["vec_2r"][i].get<double>();
    }

    print_params(matrixToString(&lqr_q_eq_task_coeffs[0][0], 4, 5));
}

void lqrKTable::print_params(std::string string_in) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(4) << string_in;  // 设置精度为4位小数
    std::cout << "\x1b[32m ---------- params begin ---------- \x1b[0m" << std::endl;
    std::cout<< "lqr_q_eq_task_coeffs: " << ss.str() <<std::endl;
    std::cout<< "mass: [" << m_head << ", " << m_abad << ", "<< m_thigh << ", "<< m_shank << ", "<< m_wheel << "]"<< std::endl;
    std::cout<< "vec_1l: [" << vec_1l(0)<< ", " << vec_1l(1)<< ", " << vec_1l(2)<< "]" << std::endl;
    std::cout<< "vec_2l: [" << vec_2l(0)<< ", " << vec_2l(1)<< ", " << vec_2l(2)<< "]" << std::endl;
    std::cout << "\x1b[32m ---------- params end ---------- \x1b[0m" << std::endl;
    // RCLCPP_INFO(get_logger(), "LQR K矩阵:\n%s", ss.str().c_str());
}


std::string lqrKTable::matrixToString(const double* matrix, int rows, int cols) {
    int precision = 3;
    int width = 10;
    std::ostringstream oss;
    oss << "Matrix [" << rows << " x " << cols << "]:" << std::endl;
    
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            // 使用固定宽度和右对齐
            oss << std::right << std::setw(width) 
                << std::fixed << std::setprecision(precision)
                << matrix[i * cols + j];
        }
        oss << std::endl;
    }
    
    return oss.str();
}


void lqrKTable::poly_calc_at(double h, Eigen::Matrix<double, 3, 8>& lqrK,
                             Eigen::Matrix<double, 4, 1>& lqr_q_eq_task,
                             Eigen::Matrix<double, 4, 1>& lqr_dq_eq_task,
                             Eigen::Matrix<double, 7, 1>& lqr_q_eq_pin,
                             Eigen::Matrix<double, 7, 1>& lqr_dq_eq_pin,
                             Eigen::Matrix<double, 7, 1>& lqr_tau_eq_pin) {
    double h2 = h*h;
    double h3 = h2*h;
    double h4 = h3*h;
    double* c;
    // Poly4 for K
    for(int i=0; i<3; i++){
        for(int j=0; j<8; j++) {
            c = lqrK_coeffs[i*8 + j];
            lqrK(i, j) = c[0]*h4 + c[1]*h3 + c[2]*h2 + c[3]*h + c[4];
        }
    }
    // Poly4 for task
    for(int i=0; i<4; i++) {
        c = lqr_q_eq_task_coeffs[i];
        lqr_q_eq_task(i) = c[0]*h4 + c[1]*h3 + c[2]*h2 + c[3]*h + c[4];
        c = lqr_dq_eq_task_coeffs[i];
        lqr_dq_eq_task(i) = c[0]*h4 + c[1]*h3 + c[2]*h2 + c[3]*h + c[4];
    }

    // Poly4 for pinocchio state
    for(int i=0; i<7; i++) {
        c = lqr_q_eq_pin_coeffs[i];
        lqr_q_eq_pin(i) = c[0]*h4 + c[1]*h3 + c[2]*h2 + c[3]*h + c[4];
        c = lqr_dq_eq_pin_coeffs[i];
        lqr_dq_eq_pin(i) = c[0]*h4 + c[1]*h3 + c[2]*h2 + c[3]*h + c[4];
        c = lqr_tau_eq_pin_coeffs[i];
        lqr_tau_eq_pin(i) = c[0]*h4 + c[1]*h3 + c[2]*h2 + c[3]*h + c[4];
    }
}

