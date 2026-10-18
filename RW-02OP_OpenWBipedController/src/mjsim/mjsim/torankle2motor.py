"""
Torankle2motor - Python版本
踝关节角度与电机角度转换模块
"""

import numpy as np
from typing import Tuple, Callable
from dataclasses import dataclass


@dataclass
class MotorAngles:
    """电机角度结构"""
    upper_motor: float
    lower_motor: float


def calculate_motor_angles(pitch_rad: float, roll_rad: float) -> MotorAngles:
    """
    计算电机角度的函数
    
    Args:
        pitch_rad: 俯仰角（弧度）
        roll_rad: 滚转角（弧度）
    
    Returns:
        MotorAngles: 包含上下电机角度的结构
    """
    pitch_rad = pitch_rad
    roll_rad = -roll_rad
    
    # 连杆长度参数
    l_PQ = 295.0  # 连杆PQ的长度
    l_NP = 52.0   # 曲柄NP的长度
    
    # 定义向量
    r_UQ = np.array([-62, 50, 0], dtype=np.float64)
    s_N = np.array([145, 50, 0], dtype=np.float64)
    s_U = np.array([440, 0, 10], dtype=np.float64)
    sr_UN = s_N - s_U
    
    # 下电机参数
    l_pq = 160.0  # 连杆pq的长度
    l_np = 52.0   # 曲柄np的长度
    
    r_Uq = np.array([-62, -50, 0], dtype=np.float64)
    s_n = np.array([280, -50, 0], dtype=np.float64)
    sr_Un = s_n - s_U
    
    # 创建俯仰角的旋转矩阵
    pitch_matrix = np.array([
        [np.sin(pitch_rad), 0, -np.cos(pitch_rad)],
        [0, 1, 0],
        [np.cos(pitch_rad), 0, np.sin(pitch_rad)]
    ])
    
    # 创建滚转角的旋转矩阵
    roll_matrix = np.array([
        [1, 0, 0],
        [0, np.cos(roll_rad), -np.sin(roll_rad)],
        [0, np.sin(roll_rad), np.cos(roll_rad)]
    ])
    
    # 两个旋转矩阵相乘
    rotation_matrix = pitch_matrix @ roll_matrix
    
    sr_UQ = rotation_matrix @ r_UQ
    sr_Uq = rotation_matrix @ r_Uq
    
    # 定义sr_NQ和sr_nq
    sr_NQ = sr_UQ - sr_UN
    sr_nq = sr_Uq - sr_Un
    
    X, Y, Z = sr_NQ[0], sr_NQ[1], sr_NQ[2]
    x, y, z = sr_nq[0], sr_nq[1], sr_nq[2]
    
    B = np.sqrt(X*X + Z*Z)
    C = np.sqrt(l_PQ*l_PQ - Y*Y)
    
    b = np.sqrt(x*x + z*z)
    c = np.sqrt(l_pq*l_pq - y*y)
    
    # 计算电机角度
    upper_motor = -((np.arctan2(-Z, X) + 
                    np.arccos((l_NP * l_NP + B * B - C * C) / (2 * l_NP * B))) - 1.57)
    lower_motor = ((np.arctan2(-z, x) + 
                   np.arccos((l_np * l_np + b * b - c * c) / (2 * l_np * b))) - 1.57)
    
    return MotorAngles(upper_motor, lower_motor)


def partial_derivative(func: Callable[[float, float], float], 
                       pitch: float, 
                       roll: float, 
                       pitch_derivative: bool) -> float:
    """
    计算偏导数的函数
    
    Args:
        func: 返回double的函数
        pitch: 俯仰角
        roll: 滚转角
        pitch_derivative: True表示对pitch求偏导，False表示对roll求偏导
    
    Returns:
        偏导数值
    """
    h = 0.0001  # 微小的步长
    if pitch_derivative:
        return (func(pitch + h, roll) - func(pitch - h, roll)) / (2 * h)
    else:
        return (func(pitch, roll + h) - func(pitch, roll - h)) / (2 * h)


def calculate_jacobian(pitch_rad: float, roll_rad: float) -> np.ndarray:
    """
    计算雅可比矩阵的函数
    
    Args:
        pitch_rad: 俯仰角（弧度）
        roll_rad: 滚转角（弧度）
    
    Returns:
        2x2 雅可比矩阵
    """
    pitch = pitch_rad
    roll = roll_rad
    
    jacobian = np.zeros((2, 2))
    
    # J[0, 0]: upperMotor对pitch的偏导
    jacobian[0, 0] = partial_derivative(
        lambda p, r: calculate_motor_angles(p, r).upper_motor,
        pitch, roll, True
    )
    
    # J[0, 1]: upperMotor对roll的偏导
    jacobian[0, 1] = partial_derivative(
        lambda p, r: calculate_motor_angles(p, r).upper_motor,
        pitch, roll, False
    )
    
    # J[1, 0]: lowerMotor对pitch的偏导
    jacobian[1, 0] = partial_derivative(
        lambda p, r: calculate_motor_angles(p, r).lower_motor,
        pitch, roll, True
    )
    
    # J[1, 1]: lowerMotor对roll的偏导
    jacobian[1, 1] = partial_derivative(
        lambda p, r: calculate_motor_angles(p, r).lower_motor,
        pitch, roll, False
    )
    
    return jacobian


def calculate_pseudo_inverse_transpose(jacobian: np.ndarray) -> np.ndarray:
    """
    计算伪逆转置矩阵
    公式：J_pseudo = J^T * (J * J^T)^-1
    
    Args:
        jacobian: 雅可比矩阵
    
    Returns:
        伪逆转置矩阵
    """
    pseudo_inverse_transpose = jacobian.T @ np.linalg.inv(jacobian @ jacobian.T)
    return pseudo_inverse_transpose


def calculate_left_pseudo_inverse(jacobian: np.ndarray) -> np.ndarray:
    """
    计算左伪逆矩阵
    公式：J_pseudo = (J^T * J)^-1 * J^T
    
    Args:
        jacobian: 雅可比矩阵
    
    Returns:
        左伪逆矩阵
    """
    left_pseudo_inverse = np.linalg.inv(jacobian.T @ jacobian) @ jacobian.T
    return left_pseudo_inverse


def newton_raphson(target_upper: float, 
                   target_lower: float, 
                   initial_pitch: float, 
                   initial_roll: float) -> Tuple[float, float]:
    """
    牛顿-拉弗森方法求解 pitch 和 roll
    
    Args:
        target_upper: 目标上电机角度
        target_lower: 目标下电机角度
        initial_pitch: 初始俯仰角
        initial_roll: 初始滚转角
    
    Returns:
        (pitch, roll): 求解得到的俯仰角和滚转角
    """
    pitch = initial_pitch
    roll = initial_roll
    max_iterations = 100
    tolerance = 1e-6
    iter_cnt = 0
    
    for i in range(max_iterations):
        angles = calculate_motor_angles(pitch, roll)
        F1 = angles.upper_motor - target_upper
        F2 = angles.lower_motor - target_lower
        iter_cnt = i
        
        # 计算雅可比矩阵元素
        J11 = partial_derivative(
            lambda p, r: calculate_motor_angles(p, r).upper_motor,
            pitch, roll, True
        )
        J12 = partial_derivative(
            lambda p, r: calculate_motor_angles(p, r).upper_motor,
            pitch, roll, False
        )
        J21 = partial_derivative(
            lambda p, r: calculate_motor_angles(p, r).lower_motor,
            pitch, roll, True
        )
        J22 = partial_derivative(
            lambda p, r: calculate_motor_angles(p, r).lower_motor,
            pitch, roll, False
        )
        
        # 计算雅可比矩阵的行列式
        det = J11 * J22 - J12 * J21
        if abs(det) < 1e-6:
            print("Determinant is too close to zero.")
            break
        
        # 计算雅可比矩阵的逆
        inv_J11 = J22 / det
        inv_J12 = -J12 / det
        inv_J21 = -J21 / det
        inv_J22 = J11 / det
        
        # 更新 pitch 和 roll
        delta_pitch = -(inv_J11 * F1 + inv_J12 * F2)
        delta_roll = -(inv_J21 * F1 + inv_J22 * F2)
        pitch += delta_pitch
        roll += delta_roll
        
        # 检查收敛
        if np.sqrt(delta_pitch * delta_pitch + delta_roll * delta_roll) < tolerance:
            break
    
    return pitch, roll, iter_cnt


# 示例用法
if __name__ == "__main__":
    # 示例1: 计算给定pitch和roll下的电机角度
    pitch_deg = 20.0
    roll_deg = 0.0
    pitch_rad = np.deg2rad(pitch_deg)
    roll_rad = np.deg2rad(roll_deg)
    
    motor_angles = calculate_motor_angles(pitch_rad, roll_rad)
    print(f"Pitch: {pitch_deg}°, Roll: {roll_deg}°")
    print(f"Upper Motor Angle: {motor_angles.upper_motor:.6f} rad ({np.rad2deg(motor_angles.upper_motor):.4f}°)")
    print(f"Lower Motor Angle: {motor_angles.lower_motor:.6f} rad ({np.rad2deg(motor_angles.lower_motor):.4f}°)")
    
    # 示例2: 计算雅可比矩阵并进行力矩转换
    print("\n" + "="*50)
    torque = np.array([20.0, 15.0])  # pitch力矩为20.0，roll力矩为15.0
    
    jacobian = calculate_jacobian(pitch_rad, roll_rad)
    print(f"\nJacobian Matrix:\n{jacobian}")
    
    pseudo_inverse_transpose = calculate_pseudo_inverse_transpose(jacobian)
    motor_torques = pseudo_inverse_transpose @ torque
    
    print(f"\nInput Torques (pitch, roll): {torque}")
    print(f"Upper Motor Torque: {motor_torques[0]:.6f}")
    print(f"Lower Motor Torque: {motor_torques[1]:.6f}")
    
    # 示例3: 使用牛顿-拉弗森方法反向求解
    print("\n" + "="*50)
    target_upper = motor_angles.upper_motor
    target_lower = motor_angles.lower_motor
    
    solved_pitch, solved_roll = newton_raphson(target_upper, target_lower, 0.0, 0.0)
    print(f"\nNewton-Raphson Solution:")
    print(f"Target Upper Motor: {target_upper:.6f} rad")
    print(f"Target Lower Motor: {target_lower:.6f} rad")
    print(f"Solved Pitch: {solved_pitch:.6f} rad ({np.rad2deg(solved_pitch):.4f}°)")
    print(f"Solved Roll: {solved_roll:.6f} rad ({np.rad2deg(solved_roll):.4f}°)")
