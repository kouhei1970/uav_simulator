"""
固定翼UAVの機体パラメータ定義

複数の機体モデルを提供
"""

import numpy as np


def get_small_uav_params():
    """
    小型UAV（目標機体）
    全幅: 1.6m
    質量: 1.7kg

    典型的な小型固定翼UAVの特性
    """
    params = {
        # 質量特性
        'mass': 1.7,           # 質量 [kg]
        'Jx': 0.082,           # X軸周りの慣性モーメント [kg*m^2]
        'Jy': 0.105,           # Y軸周りの慣性モーメント [kg*m^2]
        'Jz': 0.180,           # Z軸周りの慣性モーメント [kg*m^2]
        'Jxz': 0.015,          # XZ平面の慣性乗積 [kg*m^2]

        # 幾何特性
        'S_wing': 0.28,        # 主翼面積 [m^2]
        'b': 1.6,              # 翼幅 [m]
        'c': 0.175,            # 平均翼弦長 [m]

        # 推進特性
        'S_prop': 0.0201,      # プロペラ面積 [m^2] (直径16cm想定)
        'k_motor': 60.0,       # モーター定数
        'k_T_P': 0.0,          # プロペラ推力係数
        'k_Omega': 0.0,        # プロペラ回転数係数

        # 飛行性能
        'V_cruise': 15.0,      # 巡航速度 [m/s]

        # 環境
        'rho': 1.225,          # 空気密度 [kg/m^3]
        'gravity': 9.81,       # 重力加速度 [m/s^2]
    }

    # 慣性の計算用の定数
    params['Gamma'] = params['Jx'] * params['Jz'] - params['Jxz']**2
    params['Gamma1'] = (params['Jxz'] * (params['Jx'] - params['Jy'] + params['Jz'])) / params['Gamma']
    params['Gamma2'] = (params['Jz'] * (params['Jz'] - params['Jy']) + params['Jxz']**2) / params['Gamma']
    params['Gamma3'] = params['Jz'] / params['Gamma']
    params['Gamma4'] = params['Jxz'] / params['Gamma']
    params['Gamma5'] = (params['Jz'] - params['Jx']) / params['Jy']
    params['Gamma6'] = params['Jxz'] / params['Jy']
    params['Gamma7'] = ((params['Jx'] - params['Jy']) * params['Jx'] + params['Jxz']**2) / params['Gamma']
    params['Gamma8'] = params['Jx'] / params['Gamma']

    return params


def get_small_uav_aero_params():
    """
    小型UAV用の空力パラメータ

    全幅1.6m、質量1.7kgの機体に適した係数
    """
    params = {
        # 揚力係数
        'C_L_0': 0.30,          # 基本揚力係数
        'C_L_alpha': 4.00,      # 迎角に対する揚力係数微係数 [1/rad]
        'C_L_q': 0.0,           # ピッチレートに対する揚力係数微係数
        'C_L_delta_e': -0.40,   # エレベータに対する揚力係数微係数 [1/rad]

        # 抗力係数
        'C_D_0': 0.028,         # 基本抗力係数
        'C_D_alpha': 0.25,      # 迎角に対する抗力係数微係数
        'C_D_q': 0.0,           # ピッチレートに対する抗力係数微係数
        'C_D_delta_e': 0.0,     # エレベータに対する抗力係数微係数

        # 横力係数
        'C_Y_0': 0.0,           # 基本横力係数
        'C_Y_beta': -0.90,      # 横滑り角に対する横力係数微係数 [1/rad]
        'C_Y_p': 0.0,           # ロールレートに対する横力係数微係数
        'C_Y_r': 0.0,           # ヨーレートに対する横力係数微係数
        'C_Y_delta_a': 0.0,     # エルロンに対する横力係数微係数
        'C_Y_delta_r': -0.20,   # ラダーに対する横力係数微係数 [1/rad]

        # ローリングモーメント係数
        'C_l_0': 0.0,           # 基本ローリングモーメント係数
        'C_l_beta': -0.15,      # 横滑り角に対するローリングモーメント係数微係数 [1/rad]
        'C_l_p': -0.30,         # ロールレートに対するローリングモーメント係数微係数
        'C_l_r': 0.18,          # ヨーレートに対するローリングモーメント係数微係数
        'C_l_delta_a': 0.12,    # エルロンに対するローリングモーメント係数微係数 [1/rad]
        'C_l_delta_r': 0.10,    # ラダーに対するローリングモーメント係数微係数 [1/rad]

        # ピッチングモーメント係数
        'C_m_0': -0.025,        # 基本ピッチングモーメント係数
        'C_m_alpha': -0.50,     # 迎角に対するピッチングモーメント係数微係数 [1/rad]
        'C_m_q': -4.0,          # ピッチレートに対するピッチングモーメント係数微係数
        'C_m_delta_e': 0.55,    # エレベータに対するピッチングモーメント係数微係数 [1/rad]

        # ヨーイングモーメント係数
        'C_n_0': 0.0,           # 基本ヨーイングモーメント係数
        'C_n_beta': 0.30,       # 横滑り角に対するヨーイングモーメント係数微係数 [1/rad]
        'C_n_p': 0.02,          # ロールレートに対するヨーイングモーメント係数微係数
        'C_n_r': -0.40,         # ヨーレートに対するヨーイングモーメント係数微係数
        'C_n_delta_a': 0.05,    # エルロンに対するヨーイングモーメント係数微係数 [1/rad]
        'C_n_delta_r': -0.04,   # ラダーに対するヨーイングモーメント係数微係数 [1/rad]

        # プロペラ係数
        'C_prop': 1.0,          # プロペラ効率係数
        'k_T_P': 0.0,           # 推力係数
        'k_Omega': 0.0,         # 回転数係数
    }
    return params


def get_medium_uav_params():
    """
    中型UAV（デフォルト）
    全幅: 2.9m
    質量: 11.0kg

    RascalベースのUAV
    """
    params = {
        # 質量特性
        'mass': 11.0,           # 質量 [kg]
        'Jx': 0.824,           # X軸周りの慣性モーメント [kg*m^2]
        'Jy': 1.135,           # Y軸周りの慣性モーメント [kg*m^2]
        'Jz': 1.759,           # Z軸周りの慣性モーメント [kg*m^2]
        'Jxz': 0.120,          # XZ平面の慣性乗積 [kg*m^2]

        # 幾何特性
        'S_wing': 0.55,        # 主翼面積 [m^2]
        'b': 2.90,             # 翼幅 [m]
        'c': 0.19,             # 平均翼弦長 [m]

        # 推進特性
        'S_prop': 0.0314,      # プロペラ面積 [m^2]
        'k_motor': 80.0,       # モーター定数
        'k_T_P': 0.0,          # プロペラ推力係数
        'k_Omega': 0.0,        # プロペラ回転数係数

        # 飛行性能
        'V_cruise': 25.0,      # 巡航速度 [m/s]

        # 環境
        'rho': 1.225,          # 空気密度 [kg/m^3]
        'gravity': 9.81,       # 重力加速度 [m/s^2]
    }

    # 慣性の計算用の定数
    params['Gamma'] = params['Jx'] * params['Jz'] - params['Jxz']**2
    params['Gamma1'] = (params['Jxz'] * (params['Jx'] - params['Jy'] + params['Jz'])) / params['Gamma']
    params['Gamma2'] = (params['Jz'] * (params['Jz'] - params['Jy']) + params['Jxz']**2) / params['Gamma']
    params['Gamma3'] = params['Jz'] / params['Gamma']
    params['Gamma4'] = params['Jxz'] / params['Gamma']
    params['Gamma5'] = (params['Jz'] - params['Jx']) / params['Jy']
    params['Gamma6'] = params['Jxz'] / params['Jy']
    params['Gamma7'] = ((params['Jx'] - params['Jy']) * params['Jx'] + params['Jxz']**2) / params['Gamma']
    params['Gamma8'] = params['Jx'] / params['Gamma']

    return params


def get_medium_uav_aero_params():
    """中型UAV用の空力パラメータ"""
    params = {
        # 揚力係数
        'C_L_0': 0.28,          # 基本揚力係数
        'C_L_alpha': 3.45,      # 迎角に対する揚力係数微係数 [1/rad]
        'C_L_q': 0.0,           # ピッチレートに対する揚力係数微係数
        'C_L_delta_e': -0.36,   # エレベータに対する揚力係数微係数 [1/rad]

        # 抗力係数
        'C_D_0': 0.03,          # 基本抗力係数
        'C_D_alpha': 0.30,      # 迎角に対する抗力係数微係数
        'C_D_q': 0.0,           # ピッチレートに対する抗力係数微係数
        'C_D_delta_e': 0.0,     # エレベータに対する抗力係数微係数

        # 横力係数
        'C_Y_0': 0.0,           # 基本横力係数
        'C_Y_beta': -0.98,      # 横滑り角に対する横力係数微係数 [1/rad]
        'C_Y_p': 0.0,           # ロールレートに対する横力係数微係数
        'C_Y_r': 0.0,           # ヨーレートに対する横力係数微係数
        'C_Y_delta_a': 0.0,     # エルロンに対する横力係数微係数
        'C_Y_delta_r': -0.17,   # ラダーに対する横力係数微係数 [1/rad]

        # ローリングモーメント係数
        'C_l_0': 0.0,           # 基本ローリングモーメント係数
        'C_l_beta': -0.12,      # 横滑り角に対するローリングモーメント係数微係数 [1/rad]
        'C_l_p': -0.26,         # ロールレートに対するローリングモーメント係数微係数
        'C_l_r': 0.14,          # ヨーレートに対するローリングモーメント係数微係数
        'C_l_delta_a': 0.08,    # エルロンに対するローリングモーメント係数微係数 [1/rad]
        'C_l_delta_r': 0.105,   # ラダーに対するローリングモーメント係数微係数 [1/rad]

        # ピッチングモーメント係数
        'C_m_0': -0.02338,      # 基本ピッチングモーメント係数
        'C_m_alpha': -0.38,     # 迎角に対するピッチングモーメント係数微係数 [1/rad]
        'C_m_q': -3.6,          # ピッチレートに対するピッチングモーメント係数微係数
        'C_m_delta_e': 0.5,     # エレベータに対するピッチングモーメント係数微係数 [1/rad]

        # ヨーイングモーメント係数
        'C_n_0': 0.0,           # 基本ヨーイングモーメント係数
        'C_n_beta': 0.25,       # 横滑り角に対するヨーイングモーメント係数微係数 [1/rad]
        'C_n_p': 0.022,         # ロールレートに対するヨーイングモーメント係数微係数
        'C_n_r': -0.35,         # ヨーレートに対するヨーイングモーメント係数微係数
        'C_n_delta_a': 0.06,    # エルロンに対するヨーイングモーメント係数微係数 [1/rad]
        'C_n_delta_r': -0.032,  # ラダーに対するヨーイングモーメント係数微係数 [1/rad]

        # プロペラ係数
        'C_prop': 1.0,          # プロペラ効率係数
        'k_T_P': 0.0,           # 推力係数
        'k_Omega': 0.0,         # 回転数係数
    }
    return params


def get_micro_uav_params():
    """
    超小型UAV
    全幅: 0.8m
    質量: 0.5kg

    屋内用の超小型機
    """
    params = {
        # 質量特性
        'mass': 0.5,           # 質量 [kg]
        'Jx': 0.015,           # X軸周りの慣性モーメント [kg*m^2]
        'Jy': 0.020,           # Y軸周りの慣性モーメント [kg*m^2]
        'Jz': 0.030,           # Z軸周りの慣性モーメント [kg*m^2]
        'Jxz': 0.003,          # XZ平面の慣性乗積 [kg*m^2]

        # 幾何特性
        'S_wing': 0.12,        # 主翼面積 [m^2]
        'b': 0.8,              # 翼幅 [m]
        'c': 0.15,             # 平均翼弦長 [m]

        # 推進特性
        'S_prop': 0.0079,      # プロペラ面積 [m^2] (直径10cm想定)
        'k_motor': 40.0,       # モーター定数
        'k_T_P': 0.0,          # プロペラ推力係数
        'k_Omega': 0.0,        # プロペラ回転数係数

        # 飛行性能
        'V_cruise': 12.0,      # 巡航速度 [m/s]

        # 環境
        'rho': 1.225,          # 空気密度 [kg/m^3]
        'gravity': 9.81,       # 重力加速度 [m/s^2]
    }

    # 慣性の計算用の定数
    params['Gamma'] = params['Jx'] * params['Jz'] - params['Jxz']**2
    params['Gamma1'] = (params['Jxz'] * (params['Jx'] - params['Jy'] + params['Jz'])) / params['Gamma']
    params['Gamma2'] = (params['Jz'] * (params['Jz'] - params['Jy']) + params['Jxz']**2) / params['Gamma']
    params['Gamma3'] = params['Jz'] / params['Gamma']
    params['Gamma4'] = params['Jxz'] / params['Gamma']
    params['Gamma5'] = (params['Jz'] - params['Jx']) / params['Jy']
    params['Gamma6'] = params['Jxz'] / params['Jy']
    params['Gamma7'] = ((params['Jx'] - params['Jy']) * params['Jx'] + params['Jxz']**2) / params['Gamma']
    params['Gamma8'] = params['Jx'] / params['Gamma']

    return params


def get_micro_uav_aero_params():
    """超小型UAV用の空力パラメータ"""
    params = {
        # 揚力係数
        'C_L_0': 0.35,          # 基本揚力係数
        'C_L_alpha': 4.50,      # 迎角に対する揚力係数微係数 [1/rad]
        'C_L_q': 0.0,           # ピッチレートに対する揚力係数微係数
        'C_L_delta_e': -0.45,   # エレベータに対する揚力係数微係数 [1/rad]

        # 抗力係数
        'C_D_0': 0.035,         # 基本抗力係数
        'C_D_alpha': 0.20,      # 迎角に対する抗力係数微係数
        'C_D_q': 0.0,           # ピッチレートに対する抗力係数微係数
        'C_D_delta_e': 0.0,     # エレベータに対する抗力係数微係数

        # 横力係数
        'C_Y_0': 0.0,           # 基本横力係数
        'C_Y_beta': -0.80,      # 横滑り角に対する横力係数微係数 [1/rad]
        'C_Y_p': 0.0,           # ロールレートに対する横力係数微係数
        'C_Y_r': 0.0,           # ヨーレートに対する横力係数微係数
        'C_Y_delta_a': 0.0,     # エルロンに対する横力係数微係数
        'C_Y_delta_r': -0.25,   # ラダーに対する横力係数微係数 [1/rad]

        # ローリングモーメント係数
        'C_l_0': 0.0,           # 基本ローリングモーメント係数
        'C_l_beta': -0.18,      # 横滑り角に対するローリングモーメント係数微係数 [1/rad]
        'C_l_p': -0.35,         # ロールレートに対するローリングモーメント係数微係数
        'C_l_r': 0.20,          # ヨーレートに対するローリングモーメント係数微係数
        'C_l_delta_a': 0.15,    # エルロンに対するローリングモーメント係数微係数 [1/rad]
        'C_l_delta_r': 0.12,    # ラダーに対するローリングモーメント係数微係数 [1/rad]

        # ピッチングモーメント係数
        'C_m_0': -0.030,        # 基本ピッチングモーメント係数
        'C_m_alpha': -0.60,     # 迎角に対するピッチングモーメント係数微係数 [1/rad]
        'C_m_q': -5.0,          # ピッチレートに対するピッチングモーメント係数微係数
        'C_m_delta_e': 0.60,    # エレベータに対するピッチングモーメント係数微係数 [1/rad]

        # ヨーイングモーメント係数
        'C_n_0': 0.0,           # 基本ヨーイングモーメント係数
        'C_n_beta': 0.35,       # 横滑り角に対するヨーイングモーメント係数微係数 [1/rad]
        'C_n_p': 0.03,          # ロールレートに対するヨーイングモーメント係数微係数
        'C_n_r': -0.45,         # ヨーレートに対するヨーイングモーメント係数微係数
        'C_n_delta_a': 0.08,    # エルロンに対するヨーイングモーメント係数微係数 [1/rad]
        'C_n_delta_r': -0.05,   # ラダーに対するヨーイングモーメント係数微係数 [1/rad]

        # プロペラ係数
        'C_prop': 1.0,          # プロペラ効率係数
        'k_T_P': 0.0,           # 推力係数
        'k_Omega': 0.0,         # 回転数係数
    }
    return params


def get_large_uav_params():
    """
    大型UAV
    全幅: 4.0m
    質量: 20.0kg

    長時間飛行用の大型機
    """
    params = {
        # 質量特性
        'mass': 20.0,          # 質量 [kg]
        'Jx': 1.5,             # X軸周りの慣性モーメント [kg*m^2]
        'Jy': 2.0,             # Y軸周りの慣性モーメント [kg*m^2]
        'Jz': 3.2,             # Z軸周りの慣性モーメント [kg*m^2]
        'Jxz': 0.20,           # XZ平面の慣性乗積 [kg*m^2]

        # 幾何特性
        'S_wing': 1.0,         # 主翼面積 [m^2]
        'b': 4.0,              # 翼幅 [m]
        'c': 0.25,             # 平均翼弦長 [m]

        # 推進特性
        'S_prop': 0.0503,      # プロペラ面積 [m^2] (直径25cm想定)
        'k_motor': 100.0,      # モーター定数
        'k_T_P': 0.0,          # プロペラ推力係数
        'k_Omega': 0.0,        # プロペラ回転数係数

        # 飛行性能
        'V_cruise': 30.0,      # 巡航速度 [m/s]

        # 環境
        'rho': 1.225,          # 空気密度 [kg/m^3]
        'gravity': 9.81,       # 重力加速度 [m/s^2]
    }

    # 慣性の計算用の定数
    params['Gamma'] = params['Jx'] * params['Jz'] - params['Jxz']**2
    params['Gamma1'] = (params['Jxz'] * (params['Jx'] - params['Jy'] + params['Jz'])) / params['Gamma']
    params['Gamma2'] = (params['Jz'] * (params['Jz'] - params['Jy']) + params['Jxz']**2) / params['Gamma']
    params['Gamma3'] = params['Jz'] / params['Gamma']
    params['Gamma4'] = params['Jxz'] / params['Gamma']
    params['Gamma5'] = (params['Jz'] - params['Jx']) / params['Jy']
    params['Gamma6'] = params['Jxz'] / params['Jy']
    params['Gamma7'] = ((params['Jx'] - params['Jy']) * params['Jx'] + params['Jxz']**2) / params['Gamma']
    params['Gamma8'] = params['Jx'] / params['Gamma']

    return params


def get_large_uav_aero_params():
    """大型UAV用の空力パラメータ"""
    params = {
        # 揚力係数
        'C_L_0': 0.25,          # 基本揚力係数
        'C_L_alpha': 3.20,      # 迎角に対する揚力係数微係数 [1/rad]
        'C_L_q': 0.0,           # ピッチレートに対する揚力係数微係数
        'C_L_delta_e': -0.32,   # エレベータに対する揚力係数微係数 [1/rad]

        # 抗力係数
        'C_D_0': 0.025,         # 基本抗力係数
        'C_D_alpha': 0.35,      # 迎角に対する抗力係数微係数
        'C_D_q': 0.0,           # ピッチレートに対する抗力係数微係数
        'C_D_delta_e': 0.0,     # エレベータに対する抗力係数微係数

        # 横力係数
        'C_Y_0': 0.0,           # 基本横力係数
        'C_Y_beta': -1.05,      # 横滑り角に対する横力係数微係数 [1/rad]
        'C_Y_p': 0.0,           # ロールレートに対する横力係数微係数
        'C_Y_r': 0.0,           # ヨーレートに対する横力係数微係数
        'C_Y_delta_a': 0.0,     # エルロンに対する横力係数微係数
        'C_Y_delta_r': -0.15,   # ラダーに対する横力係数微係数 [1/rad]

        # ローリングモーメント係数
        'C_l_0': 0.0,           # 基本ローリングモーメント係数
        'C_l_beta': -0.10,      # 横滑り角に対するローリングモーメント係数微係数 [1/rad]
        'C_l_p': -0.22,         # ロールレートに対するローリングモーメント係数微係数
        'C_l_r': 0.12,          # ヨーレートに対するローリングモーメント係数微係数
        'C_l_delta_a': 0.06,    # エルロンに対するローリングモーメント係数微係数 [1/rad]
        'C_l_delta_r': 0.09,    # ラダーに対するローリングモーメント係数微係数 [1/rad]

        # ピッチングモーメント係数
        'C_m_0': -0.020,        # 基本ピッチングモーメント係数
        'C_m_alpha': -0.35,     # 迎角に対するピッチングモーメント係数微係数 [1/rad]
        'C_m_q': -3.2,          # ピッチレートに対するピッチングモーメント係数微係数
        'C_m_delta_e': 0.45,    # エレベータに対するピッチングモーメント係数微係数 [1/rad]

        # ヨーイングモーメント係数
        'C_n_0': 0.0,           # 基本ヨーイングモーメント係数
        'C_n_beta': 0.22,       # 横滑り角に対するヨーイングモーメント係数微係数 [1/rad]
        'C_n_p': 0.018,         # ロールレートに対するヨーイングモーメント係数微係数
        'C_n_r': -0.30,         # ヨーレートに対するヨーイングモーメント係数微係数
        'C_n_delta_a': 0.05,    # エルロンに対するヨーイングモーメント係数微係数 [1/rad]
        'C_n_delta_r': -0.028,  # ラダーに対するヨーイングモーメント係数微係数 [1/rad]

        # プロペラ係数
        'C_prop': 1.0,          # プロペラ効率係数
        'k_T_P': 0.0,           # 推力係数
        'k_Omega': 0.0,         # 回転数係数
    }
    return params


def get_small_uav_slightly_unstable_aero_params():
    """
    小型UAV（やや不安定）の空力パラメータ

    ピッチとロールがやや不安定
    """
    params = get_small_uav_aero_params().copy()

    # ロール：やや不安定
    params['C_l_beta'] = 0.03   # 正の値で不安定傾向
    params['C_l_p'] = -0.15     # ダンピング減少

    # ピッチ：やや不安定
    params['C_m_alpha'] = 0.05  # 正の値で不安定傾向
    params['C_m_q'] = -1.5      # ダンピング減少

    # ヨー：中立
    params['C_n_beta'] = 0.05
    params['C_n_r'] = -0.15

    return params


def get_small_uav_unstable_aero_params():
    """
    小型UAV（不安定）の空力パラメータ

    全軸が不安定
    """
    params = get_small_uav_aero_params().copy()

    # ロール：不安定
    params['C_l_beta'] = 0.10   # 強い不安定傾向
    params['C_l_p'] = -0.05     # 弱いダンピング

    # ピッチ：不安定
    params['C_m_alpha'] = 0.15  # 強い不安定傾向
    params['C_m_q'] = -0.5      # 弱いダンピング

    # ヨー：不安定
    params['C_n_beta'] = -0.15  # 負の値で不安定
    params['C_n_r'] = -0.05     # 弱いダンピング

    return params


# 便利関数
def get_aircraft_params(aircraft_type='small'):
    """
    機体タイプに応じたパラメータを取得

    パラメータ:
        aircraft_type: 'small', 'medium', 'micro', 'large',
                      'small_slightly_unstable', 'small_unstable'

    戻り値:
        (aircraft_params, aero_params): 機体パラメータと空力パラメータのタプル
    """
    if aircraft_type == 'small':
        return get_small_uav_params(), get_small_uav_aero_params()
    elif aircraft_type == 'small_slightly_unstable':
        return get_small_uav_params(), get_small_uav_slightly_unstable_aero_params()
    elif aircraft_type == 'small_unstable':
        return get_small_uav_params(), get_small_uav_unstable_aero_params()
    elif aircraft_type == 'medium':
        return get_medium_uav_params(), get_medium_uav_aero_params()
    elif aircraft_type == 'micro':
        return get_micro_uav_params(), get_micro_uav_aero_params()
    elif aircraft_type == 'large':
        return get_large_uav_params(), get_large_uav_aero_params()
    else:
        raise ValueError(f"Unknown aircraft type: {aircraft_type}")


def print_aircraft_info(aircraft_type='small'):
    """機体情報を表示"""
    aircraft_params, aero_params = get_aircraft_params(aircraft_type)

    print(f"=== {aircraft_type.upper()} UAV 機体情報 ===")
    print(f"質量: {aircraft_params['mass']:.2f} kg")
    print(f"翼幅: {aircraft_params['b']:.2f} m")
    print(f"翼面積: {aircraft_params['S_wing']:.3f} m^2")
    print(f"平均翼弦長: {aircraft_params['c']:.3f} m")
    print(f"アスペクト比: {aircraft_params['b']**2 / aircraft_params['S_wing']:.2f}")
    print(f"プロペラ面積: {aircraft_params['S_prop']:.4f} m^2")
    print(f"巡航速度: {aircraft_params['V_cruise']:.1f} m/s")

    # 安定性情報
    print(f"安定性微係数:")
    print(f"  C_l_beta (ロール): {aero_params['C_l_beta']:.3f} (負で安定)")
    print(f"  C_m_alpha (ピッチ): {aero_params['C_m_alpha']:.3f} (負で安定)")
    print(f"  C_n_beta (ヨー): {aero_params['C_n_beta']:.3f} (正で安定)")
    print()


if __name__ == "__main__":
    # 全ての機体情報を表示
    print("基本機体モデル:")
    for aircraft_type in ['micro', 'small', 'medium', 'large']:
        print_aircraft_info(aircraft_type)

    print("\n安定性バリエーション（小型UAV）:")
    for aircraft_type in ['small', 'small_slightly_unstable', 'small_unstable']:
        print_aircraft_info(aircraft_type)
