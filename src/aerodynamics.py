"""
固定翼UAVの空力モデル

安定微係数と操縦微係数に基づく空力特性のモデル化
"""

import numpy as np


class AerodynamicModel:
    """空力モデル"""

    def __init__(self, params=None, aircraft_type=None):
        """
        パラメータ:
            params: 空力パラメータの辞書
            aircraft_type: 機体タイプ ('small', 'medium', 'micro', 'large')
                         paramsとaircraft_typeの両方が指定された場合はparamsを優先
        """
        if params is None:
            if aircraft_type is not None:
                # config/aircraft_params.pyから空力パラメータを読み込み
                try:
                    import sys
                    import os
                    sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
                    from config.aircraft_params import get_aircraft_params
                    _, params = get_aircraft_params(aircraft_type)
                except ImportError:
                    print(f"Warning: Could not load aircraft type '{aircraft_type}', using default params")
                    params = self._default_aero_params()
            else:
                params = self._default_aero_params()

        self.aero_params = params
        self.aircraft_type = aircraft_type if aircraft_type else 'custom'

    def _default_aero_params(self):
        """デフォルトの空力パラメータ（小型UAV - 安定型）"""
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

            # ローリングモーメント係数（安定）
            'C_l_0': 0.0,           # 基本ローリングモーメント係数
            'C_l_beta': -0.15,      # 横滑り角に対するローリングモーメント係数微係数 [1/rad]（負で安定）
            'C_l_p': -0.30,         # ロールレートに対するローリングモーメント係数微係数（負で安定）
            'C_l_r': 0.18,          # ヨーレートに対するローリングモーメント係数微係数
            'C_l_delta_a': 0.12,    # エルロンに対するローリングモーメント係数微係数 [1/rad]
            'C_l_delta_r': 0.10,    # ラダーに対するローリングモーメント係数微係数 [1/rad]

            # ピッチングモーメント係数（安定）
            'C_m_0': -0.025,        # 基本ピッチングモーメント係数
            'C_m_alpha': -0.50,     # 迎角に対するピッチングモーメント係数微係数 [1/rad]（負で安定）
            'C_m_q': -4.0,          # ピッチレートに対するピッチングモーメント係数微係数（負で安定）
            'C_m_delta_e': 0.55,    # エレベータに対するピッチングモーメント係数微係数 [1/rad]

            # ヨーイングモーメント係数（安定）
            'C_n_0': 0.0,           # 基本ヨーイングモーメント係数
            'C_n_beta': 0.30,       # 横滑り角に対するヨーイングモーメント係数微係数 [1/rad]（正で安定）
            'C_n_p': 0.02,          # ロールレートに対するヨーイングモーメント係数微係数
            'C_n_r': -0.40,         # ヨーレートに対するヨーイングモーメント係数微係数（負で安定）
            'C_n_delta_a': 0.05,    # エルロンに対するヨーイングモーメント係数微係数 [1/rad]
            'C_n_delta_r': -0.04,   # ラダーに対するヨーイングモーメント係数微係数 [1/rad]

            # プロペラ係数
            'C_prop': 1.0,          # プロペラ効率係数
            'k_T_P': 0.0,           # 推力係数
            'k_Omega': 0.0,         # 回転数係数
        }
        return params

    def compute_forces_moments(self, uav, control):
        """
        空力力とモーメントを計算

        パラメータ:
            uav: FixedWingUAVオブジェクト
            control: 制御入力 [delta_e, delta_a, delta_r, delta_t]

        戻り値:
            forces_moments: [Fx, Fy, Fz, L, M, N] (機体座標系)
        """
        # 制御入力
        delta_e = control[0]  # エレベータ
        delta_a = control[1]  # エルロン
        delta_r = control[2]  # ラダー
        delta_t = control[3]  # スロットル

        # 状態変数
        u, v, w = uav.get_velocity()
        p, q, r = uav.get_angular_velocity()

        # 空力パラメータ
        Va = uav.get_airspeed()  # 対気速度
        alpha = uav.get_angle_of_attack()  # 迎角
        beta = uav.get_sideslip_angle()  # 横滑り角

        # 機体パラメータ
        S = uav.params['S_wing']
        b = uav.params['b']
        c = uav.params['c']
        rho = uav.params['rho']

        # 動圧
        q_bar = 0.5 * rho * Va**2

        # 無次元化した角速度
        if Va < 0.1:
            p_bar = 0.0
            q_bar_rate = 0.0
            r_bar = 0.0
        else:
            p_bar = p * b / (2 * Va)
            q_bar_rate = q * c / (2 * Va)
            r_bar = r * b / (2 * Va)

        # 揚力係数
        C_L = (self.aero_params['C_L_0'] +
               self.aero_params['C_L_alpha'] * alpha +
               self.aero_params['C_L_q'] * q_bar_rate +
               self.aero_params['C_L_delta_e'] * delta_e)

        # 抗力係数
        C_D = (self.aero_params['C_D_0'] +
               self.aero_params['C_D_alpha'] * alpha +
               self.aero_params['C_D_q'] * q_bar_rate +
               self.aero_params['C_D_delta_e'] * delta_e)

        # 横力係数
        C_Y = (self.aero_params['C_Y_0'] +
               self.aero_params['C_Y_beta'] * beta +
               self.aero_params['C_Y_p'] * p_bar +
               self.aero_params['C_Y_r'] * r_bar +
               self.aero_params['C_Y_delta_a'] * delta_a +
               self.aero_params['C_Y_delta_r'] * delta_r)

        # ローリングモーメント係数
        C_l = (self.aero_params['C_l_0'] +
               self.aero_params['C_l_beta'] * beta +
               self.aero_params['C_l_p'] * p_bar +
               self.aero_params['C_l_r'] * r_bar +
               self.aero_params['C_l_delta_a'] * delta_a +
               self.aero_params['C_l_delta_r'] * delta_r)

        # ピッチングモーメント係数
        C_m = (self.aero_params['C_m_0'] +
               self.aero_params['C_m_alpha'] * alpha +
               self.aero_params['C_m_q'] * q_bar_rate +
               self.aero_params['C_m_delta_e'] * delta_e)

        # ヨーイングモーメント係数
        C_n = (self.aero_params['C_n_0'] +
               self.aero_params['C_n_beta'] * beta +
               self.aero_params['C_n_p'] * p_bar +
               self.aero_params['C_n_r'] * r_bar +
               self.aero_params['C_n_delta_a'] * delta_a +
               self.aero_params['C_n_delta_r'] * delta_r)

        # 風軸での力
        F_lift = q_bar * S * C_L
        F_drag = q_bar * S * C_D
        F_Y = q_bar * S * C_Y

        # 風軸から機体軸への変換
        cos_alpha = np.cos(alpha)
        sin_alpha = np.sin(alpha)

        # 機体軸での空力力
        fx_aero = -F_drag * cos_alpha + F_lift * sin_alpha
        fy_aero = F_Y
        fz_aero = -F_drag * sin_alpha - F_lift * cos_alpha

        # 推力
        # 簡易的なプロペラモデル
        f_thrust = 0.5 * rho * uav.params['S_prop'] * self.aero_params['C_prop'] * \
                   ((uav.params['k_motor'] * delta_t)**2 - Va**2)

        fx_thrust = f_thrust
        fy_thrust = 0.0
        fz_thrust = 0.0

        # 全体の力
        fx = fx_aero + fx_thrust
        fy = fy_aero + fy_thrust
        fz = fz_aero + fz_thrust

        # モーメント
        L = q_bar * S * b * C_l  # ローリングモーメント
        M = q_bar * S * c * C_m  # ピッチングモーメント
        N = q_bar * S * b * C_n  # ヨーイングモーメント

        forces_moments = np.array([fx, fy, fz, L, M, N])

        return forces_moments

    def get_trim_controls(self, Va, gamma, R=np.inf):
        """
        トリム状態の制御入力を計算

        パラメータ:
            Va: トリム速度 [m/s]
            gamma: 経路角 [rad]
            R: 旋回半径 [m] (np.infで直線飛行)

        戻り値:
            control: トリム制御入力 [delta_e, delta_a, delta_r, delta_t]
        """
        # 簡易的なトリム計算(より詳細な実装が必要な場合は最適化を使用)
        # ここでは基本的な値を返す
        if R == np.inf:
            # 直線飛行
            delta_e = 0.0
            delta_a = 0.0
            delta_r = 0.0
        else:
            # 旋回飛行
            delta_a = 0.1  # 簡易的な値
            delta_r = 0.05

        # スロットル推定
        delta_t = 0.5  # 基本的な値

        return np.array([delta_e, delta_a, delta_r, delta_t])
