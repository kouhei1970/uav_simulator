"""
固定翼UAVの6自由度動力学モデル

北東下(NED)座標系を使用した完全な非線形運動方程式
"""

import numpy as np
from scipy.integrate import ode


class FixedWingUAV:
    """固定翼UAVの動力学モデル"""

    def __init__(self, params=None):
        """
        パラメータ:
            params: 機体パラメータの辞書(Noneの場合はデフォルト値を使用)
        """
        if params is None:
            params = self._default_params()

        self.params = params

        # 状態変数の初期化
        # [x, y, z, u, v, w, phi, theta, psi, p, q, r]
        self.state = np.zeros(12)

        # 初期状態の設定(トリム状態)
        self.state[3] = 25.0  # u: 前進速度 [m/s]
        self.state[5] = 0.0   # w: 下向き速度 [m/s]
        self.state[6] = 0.0   # phi: ロール角 [rad]
        self.state[7] = 0.0   # theta: ピッチ角 [rad]
        self.state[8] = 0.0   # psi: ヨー角 [rad]

        # 制御入力の初期化
        # [delta_e, delta_a, delta_r, delta_t]
        self.control = np.zeros(4)
        self.control[3] = 0.5  # スロットル

        # 外力(風など)
        self.external_forces = np.zeros(3)  # 機体座標系での外力 [N]
        self.external_moments = np.zeros(3)  # 機体座標系での外モーメント [Nm]

        # 積分器の設定
        self.integrator = None
        self.time = 0.0

    def _default_params(self):
        """デフォルトの機体パラメータ"""
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

    def set_state(self, state):
        """状態変数を設定"""
        self.state = np.array(state)

    def set_control(self, control):
        """制御入力を設定"""
        self.control = np.array(control)

    def get_state(self):
        """現在の状態を取得"""
        return self.state.copy()

    def get_position(self):
        """位置を取得 (NED座標系)"""
        return self.state[0:3]

    def get_velocity(self):
        """速度を取得 (機体座標系)"""
        return self.state[3:6]

    def get_attitude(self):
        """姿勢を取得 (オイラー角: phi, theta, psi)"""
        return self.state[6:9]

    def get_angular_velocity(self):
        """角速度を取得 (機体座標系: p, q, r)"""
        return self.state[9:12]

    def get_airspeed(self):
        """対気速度を取得"""
        u, v, w = self.get_velocity()
        return np.sqrt(u**2 + v**2 + w**2)

    def get_angle_of_attack(self):
        """迎角を取得 [rad]"""
        u, v, w = self.get_velocity()
        return np.arctan2(w, u)

    def get_sideslip_angle(self):
        """横滑り角を取得 [rad]"""
        u, v, w = self.get_velocity()
        Va = self.get_airspeed()
        if Va < 0.1:
            return 0.0
        return np.arcsin(v / Va)

    def derivatives(self, t, state, control, forces_moments):
        """
        状態の時間微分を計算

        パラメータ:
            t: 時刻 [s]
            state: 状態変数 [x, y, z, u, v, w, phi, theta, psi, p, q, r]
            control: 制御入力 [delta_e, delta_a, delta_r, delta_t]
            forces_moments: 力とモーメント [Fx, Fy, Fz, L, M, N]

        戻り値:
            state_dot: 状態の時間微分
        """
        # 状態変数の展開
        x, y, z = state[0:3]
        u, v, w = state[3:6]
        phi, theta, psi = state[6:9]
        p, q, r = state[9:12]

        # 力とモーメントの展開
        fx, fy, fz = forces_moments[0:3]
        L, M, N = forces_moments[3:6]

        # パラメータ
        m = self.params['mass']
        g = self.params['gravity']

        # 位置の微分(NED座標系)
        # 機体座標系からNED座標系への変換行列
        c_phi = np.cos(phi)
        s_phi = np.sin(phi)
        c_theta = np.cos(theta)
        s_theta = np.sin(theta)
        c_psi = np.cos(psi)
        s_psi = np.sin(psi)

        # 回転行列
        x_dot = (c_theta * c_psi) * u + \
                (s_phi * s_theta * c_psi - c_phi * s_psi) * v + \
                (c_phi * s_theta * c_psi + s_phi * s_psi) * w

        y_dot = (c_theta * s_psi) * u + \
                (s_phi * s_theta * s_psi + c_phi * c_psi) * v + \
                (c_phi * s_theta * s_psi - s_phi * c_psi) * w

        z_dot = (s_theta) * u + \
                (-s_phi * c_theta) * v + \
                (-c_phi * c_theta) * w

        # 速度の微分(機体座標系)
        u_dot = r * v - q * w - g * s_theta + fx / m
        v_dot = p * w - r * u + g * c_theta * s_phi + fy / m
        w_dot = q * u - p * v + g * c_theta * c_phi + fz / m

        # 姿勢の微分
        phi_dot = p + s_phi * np.tan(theta) * q + c_phi * np.tan(theta) * r
        theta_dot = c_phi * q - s_phi * r
        psi_dot = s_phi / c_theta * q + c_phi / c_theta * r

        # 角速度の微分
        p_dot = self.params['Gamma1'] * p * q - self.params['Gamma2'] * q * r + \
                self.params['Gamma3'] * L + self.params['Gamma4'] * N

        q_dot = self.params['Gamma5'] * p * r - self.params['Gamma6'] * (p**2 - r**2) + M / self.params['Jy']

        r_dot = self.params['Gamma7'] * p * q - self.params['Gamma1'] * q * r + \
                self.params['Gamma4'] * L + self.params['Gamma8'] * N

        # 状態微分をまとめる
        state_dot = np.array([
            x_dot, y_dot, z_dot,
            u_dot, v_dot, w_dot,
            phi_dot, theta_dot, psi_dot,
            p_dot, q_dot, r_dot
        ])

        return state_dot

    def update(self, dt, aerodynamic_forces_moments):
        """
        状態を更新(4次のルンゲクッタ法)

        パラメータ:
            dt: 時間ステップ [s]
            aerodynamic_forces_moments: 空力による力とモーメント
        """
        # 外力も含める
        forces_moments = aerodynamic_forces_moments.copy()
        forces_moments[0:3] += self.external_forces
        forces_moments[3:6] += self.external_moments

        # RK4積分
        k1 = self.derivatives(self.time, self.state, self.control, forces_moments)
        k2 = self.derivatives(self.time + dt/2, self.state + dt/2 * k1, self.control, forces_moments)
        k3 = self.derivatives(self.time + dt/2, self.state + dt/2 * k2, self.control, forces_moments)
        k4 = self.derivatives(self.time + dt, self.state + dt * k3, self.control, forces_moments)

        self.state = self.state + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
        self.time += dt

        return self.state

    def rotation_matrix_body_to_ned(self):
        """機体座標系からNED座標系への回転行列"""
        phi, theta, psi = self.get_attitude()

        c_phi = np.cos(phi)
        s_phi = np.sin(phi)
        c_theta = np.cos(theta)
        s_theta = np.sin(theta)
        c_psi = np.cos(psi)
        s_psi = np.sin(psi)

        R = np.array([
            [c_theta * c_psi, s_phi * s_theta * c_psi - c_phi * s_psi, c_phi * s_theta * c_psi + s_phi * s_psi],
            [c_theta * s_psi, s_phi * s_theta * s_psi + c_phi * c_psi, c_phi * s_theta * s_psi - s_phi * c_psi],
            [s_theta, -s_phi * c_theta, -c_phi * c_theta]
        ])

        return R
