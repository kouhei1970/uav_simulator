"""
固定翼UAVの制御則

姿勢制御、高度制御、速度制御のPIDコントローラ
"""

import numpy as np


class PIDController:
    """基本的なPIDコントローラ"""

    def __init__(self, kp=0.0, ki=0.0, kd=0.0, limit=None):
        """
        パラメータ:
            kp: 比例ゲイン
            ki: 積分ゲイン
            kd: 微分ゲイン
            limit: 出力の制限 (min, max)のタプル
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit

        # 内部状態
        self.integrator = 0.0
        self.error_prev = 0.0
        self.differentiator = 0.0

    def update(self, error, dt):
        """
        PID制御の更新

        パラメータ:
            error: 誤差
            dt: 時間ステップ [s]

        戻り値:
            u: 制御入力
        """
        # 比例項
        proportional = self.kp * error

        # 積分項
        self.integrator += error * dt
        integral = self.ki * self.integrator

        # 微分項
        if dt > 0:
            self.differentiator = (error - self.error_prev) / dt
        derivative = self.kd * self.differentiator

        # 制御入力
        u = proportional + integral + derivative

        # 飽和制限
        if self.limit is not None:
            u = np.clip(u, self.limit[0], self.limit[1])

        self.error_prev = error

        return u

    def reset(self):
        """PIDの内部状態をリセット"""
        self.integrator = 0.0
        self.error_prev = 0.0
        self.differentiator = 0.0


class AttitudeController:
    """姿勢制御器(ロール、ピッチ、ヨー)"""

    def __init__(self):
        """姿勢制御器の初期化"""
        # ロール角制御用PID
        self.roll_controller = PIDController(
            kp=0.5,
            ki=0.0,
            kd=0.1,
            limit=(-0.5, 0.5)  # エルロン制限 [rad]
        )

        # ロールレート制御用PID
        self.roll_rate_controller = PIDController(
            kp=0.2,
            ki=0.0,
            kd=0.05,
            limit=(-0.5, 0.5)
        )

        # ピッチ角制御用PID
        self.pitch_controller = PIDController(
            kp=0.5,
            ki=0.0,
            kd=0.1,
            limit=(-0.5, 0.5)  # エレベータ制限 [rad]
        )

        # ピッチレート制御用PID
        self.pitch_rate_controller = PIDController(
            kp=0.2,
            ki=0.0,
            kd=0.05,
            limit=(-0.5, 0.5)
        )

        # ヨーレート制御用PID
        self.yaw_rate_controller = PIDController(
            kp=0.1,
            ki=0.0,
            kd=0.02,
            limit=(-0.5, 0.5)  # ラダー制限 [rad]
        )

        # サイドスリップ補正用PID
        self.sideslip_controller = PIDController(
            kp=0.5,
            ki=0.0,
            kd=0.0,
            limit=(-0.5, 0.5)
        )

    def compute_control(self, uav, phi_c, theta_c, dt):
        """
        姿勢制御入力を計算

        パラメータ:
            uav: FixedWingUAVオブジェクト
            phi_c: 目標ロール角 [rad]
            theta_c: 目標ピッチ角 [rad]
            dt: 時間ステップ [s]

        戻り値:
            delta_a: エルロン偏角 [rad]
            delta_e: エレベータ偏角 [rad]
            delta_r: ラダー偏角 [rad]
        """
        # 現在の姿勢と角速度
        phi, theta, psi = uav.get_attitude()
        p, q, r = uav.get_angular_velocity()

        # ロール角制御(カスケード制御)
        # Normalize angle error to ±π for proper control
        phi_error = self._wrap_angle(phi_c - phi)
        p_c = self.roll_controller.update(phi_error, dt)  # 目標ロールレート
        p_error = p_c - p
        delta_a = self.roll_rate_controller.update(p_error, dt)

        # ピッチ角制御(カスケード制御)
        theta_error = theta_c - theta
        q_c = self.pitch_controller.update(theta_error, dt)  # 目標ピッチレート
        q_error = q_c - q
        delta_e = self.pitch_rate_controller.update(q_error, dt)

        # サイドスリップ補正(横滑り角をゼロに保つ)
        beta = uav.get_sideslip_angle()
        delta_r = self.sideslip_controller.update(-beta, dt)

        return delta_a, delta_e, delta_r

    def _wrap_angle(self, angle):
        """Wrap angle to ±π range"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def reset(self):
        """全てのPIDコントローラをリセット"""
        self.roll_controller.reset()
        self.roll_rate_controller.reset()
        self.pitch_controller.reset()
        self.pitch_rate_controller.reset()
        self.yaw_rate_controller.reset()
        self.sideslip_controller.reset()


class CascadeAttitudeController:
    """完全なカスケードPID姿勢制御器

    ロール、ピッチ、ヨーの3軸すべてでカスケード制御を実装:
    - アウターループ: 角度制御
    - インナーループ: 角速度制御
    """

    def __init__(self):
        """カスケード姿勢制御器の初期化"""
        # ロール角制御用PID (アウターループ)
        self.roll_angle_controller = PIDController(
            kp=5.0,
            ki=0.2,
            kd=0.8,
            limit=(-2.0, 2.0)  # ロールレート指令制限 [rad/s]
        )

        # ロールレート制御用PID (インナーループ)
        self.roll_rate_controller = PIDController(
            kp=0.15,
            ki=0.01,
            kd=0.02,
            limit=(-0.4, 0.4)  # エルロン制限 [rad]
        )

        # ピッチ角制御用PID (アウターループ)
        self.pitch_angle_controller = PIDController(
            kp=1.2,
            ki=0.03,
            kd=0.25,
            limit=(-1.0, 1.0)  # ピッチレート指令制限 [rad/s]
        )

        # ピッチレート制御用PID (インナーループ)
        self.pitch_rate_controller = PIDController(
            kp=0.04,
            ki=0.0,
            kd=0.008,
            limit=(-0.12, 0.12)  # エレベータ制限 [rad]
        )

        # ヨー角制御用PID (アウターループ)
        self.yaw_angle_controller = PIDController(
            kp=3.0,
            ki=0.1,
            kd=0.6,
            limit=(-2.0, 2.0)  # ヨーレート指令制限 [rad/s]
        )

        # ヨーレート制御用PID (インナーループ)
        self.yaw_rate_controller = PIDController(
            kp=0.12,
            ki=0.01,
            kd=0.025,
            limit=(-0.5, 0.5)  # ラダー制限 [rad]
        )

    def compute_control(self, uav, phi_c, theta_c, psi_c, dt):
        """
        カスケードPID姿勢制御入力を計算

        パラメータ:
            uav: FixedWingUAVオブジェクト
            phi_c: 目標ロール角 [rad]
            theta_c: 目標ピッチ角 [rad]
            psi_c: 目標ヨー角 [rad]
            dt: 時間ステップ [s]

        戻り値:
            delta_a: エルロン偏角 [rad]
            delta_e: エレベータ偏角 [rad]
            delta_r: ラダー偏角 [rad]
            p_c: 目標ロールレート [rad/s]
            q_c: 目標ピッチレート [rad/s]
            r_c: 目標ヨーレート [rad/s]
        """
        # 現在の姿勢と角速度
        phi, theta, psi = uav.get_attitude()
        p, q, r = uav.get_angular_velocity()

        # ロール角カスケード制御
        # アウターループ: 角度誤差 → 目標角速度
        phi_error = self._wrap_angle(phi_c - phi)
        p_c = self.roll_angle_controller.update(phi_error, dt)

        # インナーループ: 角速度誤差 → 舵角
        p_error = p_c - p
        delta_a = self.roll_rate_controller.update(p_error, dt)

        # ピッチ角カスケード制御
        # アウターループ: 角度誤差 → 目標角速度
        theta_error = self._wrap_angle(theta_c - theta)
        q_c = self.pitch_angle_controller.update(theta_error, dt)

        # インナーループ: 角速度誤差 → 舵角
        q_error = q_c - q
        delta_e = self.pitch_rate_controller.update(q_error, dt)

        # ヨー角カスケード制御
        # アウターループ: 角度誤差 → 目標角速度
        psi_error = self._wrap_angle(psi_c - psi)
        r_c = self.yaw_angle_controller.update(psi_error, dt)

        # インナーループ: 角速度誤差 → 舵角
        r_error = r_c - r
        delta_r = self.yaw_rate_controller.update(r_error, dt)

        return delta_a, delta_e, delta_r, p_c, q_c, r_c

    def _wrap_angle(self, angle):
        """角度を±π範囲にラップ"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def reset(self):
        """全てのPIDコントローラをリセット"""
        self.roll_angle_controller.reset()
        self.roll_rate_controller.reset()
        self.pitch_angle_controller.reset()
        self.pitch_rate_controller.reset()
        self.yaw_angle_controller.reset()
        self.yaw_rate_controller.reset()


class AltitudeController:
    """高度制御器"""

    def __init__(self):
        """高度制御器の初期化"""
        # 高度制御用PID
        self.altitude_controller = PIDController(
            kp=0.05,
            ki=0.01,
            kd=0.02,
            limit=(-np.pi/4, np.pi/4)  # ピッチ角制限
        )

    def compute_pitch_command(self, uav, h_c, dt):
        """
        高度制御のためのピッチ角指令を計算

        パラメータ:
            uav: FixedWingUAVオブジェクト
            h_c: 目標高度 [m] (負の値、NED座標系)
            dt: 時間ステップ [s]

        戻り値:
            theta_c: 目標ピッチ角 [rad]
        """
        # 現在の高度
        h = uav.get_position()[2]

        # 高度誤差
        h_error = h_c - h

        # ピッチ角指令
        theta_c = self.altitude_controller.update(h_error, dt)

        return theta_c

    def reset(self):
        """PIDコントローラをリセット"""
        self.altitude_controller.reset()


class AirspeedController:
    """対気速度制御器"""

    def __init__(self):
        """対気速度制御器の初期化"""
        # 速度制御用PID
        self.airspeed_controller = PIDController(
            kp=0.1,
            ki=0.02,
            kd=0.01,
            limit=(0.0, 1.0)  # スロットル制限
        )

        # ピッチ角による速度制御
        self.airspeed_pitch_controller = PIDController(
            kp=0.05,
            ki=0.01,
            kd=0.02,
            limit=(-np.pi/6, np.pi/6)
        )

    def compute_throttle_command(self, uav, Va_c, dt):
        """
        速度制御のためのスロットル指令を計算

        パラメータ:
            uav: FixedWingUAVオブジェクト
            Va_c: 目標対気速度 [m/s]
            dt: 時間ステップ [s]

        戻り値:
            delta_t: スロットル [0-1]
        """
        # 現在の対気速度
        Va = uav.get_airspeed()

        # 速度誤差
        Va_error = Va_c - Va

        # スロットル指令
        delta_t = self.airspeed_controller.update(Va_error, dt)

        return delta_t

    def compute_pitch_command(self, uav, Va_c, dt):
        """
        速度制御のためのピッチ角指令を計算(代替方法)

        パラメータ:
            uav: FixedWingUAVオブジェクト
            Va_c: 目標対気速度 [m/s]
            dt: 時間ステップ [s]

        戻り値:
            theta_c: 目標ピッチ角 [rad]
        """
        # 現在の対気速度
        Va = uav.get_airspeed()

        # 速度誤差
        Va_error = Va_c - Va

        # ピッチ角指令(速度が遅い場合は機首下げ)
        theta_c = -self.airspeed_pitch_controller.update(Va_error, dt)

        return theta_c

    def reset(self):
        """PIDコントローラをリセット"""
        self.airspeed_controller.reset()
        self.airspeed_pitch_controller.reset()


class TotalEnergyController:
    """全エネルギー制御(TECS: Total Energy Control System)"""

    def __init__(self):
        """TECS制御器の初期化"""
        # エネルギー誤差制御用PID
        self.energy_controller = PIDController(
            kp=0.05,
            ki=0.01,
            kd=0.02,
            limit=(0.0, 1.0)
        )

        # エネルギー配分制御用PID
        self.energy_distribution_controller = PIDController(
            kp=0.05,
            ki=0.01,
            kd=0.02,
            limit=(-np.pi/4, np.pi/4)
        )

    def compute_control(self, uav, h_c, Va_c, dt):
        """
        全エネルギー制御入力を計算

        パラメータ:
            uav: FixedWingUAVオブジェクト
            h_c: 目標高度 [m]
            Va_c: 目標対気速度 [m/s]
            dt: 時間ステップ [s]

        戻り値:
            delta_t: スロットル [0-1]
            theta_c: 目標ピッチ角 [rad]
        """
        # 現在の状態
        h = uav.get_position()[2]
        Va = uav.get_airspeed()
        m = uav.params['mass']
        g = uav.params['gravity']

        # 全エネルギー(運動エネルギー + 位置エネルギー)
        E = 0.5 * m * Va**2 - m * g * h
        E_c = 0.5 * m * Va_c**2 - m * g * h_c
        E_error = E_c - E

        # エネルギー配分(高度と速度のバランス)
        L = -h + Va**2 / (2 * g)
        L_c = -h_c + Va_c**2 / (2 * g)
        L_error = L_c - L

        # スロットルでエネルギー誤差を制御
        delta_t = self.energy_controller.update(E_error, dt)

        # ピッチ角でエネルギー配分を制御
        theta_c = self.energy_distribution_controller.update(L_error, dt)

        return delta_t, theta_c

    def reset(self):
        """PIDコントローラをリセット"""
        self.energy_controller.reset()
        self.energy_distribution_controller.reset()
