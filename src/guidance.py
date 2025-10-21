"""
固定翼UAVの誘導則

経路点追従、直線経路追従、旋回飛行などの誘導アルゴリズム
"""

import numpy as np


class WaypointGuidance:
    """経路点追従誘導則"""

    def __init__(self, waypoints, R_min=50.0):
        """
        パラメータ:
            waypoints: 経路点のリスト [[x1, y1, z1], [x2, y2, z2], ...]
            R_min: 経路点への到達判定半径 [m]
        """
        self.waypoints = np.array(waypoints)
        self.R_min = R_min
        self.current_waypoint_index = 0
        self.completed = False

    def get_current_waypoint(self):
        """現在の目標経路点を取得"""
        if self.current_waypoint_index < len(self.waypoints):
            return self.waypoints[self.current_waypoint_index]
        else:
            return self.waypoints[-1]  # 最後の経路点

    def update(self, position):
        """
        経路点の更新

        パラメータ:
            position: 現在位置 [x, y, z]

        戻り値:
            waypoint: 現在の目標経路点
            completed: 全経路点を通過したかどうか
        """
        current_wp = self.get_current_waypoint()

        # 経路点までの距離
        distance = np.linalg.norm(position - current_wp)

        # 経路点に到達したら次の経路点へ
        if distance < self.R_min:
            self.current_waypoint_index += 1
            if self.current_waypoint_index >= len(self.waypoints):
                self.completed = True
                self.current_waypoint_index = len(self.waypoints) - 1

        return self.get_current_waypoint(), self.completed

    def compute_heading_command(self, position):
        """
        目標方位角を計算

        パラメータ:
            position: 現在位置 [x, y, z]

        戻り値:
            psi_c: 目標ヨー角 [rad]
        """
        waypoint = self.get_current_waypoint()

        # 経路点への相対位置
        delta = waypoint - position

        # 目標方位角(北からの角度)
        psi_c = np.arctan2(delta[1], delta[0])

        return psi_c

    def reset(self):
        """誘導則をリセット"""
        self.current_waypoint_index = 0
        self.completed = False


class StraightLineGuidance:
    """直線経路追従誘導則"""

    def __init__(self, start_point, end_point):
        """
        パラメータ:
            start_point: 経路の始点 [x, y, z]
            end_point: 経路の終点 [x, y, z]
        """
        self.start_point = np.array(start_point)
        self.end_point = np.array(end_point)

        # 経路方向の単位ベクトル
        self.path_direction = self.end_point - self.start_point
        self.path_length = np.linalg.norm(self.path_direction)
        if self.path_length > 0:
            self.path_direction = self.path_direction / self.path_length
        else:
            self.path_direction = np.array([1.0, 0.0, 0.0])

    def compute_heading_command(self, position, chi_inf=np.pi/2, k_path=0.05):
        """
        経路追従のための目標方位角を計算(ベクトル場誘導)

        パラメータ:
            position: 現在位置 [x, y, z]
            chi_inf: 最大アプローチ角 [rad]
            k_path: 経路ゲイン

        戻り値:
            psi_c: 目標ヨー角 [rad]
        """
        # 経路への相対位置
        r = position - self.start_point

        # 経路に沿った距離(内積)
        s = np.dot(r, self.path_direction)

        # 経路からの距離(外積のノルム)
        # 2Dの場合の簡易計算
        path_dir_2d = self.path_direction[0:2]
        r_2d = r[0:2]

        # 経路への垂直成分
        e_py = np.cross(np.append(path_dir_2d, 0), np.append(r_2d, 0))[2]

        # 経路方向の角度
        chi_q = np.arctan2(self.path_direction[1], self.path_direction[0])

        # アプローチ角
        chi_d = chi_q - chi_inf * (2/np.pi) * np.arctan(k_path * e_py)

        return chi_d

    def compute_crosstrack_error(self, position):
        """
        経路からのクロストラック誤差を計算

        パラメータ:
            position: 現在位置 [x, y, z]

        戻り値:
            error: クロストラック誤差 [m]
        """
        # 経路への相対位置
        r = position - self.start_point

        # 経路に垂直な成分(クロストラック誤差)
        s = np.dot(r, self.path_direction)
        r_perpendicular = r - s * self.path_direction

        return np.linalg.norm(r_perpendicular)


class OrbitGuidance:
    """旋回飛行誘導則"""

    def __init__(self, center, radius, direction='CW'):
        """
        パラメータ:
            center: 旋回中心 [x, y, z]
            radius: 旋回半径 [m]
            direction: 旋回方向 ('CW': 時計回り, 'CCW': 反時計回り)
        """
        self.center = np.array(center)
        self.radius = radius
        self.direction = direction

    def compute_heading_command(self, position, k_orbit=2.0):
        """
        旋回飛行のための目標方位角を計算

        パラメータ:
            position: 現在位置 [x, y, z]
            k_orbit: 旋回ゲイン

        戻り値:
            psi_c: 目標ヨー角 [rad]
        """
        # 旋回中心への相対位置(2D)
        d = position[0:2] - self.center[0:2]
        distance = np.linalg.norm(d)

        if distance < 0.1:
            return 0.0

        # 旋回中心からの角度
        angle_from_center = np.arctan2(d[1], d[0])

        # 旋回方向
        if self.direction == 'CW':
            lambda_val = 1
        else:  # CCW
            lambda_val = -1

        # 目標方位角
        psi_c = angle_from_center + lambda_val * np.pi/2 + \
                lambda_val * np.arctan(k_orbit * (distance - self.radius) / self.radius)

        return psi_c

    def compute_radius_error(self, position):
        """
        目標旋回半径からの誤差を計算

        パラメータ:
            position: 現在位置 [x, y, z]

        戻り値:
            error: 半径誤差 [m]
        """
        d = position[0:2] - self.center[0:2]
        distance = np.linalg.norm(d)
        return distance - self.radius


class PathManager:
    """経路管理(複数の誘導則を統合)"""

    def __init__(self):
        """経路管理の初期化"""
        self.guidance_mode = None
        self.guidance = None

    def set_waypoint_mode(self, waypoints, R_min=50.0):
        """経路点追従モードに設定"""
        self.guidance_mode = 'waypoint'
        self.guidance = WaypointGuidance(waypoints, R_min)

    def set_straight_line_mode(self, start_point, end_point):
        """直線経路追従モードに設定"""
        self.guidance_mode = 'line'
        self.guidance = StraightLineGuidance(start_point, end_point)

    def set_orbit_mode(self, center, radius, direction='CW'):
        """旋回飛行モードに設定"""
        self.guidance_mode = 'orbit'
        self.guidance = OrbitGuidance(center, radius, direction)

    def compute_guidance_commands(self, position):
        """
        誘導指令を計算

        パラメータ:
            position: 現在位置 [x, y, z]

        戻り値:
            psi_c: 目標ヨー角 [rad]
            h_c: 目標高度 [m] (NED座標系)
        """
        if self.guidance is None:
            return 0.0, position[2]

        if self.guidance_mode == 'waypoint':
            waypoint, completed = self.guidance.update(position)
            psi_c = self.guidance.compute_heading_command(position)
            h_c = waypoint[2]

        elif self.guidance_mode == 'line':
            psi_c = self.guidance.compute_heading_command(position)
            h_c = self.guidance.end_point[2]

        elif self.guidance_mode == 'orbit':
            psi_c = self.guidance.compute_heading_command(position)
            h_c = self.guidance.center[2]

        else:
            psi_c = 0.0
            h_c = position[2]

        return psi_c, h_c


class L1Guidance:
    """
    L1適応誘導則(L1 Adaptive Guidance)

    直線経路および円軌道の追従に使用される適応誘導アルゴリズム。
    Park, Deyst, How (2004) "A New Nonlinear Guidance Logic for Trajectory Tracking"に基づく。
    """

    def __init__(self, L1_distance=None, L1_damping=0.707, L1_period=15.0):
        """
        パラメータ:
            L1_distance: L1距離 [m] (Noneの場合は自動計算)
            L1_damping: L1ダンピング係数 (通常0.5-1.0)
            L1_period: L1周期 [s] (通常10-25)
        """
        self.L1_distance = L1_distance
        self.L1_damping = L1_damping
        self.L1_period = L1_period
        self.gravity = 9.81

    def compute_L1_distance(self, V_a):
        """
        対気速度からL1距離を計算

        パラメータ:
            V_a: 対気速度 [m/s]

        戻り値:
            L1: L1距離 [m]
        """
        if self.L1_distance is not None:
            return self.L1_distance

        # L1 = (1/π) * ζ * T * V_a
        # ζ: damping ratio, T: period
        L1 = (1.0 / np.pi) * self.L1_damping * self.L1_period * V_a
        return L1

    def compute_lateral_acceleration(self, position, V_a, chi, path_type='line',
                                     path_params=None):
        """
        L1誘導則による横方向加速度指令を計算

        パラメータ:
            position: 現在位置 [x, y, z]
            V_a: 対気速度 [m/s]
            chi: 現在のコース角 [rad]
            path_type: 経路タイプ ('line' or 'orbit')
            path_params: 経路パラメータ
                - 'line': {'start': [x,y,z], 'end': [x,y,z]} or {'point': [x,y,z], 'direction': [x,y,z]}
                - 'orbit': {'center': [x,y,z], 'radius': float, 'direction': 'CW' or 'CCW'}

        戻り値:
            a_cmd: 横方向加速度指令 [m/s²]
            eta: 方位角誤差 [rad]
        """
        L1 = self.compute_L1_distance(V_a)

        if path_type == 'line':
            a_cmd, eta = self._compute_line_tracking(position, V_a, chi, L1, path_params)
        elif path_type == 'orbit':
            a_cmd, eta = self._compute_orbit_tracking(position, V_a, chi, L1, path_params)
        else:
            raise ValueError(f"Unknown path type: {path_type}")

        return a_cmd, eta

    def _compute_line_tracking(self, position, V_a, chi, L1, path_params):
        """直線経路追従のための横方向加速度を計算"""
        # 経路パラメータの取得
        if 'start' in path_params and 'end' in path_params:
            start_point = np.array(path_params['start'])
            end_point = np.array(path_params['end'])
            path_direction = end_point - start_point
            path_direction = path_direction / np.linalg.norm(path_direction)
            path_point = start_point
        elif 'point' in path_params and 'direction' in path_params:
            path_point = np.array(path_params['point'])
            path_direction = np.array(path_params['direction'])
            path_direction = path_direction / np.linalg.norm(path_direction)
        else:
            raise ValueError("Invalid path_params for line tracking")

        # 経路の方位角
        chi_q = np.arctan2(path_direction[1], path_direction[0])

        # 経路への相対位置(2D)
        r = position[0:2] - path_point[0:2]

        # クロストラック誤差(経路に垂直な距離)
        # e_py = r × q (外積のz成分)
        e_py = r[0] * path_direction[1] - r[1] * path_direction[0]

        # 方位角誤差
        eta = self._wrap_angle(chi_q - chi)

        # L1法則による横方向加速度
        # a_cmd = 2 * V_a² / L1 * sin(η)
        # ここで sin(η) ≈ η + e_py/L1 (小角度近似)
        a_cmd = 2.0 * V_a**2 / L1 * np.sin(np.arctan2(-e_py, L1))

        return a_cmd, eta

    def _compute_orbit_tracking(self, position, V_a, chi, L1, path_params):
        """円軌道追従のための横方向加速度を計算"""
        center = np.array(path_params['center'])
        radius = path_params['radius']
        direction = path_params.get('direction', 'CW')

        # 旋回中心への相対位置(2D)
        d_vec = position[0:2] - center[0:2]
        d = np.linalg.norm(d_vec)

        if d < 0.1:
            return 0.0, 0.0

        # 旋回方向
        lambda_val = 1 if direction == 'CW' else -1

        # 中心からの角度
        angle_from_center = np.arctan2(d_vec[1], d_vec[0])

        # 目標コース角(接線方向)
        chi_q = angle_from_center + lambda_val * np.pi / 2

        # 方位角誤差
        eta = self._wrap_angle(chi_q - chi)

        # 半径誤差
        e_r = d - radius

        # L1法則による横方向加速度
        # 円軌道の場合: a_cmd = 2 * V_a² / L1 * sin(arctan2(-e_r, L1)) + V_a² / R
        a_lateral = 2.0 * V_a**2 / L1 * np.sin(np.arctan2(lambda_val * e_r, L1))
        a_centripetal = lambda_val * V_a**2 / radius
        a_cmd = a_lateral + a_centripetal

        return a_cmd, eta

    def compute_roll_command(self, a_cmd, V_a, phi_limit=np.pi/4):
        """
        横方向加速度指令からロール角指令を計算

        パラメータ:
            a_cmd: 横方向加速度指令 [m/s²]
            V_a: 対気速度 [m/s]
            phi_limit: 最大バンク角制限 [rad]

        戻り値:
            phi_c: 目標ロール角 [rad]
        """
        # 協調旋回の関係: a = g * tan(φ)
        phi_c = np.arctan(a_cmd / self.gravity)

        # バンク角制限
        phi_c = np.clip(phi_c, -phi_limit, phi_limit)

        return phi_c

    def _wrap_angle(self, angle):
        """角度を±πの範囲に正規化"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle


class CoordinatedTurnGuidance:
    """協調旋回誘導(バンク角を使った旋回)"""

    def __init__(self, V_a=25.0):
        """
        パラメータ:
            V_a: 対気速度 [m/s]
        """
        self.V_a = V_a

    def compute_roll_command(self, psi, psi_c, k_psi=0.5):
        """
        方位角追従のためのロール角指令を計算

        パラメータ:
            psi: 現在のヨー角 [rad]
            psi_c: 目標ヨー角 [rad]
            k_psi: 方位角ゲイン

        戻り値:
            phi_c: 目標ロール角 [rad]
        """
        # 方位角誤差(±πの範囲に正規化)
        psi_error = self.wrap_angle(psi_c - psi)

        # 目標ロール角
        phi_c = k_psi * psi_error

        # バンク角制限(±45度)
        phi_c = np.clip(phi_c, -np.pi/4, np.pi/4)

        return phi_c

    def wrap_angle(self, angle):
        """角度を±πの範囲に正規化"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def compute_turn_radius(self, phi):
        """
        バンク角から旋回半径を計算

        パラメータ:
            phi: ロール角 [rad]

        戻り値:
            R: 旋回半径 [m]
        """
        g = 9.81  # 重力加速度
        if np.abs(np.tan(phi)) < 0.01:
            return np.inf
        return self.V_a**2 / (g * np.tan(phi))


class ProportionalOrbitGuidance:
    """
    比例制御ベースの円軌道誘導

    GPS誤差のある中心座標と半径推定値を使用して、
    半径誤差の比例制御により円旋回を実現します。

    制御則:
        φ_c = φ_ff + K_p * e_r

    ここで:
        φ_ff: フィードフォワード項（定常旋回のバンク角）
        e_r: 半径誤差（実際の距離 - 目標半径）
        K_p: 比例ゲイン
    """

    def __init__(self, K_p=0.05, phi_max=np.pi/4):
        """
        パラメータ:
            K_p: 半径誤差の比例ゲイン [rad/m]
            phi_max: 最大バンク角制限 [rad]
        """
        self.K_p = K_p
        self.phi_max = phi_max
        self.gravity = 9.81  # 重力加速度 [m/s²]

    def compute_roll_command(self, position, center, radius, V_a, direction='CW'):
        """
        比例制御による円旋回のロール角指令を計算

        パラメータ:
            position: 現在位置 [x, y, z]
            center: 旋回中心（GPS推定値、誤差を含む） [x, y, z]
            radius: 目標旋回半径（GPS推定値、誤差を含む） [m]
            V_a: 対気速度 [m/s]
            direction: 旋回方向 ('CW': 時計回り, 'CCW': 反時計回り)

        戻り値:
            phi_c: 目標ロール角 [rad]
            e_r: 半径誤差 [m]
            d: 中心からの実際の距離 [m]
        """
        # 旋回方向の符号
        lambda_val = 1 if direction == 'CW' else -1

        # 中心からの距離ベクトル（2D）
        d_vec = position[0:2] - center[0:2]
        d = np.linalg.norm(d_vec)

        # 中心に近すぎる場合の特異点回避
        if d < 0.1:
            return 0.0, 0.0, d

        # 半径誤差
        # 正: 目標半径より外側、負: 目標半径より内側
        e_r = d - radius

        # フィードフォワード項：定常旋回のバンク角
        # 協調旋回: V_a² / (g * R) = g * tan(φ_ff)
        # したがって: φ_ff = arctan(V_a² / (g * R))
        phi_ff = lambda_val * np.arctan(V_a**2 / (self.gravity * radius))

        # 比例制御項：半径誤差に対する補正
        # 外側にいる場合（e_r > 0）→バンク角を増やして内側に
        # 内側にいる場合（e_r < 0）→バンク角を減らして外側に
        phi_p = lambda_val * self.K_p * e_r

        # 合計ロール角指令
        phi_c = phi_ff + phi_p

        # バンク角制限
        phi_c = np.clip(phi_c, -self.phi_max, self.phi_max)

        return phi_c, e_r, d

    def compute_feedforward_roll(self, V_a, radius, direction='CW'):
        """
        定常旋回のためのフィードフォワードロール角を計算

        パラメータ:
            V_a: 対気速度 [m/s]
            radius: 旋回半径 [m]
            direction: 旋回方向 ('CW' or 'CCW')

        戻り値:
            phi_ff: フィードフォワードロール角 [rad]
        """
        lambda_val = 1 if direction == 'CW' else -1
        phi_ff = lambda_val * np.arctan(V_a**2 / (self.gravity * radius))
        return phi_ff

    def analyze_stability(self, V_a, radius):
        """
        線形化システムの安定性を解析

        パラメータ:
            V_a: 対気速度 [m/s]
            radius: 旋回半径 [m]

        戻り値:
            eigenvalue: システムの固有値（負なら安定）
            time_constant: 時定数 [s]
            damping_ratio: 減衰比
        """
        # 線形化システムの解析
        # ドキュメントの安定性解析セクション参照

        # フィードフォワードバンク角
        phi_ff = np.arctan(V_a**2 / (self.gravity * radius))

        # システムパラメータ
        # d(phi)/dt ≈ -a * e_r, a = K_p * V_a
        a = self.K_p * V_a

        # 1次システムの固有値（時定数の逆数）
        # 簡略化モデル: de_r/dt = -a * e_r
        eigenvalue = -a

        # 時定数: τ = 1/a
        time_constant = 1.0 / a if a > 0 else np.inf

        # 1次システムなので減衰比は定義されない（2次系のパラメータ）
        # 便宜上1.0（臨界減衰相当）を返す
        damping_ratio = 1.0

        return eigenvalue, time_constant, damping_ratio

