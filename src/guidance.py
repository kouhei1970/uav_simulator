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
