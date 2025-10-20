"""
シミュレーション結果の可視化ツール

3D軌跡、状態プロット、アニメーションなど
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


class SimulationVisualizer:
    """シミュレーション結果の可視化"""

    def __init__(self):
        """可視化の初期化"""
        self.time_history = []
        self.state_history = []
        self.control_history = []

    def add_data(self, time, state, control):
        """
        データを追加

        パラメータ:
            time: 時刻 [s]
            state: 状態ベクトル
            control: 制御入力ベクトル
        """
        self.time_history.append(time)
        self.state_history.append(state.copy())
        self.control_history.append(control.copy())

    def plot_3d_trajectory(self, waypoints=None):
        """
        3D軌跡をプロット

        パラメータ:
            waypoints: 経路点のリスト(オプション)
        """
        if len(self.state_history) == 0:
            print("データがありません")
            return

        states = np.array(self.state_history)
        x = states[:, 0]
        y = states[:, 1]
        z = -states[:, 2]  # 高度を正の値で表示

        fig = plt.figure(figsize=(10, 8))
        ax = fig.add_subplot(111, projection='3d')

        # 軌跡をプロット
        ax.plot(x, y, z, 'b-', linewidth=2, label='軌跡')
        ax.plot([x[0]], [y[0]], [z[0]], 'go', markersize=10, label='開始点')
        ax.plot([x[-1]], [y[-1]], [z[-1]], 'ro', markersize=10, label='終了点')

        # 経路点をプロット
        if waypoints is not None:
            wp = np.array(waypoints)
            ax.plot(wp[:, 0], wp[:, 1], -wp[:, 2], 'r*', markersize=15, label='経路点')

        ax.set_xlabel('北 [m]')
        ax.set_ylabel('東 [m]')
        ax.set_zlabel('高度 [m]')
        ax.set_title('3D飛行軌跡')
        ax.legend()
        ax.grid(True)

        plt.tight_layout()
        plt.show()

    def plot_states(self):
        """状態変数の時系列をプロット"""
        if len(self.state_history) == 0:
            print("データがありません")
            return

        times = np.array(self.time_history)
        states = np.array(self.state_history)

        fig, axes = plt.subplots(4, 3, figsize=(15, 12))

        # 位置
        axes[0, 0].plot(times, states[:, 0])
        axes[0, 0].set_ylabel('x [m]')
        axes[0, 0].set_title('北方向位置')
        axes[0, 0].grid(True)

        axes[0, 1].plot(times, states[:, 1])
        axes[0, 1].set_ylabel('y [m]')
        axes[0, 1].set_title('東方向位置')
        axes[0, 1].grid(True)

        axes[0, 2].plot(times, -states[:, 2])
        axes[0, 2].set_ylabel('高度 [m]')
        axes[0, 2].set_title('高度')
        axes[0, 2].grid(True)

        # 速度
        axes[1, 0].plot(times, states[:, 3])
        axes[1, 0].set_ylabel('u [m/s]')
        axes[1, 0].set_title('前進速度')
        axes[1, 0].grid(True)

        axes[1, 1].plot(times, states[:, 4])
        axes[1, 1].set_ylabel('v [m/s]')
        axes[1, 1].set_title('横方向速度')
        axes[1, 1].grid(True)

        axes[1, 2].plot(times, states[:, 5])
        axes[1, 2].set_ylabel('w [m/s]')
        axes[1, 2].set_title('下方向速度')
        axes[1, 2].grid(True)

        # 姿勢
        axes[2, 0].plot(times, np.rad2deg(states[:, 6]))
        axes[2, 0].set_ylabel('φ [deg]')
        axes[2, 0].set_title('ロール角')
        axes[2, 0].grid(True)

        axes[2, 1].plot(times, np.rad2deg(states[:, 7]))
        axes[2, 1].set_ylabel('θ [deg]')
        axes[2, 1].set_title('ピッチ角')
        axes[2, 1].grid(True)

        axes[2, 2].plot(times, np.rad2deg(states[:, 8]))
        axes[2, 2].set_ylabel('ψ [deg]')
        axes[2, 2].set_title('ヨー角')
        axes[2, 2].grid(True)

        # 角速度
        axes[3, 0].plot(times, np.rad2deg(states[:, 9]))
        axes[3, 0].set_ylabel('p [deg/s]')
        axes[3, 0].set_xlabel('時間 [s]')
        axes[3, 0].set_title('ロールレート')
        axes[3, 0].grid(True)

        axes[3, 1].plot(times, np.rad2deg(states[:, 10]))
        axes[3, 1].set_ylabel('q [deg/s]')
        axes[3, 1].set_xlabel('時間 [s]')
        axes[3, 1].set_title('ピッチレート')
        axes[3, 1].grid(True)

        axes[3, 2].plot(times, np.rad2deg(states[:, 11]))
        axes[3, 2].set_ylabel('r [deg/s]')
        axes[3, 2].set_xlabel('時間 [s]')
        axes[3, 2].set_title('ヨーレート')
        axes[3, 2].grid(True)

        plt.tight_layout()
        plt.show()

    def plot_controls(self):
        """制御入力の時系列をプロット"""
        if len(self.control_history) == 0:
            print("データがありません")
            return

        times = np.array(self.time_history)
        controls = np.array(self.control_history)

        fig, axes = plt.subplots(2, 2, figsize=(12, 8))

        # エレベータ
        axes[0, 0].plot(times, np.rad2deg(controls[:, 0]))
        axes[0, 0].set_ylabel('δe [deg]')
        axes[0, 0].set_title('エレベータ')
        axes[0, 0].grid(True)

        # エルロン
        axes[0, 1].plot(times, np.rad2deg(controls[:, 1]))
        axes[0, 1].set_ylabel('δa [deg]')
        axes[0, 1].set_title('エルロン')
        axes[0, 1].grid(True)

        # ラダー
        axes[1, 0].plot(times, np.rad2deg(controls[:, 2]))
        axes[1, 0].set_ylabel('δr [deg]')
        axes[1, 0].set_xlabel('時間 [s]')
        axes[1, 0].set_title('ラダー')
        axes[1, 0].grid(True)

        # スロットル
        axes[1, 1].plot(times, controls[:, 3])
        axes[1, 1].set_ylabel('δt [-]')
        axes[1, 1].set_xlabel('時間 [s]')
        axes[1, 1].set_title('スロットル')
        axes[1, 1].set_ylim([-0.1, 1.1])
        axes[1, 1].grid(True)

        plt.tight_layout()
        plt.show()

    def plot_airdata(self):
        """対気データの時系列をプロット"""
        if len(self.state_history) == 0:
            print("データがありません")
            return

        times = np.array(self.time_history)
        states = np.array(self.state_history)

        # 対気速度、迎角、横滑り角を計算
        Va_list = []
        alpha_list = []
        beta_list = []

        for state in states:
            u, v, w = state[3:6]
            Va = np.sqrt(u**2 + v**2 + w**2)
            alpha = np.arctan2(w, u)
            if Va > 0.1:
                beta = np.arcsin(v / Va)
            else:
                beta = 0.0

            Va_list.append(Va)
            alpha_list.append(alpha)
            beta_list.append(beta)

        Va_arr = np.array(Va_list)
        alpha_arr = np.rad2deg(np.array(alpha_list))
        beta_arr = np.rad2deg(np.array(beta_list))

        fig, axes = plt.subplots(3, 1, figsize=(10, 9))

        # 対気速度
        axes[0].plot(times, Va_arr)
        axes[0].set_ylabel('Va [m/s]')
        axes[0].set_title('対気速度')
        axes[0].grid(True)

        # 迎角
        axes[1].plot(times, alpha_arr)
        axes[1].set_ylabel('α [deg]')
        axes[1].set_title('迎角')
        axes[1].grid(True)

        # 横滑り角
        axes[2].plot(times, beta_arr)
        axes[2].set_ylabel('β [deg]')
        axes[2].set_xlabel('時間 [s]')
        axes[2].set_title('横滑り角')
        axes[2].grid(True)

        plt.tight_layout()
        plt.show()

    def plot_2d_trajectory(self, waypoints=None, orbit_center=None, orbit_radius=None):
        """
        2D軌跡をプロット(上面図)

        パラメータ:
            waypoints: 経路点のリスト(オプション)
            orbit_center: 旋回中心(オプション)
            orbit_radius: 旋回半径(オプション)
        """
        if len(self.state_history) == 0:
            print("データがありません")
            return

        states = np.array(self.state_history)
        x = states[:, 0]
        y = states[:, 1]

        fig, ax = plt.subplots(figsize=(10, 10))

        # 軌跡をプロット
        ax.plot(x, y, 'b-', linewidth=2, label='軌跡')
        ax.plot(x[0], y[0], 'go', markersize=10, label='開始点')
        ax.plot(x[-1], y[-1], 'ro', markersize=10, label='終了点')

        # 経路点をプロット
        if waypoints is not None:
            wp = np.array(waypoints)
            ax.plot(wp[:, 0], wp[:, 1], 'r*', markersize=15, label='経路点')
            # 経路点間を線で結ぶ
            ax.plot(wp[:, 0], wp[:, 1], 'r--', alpha=0.5, linewidth=1)

        # 旋回円をプロット
        if orbit_center is not None and orbit_radius is not None:
            circle = plt.Circle((orbit_center[0], orbit_center[1]), orbit_radius,
                              color='r', fill=False, linestyle='--', linewidth=2, label='目標旋回円')
            ax.add_patch(circle)

        ax.set_xlabel('北 [m]')
        ax.set_ylabel('東 [m]')
        ax.set_title('2D飛行軌跡(上面図)')
        ax.legend()
        ax.grid(True)
        ax.axis('equal')

        plt.tight_layout()
        plt.show()

    def save_data(self, filename):
        """
        データをファイルに保存

        パラメータ:
            filename: 保存するファイル名
        """
        data = {
            'time': np.array(self.time_history),
            'states': np.array(self.state_history),
            'controls': np.array(self.control_history)
        }
        np.savez(filename, **data)
        print(f"データを{filename}に保存しました")

    def load_data(self, filename):
        """
        データをファイルから読み込み

        パラメータ:
            filename: 読み込むファイル名
        """
        data = np.load(filename)
        self.time_history = data['time'].tolist()
        self.state_history = data['states'].tolist()
        self.control_history = data['controls'].tolist()
        print(f"データを{filename}から読み込みました")

    def reset(self):
        """履歴をリセット"""
        self.time_history = []
        self.state_history = []
        self.control_history = []
