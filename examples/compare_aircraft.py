#!/usr/bin/env python3
"""
複数機体の比較シミュレーション

異なる機体タイプの飛行特性を比較
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import matplotlib.pyplot as plt
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel
from src.controller import AttitudeController, AltitudeController, AirspeedController
from src.visualization import SimulationVisualizer


def simulate_aircraft(aircraft_type, target_altitude=-100.0, target_speed=None, sim_time=60.0):
    """
    指定された機体タイプでシミュレーション

    パラメータ:
        aircraft_type: 機体タイプ ('small', 'medium', 'micro', 'large')
        target_altitude: 目標高度 [m]
        target_speed: 目標速度 [m/s] (Noneの場合は機体サイズに応じて自動設定)
        sim_time: シミュレーション時間 [s]

    戻り値:
        viz: SimulationVisualizerオブジェクト
    """
    print(f"\n=== {aircraft_type.upper()} UAV シミュレーション ===")

    # 機体サイズに応じた目標速度の設定
    if target_speed is None:
        speed_map = {
            'micro': 15.0,
            'small': 20.0,
            'medium': 25.0,
            'large': 30.0
        }
        target_speed = speed_map.get(aircraft_type, 25.0)

    # UAVの初期化
    uav = FixedWingUAV(aircraft_type=aircraft_type)
    uav.set_state([0, 0, target_altitude, target_speed, 0, 0, 0, 0, 0, 0, 0, 0])

    # 空力モデル
    aero = AerodynamicModel(aircraft_type=aircraft_type)

    # 制御器の初期化
    attitude_controller = AttitudeController()
    altitude_controller = AltitudeController()
    airspeed_controller = AirspeedController()

    # 可視化
    viz = SimulationVisualizer()

    # 機体情報を表示
    print(f"質量: {uav.params['mass']:.2f} kg")
    print(f"翼幅: {uav.params['b']:.2f} m")
    print(f"翼面積: {uav.params['S_wing']:.3f} m^2")
    print(f"目標速度: {target_speed:.1f} m/s")

    # シミュレーションパラメータ
    dt = 0.01  # 時間ステップ [s]

    # 目標値
    h_c = target_altitude - 50.0  # さらに50m上昇
    Va_c = target_speed

    # シミュレーションループ
    time = 0.0
    step = 0

    while time < sim_time:
        # 現在の状態
        position = uav.get_position()
        phi, theta, psi = uav.get_attitude()
        Va = uav.get_airspeed()

        # 高度制御でピッチ角指令を生成
        theta_c = altitude_controller.compute_pitch_command(uav, h_c, dt)

        # 速度制御でスロットル指令を生成
        delta_t = airspeed_controller.compute_throttle_command(uav, Va_c, dt)

        # 姿勢制御で舵面指令を生成
        phi_c = 0.0  # 水平飛行
        delta_a, delta_e, delta_r = attitude_controller.compute_control(uav, phi_c, theta_c, dt)

        # 制御入力を設定
        control = np.array([delta_e, delta_a, delta_r, delta_t])
        uav.set_control(control)

        # 空力力とモーメントを計算
        forces_moments = aero.compute_forces_moments(uav, control)

        # 状態を更新
        uav.update(dt, forces_moments)

        # データを記録
        if step % 10 == 0:  # 0.1秒ごとに記録
            viz.add_data(time, uav.get_state(), control)

        time += dt
        step += 1

    print(f"最終高度: {-position[2]:.1f} m")
    print(f"最終速度: {Va:.1f} m/s")

    return viz


def main():
    print("=== 複数機体の比較シミュレーション ===")

    # シミュレーション時間
    sim_time = 60.0

    # 各機体タイプでシミュレーション
    aircraft_types = ['micro', 'small', 'medium', 'large']
    results = {}

    for aircraft_type in aircraft_types:
        viz = simulate_aircraft(aircraft_type, target_altitude=-100.0, sim_time=sim_time)
        results[aircraft_type] = viz

    # 結果の比較プロット
    print("\n結果を比較プロット中...")

    # 3D軌跡の比較
    fig = plt.figure(figsize=(15, 10))
    ax = fig.add_subplot(111, projection='3d')

    colors = {'micro': 'r', 'small': 'g', 'medium': 'b', 'large': 'm'}

    for aircraft_type, viz in results.items():
        states = np.array(viz.state_history)
        x = states[:, 0]
        y = states[:, 1]
        z = -states[:, 2]  # 高度を正の値で表示
        ax.plot(x, y, z, colors[aircraft_type], linewidth=2, label=aircraft_type.upper())

    ax.set_xlabel('北 [m]')
    ax.set_ylabel('東 [m]')
    ax.set_zlabel('高度 [m]')
    ax.set_title('機体タイプ別 3D飛行軌跡比較')
    ax.legend()
    ax.grid(True)
    plt.tight_layout()

    # 高度と速度の時系列比較
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))

    for aircraft_type, viz in results.items():
        times = np.array(viz.time_history)
        states = np.array(viz.state_history)

        # 高度
        altitude = -states[:, 2]
        axes[0].plot(times, altitude, colors[aircraft_type], linewidth=2, label=aircraft_type.upper())

        # 対気速度
        Va_list = []
        for state in states:
            u, v, w = state[3:6]
            Va = np.sqrt(u**2 + v**2 + w**2)
            Va_list.append(Va)
        axes[1].plot(times, Va_list, colors[aircraft_type], linewidth=2, label=aircraft_type.upper())

    axes[0].set_ylabel('高度 [m]')
    axes[0].set_title('高度比較')
    axes[0].legend()
    axes[0].grid(True)

    axes[1].set_ylabel('対気速度 [m/s]')
    axes[1].set_xlabel('時間 [s]')
    axes[1].set_title('対気速度比較')
    axes[1].legend()
    axes[1].grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
