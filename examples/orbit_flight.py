#!/usr/bin/env python3
"""
旋回飛行シミュレーション

指定した中心と半径で旋回飛行する誘導則の検証
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel
from src.controller import AttitudeController, AltitudeController, AirspeedController
from src.guidance import OrbitGuidance, CoordinatedTurnGuidance
from src.visualization import SimulationVisualizer


def main():
    print("=== 旋回飛行シミュレーション ===")

    # シミュレーションパラメータ
    dt = 0.01  # 時間ステップ [s]
    T_sim = 120.0  # シミュレーション時間 [s]

    # 旋回パラメータ
    orbit_center = np.array([500, 500, -150])  # 旋回中心 [m]
    orbit_radius = 200.0  # 旋回半径 [m]
    orbit_direction = 'CW'  # 旋回方向(CW: 時計回り, CCW: 反時計回り)

    print(f"旋回中心: 北={orbit_center[0]}m, 東={orbit_center[1]}m, 高度={-orbit_center[2]}m")
    print(f"旋回半径: {orbit_radius}m")
    print(f"旋回方向: {orbit_direction}")

    # UAVの初期化
    uav = FixedWingUAV()
    # 旋回円の外側から開始
    initial_pos = orbit_center + np.array([orbit_radius + 100, 0, 0])
    uav.set_state([initial_pos[0], initial_pos[1], initial_pos[2], 25, 0, 0, 0, 0, 0, 0, 0, 0])

    # 空力モデル
    aero = AerodynamicModel()

    # 制御器の初期化
    attitude_controller = AttitudeController()
    altitude_controller = AltitudeController()
    airspeed_controller = AirspeedController()

    # 誘導則の初期化
    orbit_guidance = OrbitGuidance(orbit_center, orbit_radius, direction=orbit_direction)
    turn_guidance = CoordinatedTurnGuidance(V_a=25.0)

    # 可視化
    viz = SimulationVisualizer()

    # 目標値の設定
    Va_c = 25.0  # 目標速度 [m/s]

    # シミュレーションループ
    time = 0.0
    step = 0

    print("\nシミュレーション開始...")

    while time < T_sim:
        # 現在の状態
        position = uav.get_position()
        phi, theta, psi = uav.get_attitude()
        Va = uav.get_airspeed()

        # 誘導則で目標方位角を計算
        psi_c = orbit_guidance.compute_heading_command(position, k_orbit=2.0)
        h_c = orbit_center[2]

        # 協調旋回でロール角指令を生成
        phi_c = turn_guidance.compute_roll_command(psi, psi_c, k_psi=0.8)

        # 高度制御でピッチ角指令を生成
        theta_c = altitude_controller.compute_pitch_command(uav, h_c, dt)

        # 速度制御でスロットル指令を生成
        delta_t = airspeed_controller.compute_throttle_command(uav, Va_c, dt)

        # 姿勢制御で舵面指令を生成
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

        # 進捗表示
        if step % 1000 == 0:
            radius_error = orbit_guidance.compute_radius_error(position)
            print(f"時刻: {time:.1f}s, 半径誤差: {radius_error:.1f}m, "
                  f"ロール角: {np.rad2deg(phi):.1f}deg")

        time += dt
        step += 1

    print("シミュレーション完了")

    # 最終的な半径誤差を計算
    final_radius_error = orbit_guidance.compute_radius_error(uav.get_position())
    print(f"最終半径誤差: {final_radius_error:.2f} m")

    # 結果の可視化
    print("\n結果をプロット中...")
    viz.plot_3d_trajectory()
    viz.plot_2d_trajectory(orbit_center=orbit_center, orbit_radius=orbit_radius)
    viz.plot_states()
    viz.plot_controls()
    viz.plot_airdata()


if __name__ == "__main__":
    main()
