#!/usr/bin/env python3
"""
基本的な飛行シミュレーション

姿勢制御と高度制御の基本的な動作確認
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel
from src.controller import AttitudeController, AltitudeController, AirspeedController
from src.visualization import SimulationVisualizer


def main():
    print("=== 基本飛行シミュレーション ===")

    # シミュレーションパラメータ
    dt = 0.01  # 時間ステップ [s]
    T_sim = 60.0  # シミュレーション時間 [s]

    # UAVの初期化
    uav = FixedWingUAV()
    uav.set_state([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])  # 初期状態

    # 空力モデル
    aero = AerodynamicModel()

    # 制御器の初期化
    attitude_controller = AttitudeController()
    altitude_controller = AltitudeController()
    airspeed_controller = AirspeedController()

    # 可視化
    viz = SimulationVisualizer()

    # 目標値の設定
    h_c = -150.0  # 目標高度 [m] (NED座標系)
    Va_c = 25.0   # 目標速度 [m/s]

    # シミュレーションループ
    time = 0.0
    step = 0

    print(f"初期位置: {uav.get_position()}")
    print(f"目標高度: {-h_c} m")
    print(f"目標速度: {Va_c} m/s")
    print("シミュレーション開始...")

    while time < T_sim:
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

        # 進捗表示
        if step % 1000 == 0:
            print(f"時刻: {time:.1f}s, 高度: {-position[2]:.1f}m, 速度: {Va:.1f}m/s")

        time += dt
        step += 1

    print("シミュレーション完了")
    print(f"最終位置: {uav.get_position()}")
    print(f"最終速度: {uav.get_airspeed():.2f} m/s")

    # 結果の可視化
    print("\n結果をプロット中...")
    viz.plot_3d_trajectory()
    viz.plot_states()
    viz.plot_controls()
    viz.plot_airdata()


if __name__ == "__main__":
    main()
