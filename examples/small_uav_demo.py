#!/usr/bin/env python3
"""
小型UAV（全幅1.6m、質量1.7kg）デモンストレーション

目標機体での経路点追従シミュレーション
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel
from src.controller import AttitudeController, AltitudeController, AirspeedController
from src.guidance import WaypointGuidance, CoordinatedTurnGuidance
from src.visualization import SimulationVisualizer


def main():
    print("=== 小型UAV (1.6m, 1.7kg) 経路点航法シミュレーション ===")

    # シミュレーションパラメータ
    dt = 0.01  # 時間ステップ [s]
    T_sim = 150.0  # シミュレーション時間 [s]

    # 経路点の定義(北、東、下) [m]
    # 小型機に適したスケール
    waypoints = np.array([
        [0, 0, -80],
        [300, 0, -80],
        [300, 300, -100],
        [0, 300, -100],
        [0, 0, -80]
    ])

    print("\n経路点:")
    for i, wp in enumerate(waypoints):
        print(f"  WP{i}: 北={wp[0]}m, 東={wp[1]}m, 高度={-wp[2]}m")

    # 小型UAVの初期化
    uav = FixedWingUAV(aircraft_type='small')
    uav.set_state([0, 0, -80, 20, 0, 0, 0, 0, 0, 0, 0, 0])

    # 機体情報を表示
    print(f"\n機体仕様:")
    print(f"  質量: {uav.params['mass']:.2f} kg")
    print(f"  翼幅: {uav.params['b']:.2f} m")
    print(f"  翼面積: {uav.params['S_wing']:.3f} m^2")
    print(f"  アスペクト比: {uav.params['b']**2 / uav.params['S_wing']:.2f}")

    # 空力モデル
    aero = AerodynamicModel(aircraft_type='small')

    # 制御器の初期化
    attitude_controller = AttitudeController()
    altitude_controller = AltitudeController()
    airspeed_controller = AirspeedController()

    # 誘導則の初期化
    waypoint_guidance = WaypointGuidance(waypoints, R_min=40.0)  # 小型機用の到達半径
    turn_guidance = CoordinatedTurnGuidance(V_a=20.0)  # 小型機の巡航速度

    # 可視化
    viz = SimulationVisualizer()

    # 目標値の設定
    Va_c = 20.0  # 小型機に適した巡航速度 [m/s]

    # シミュレーションループ
    time = 0.0
    step = 0

    print("\nシミュレーション開始...")

    while time < T_sim and not waypoint_guidance.completed:
        # 現在の状態
        position = uav.get_position()
        phi, theta, psi = uav.get_attitude()
        Va = uav.get_airspeed()

        # 誘導則で目標方位角と高度を計算
        waypoint, completed = waypoint_guidance.update(position)
        psi_c = waypoint_guidance.compute_heading_command(position)
        h_c = waypoint[2]

        # 協調旋回でロール角指令を生成
        phi_c = turn_guidance.compute_roll_command(psi, psi_c, k_psi=0.6)

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
            current_wp_idx = waypoint_guidance.current_waypoint_index
            print(f"時刻: {time:.1f}s, WP{current_wp_idx}, "
                  f"位置: ({position[0]:.0f}, {position[1]:.0f}, {-position[2]:.0f}), "
                  f"速度: {Va:.1f}m/s")

        time += dt
        step += 1

    print("\nシミュレーション完了")
    if waypoint_guidance.completed:
        print("全ての経路点を通過しました")
    else:
        print(f"時間切れ: WP{waypoint_guidance.current_waypoint_index}まで到達")

    # 飛行統計
    states = np.array(viz.state_history)
    times = np.array(viz.time_history)

    # 総飛行距離を計算
    total_distance = 0.0
    for i in range(1, len(states)):
        dx = states[i, 0] - states[i-1, 0]
        dy = states[i, 1] - states[i-1, 1]
        dz = states[i, 2] - states[i-1, 2]
        total_distance += np.sqrt(dx**2 + dy**2 + dz**2)

    print(f"\n飛行統計:")
    print(f"  総飛行時間: {times[-1]:.1f} s")
    print(f"  総飛行距離: {total_distance:.1f} m")
    print(f"  平均速度: {total_distance/times[-1]:.1f} m/s")

    # 結果の可視化
    print("\n結果をプロット中...")
    viz.plot_3d_trajectory(waypoints=waypoints)
    viz.plot_2d_trajectory(waypoints=waypoints)
    viz.plot_states()
    viz.plot_controls()
    viz.plot_airdata()

    # データを保存
    viz.save_data('small_uav_flight_data.npz')


if __name__ == "__main__":
    main()
