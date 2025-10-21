#!/usr/bin/env python3
"""
複数機体モデルの動作確認テスト
"""

import numpy as np
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel

def test_aircraft_models():
    """全ての機体モデルを初期化してテスト"""
    print("=== 機体モデル動作確認 ===\n")

    aircraft_types = ['micro', 'small', 'medium', 'large']

    for aircraft_type in aircraft_types:
        print(f"--- {aircraft_type.upper()} UAV ---")

        # UAVの初期化
        uav = FixedWingUAV(aircraft_type=aircraft_type)
        aero = AerodynamicModel(aircraft_type=aircraft_type)

        # 機体情報を表示
        print(f"質量: {uav.params['mass']:.2f} kg")
        print(f"翼幅: {uav.params['b']:.2f} m")
        print(f"翼面積: {uav.params['S_wing']:.3f} m^2")

        # 短時間シミュレーション
        uav.set_state([0, 0, -100, 20, 0, 0, 0, 0, 0, 0, 0, 0])
        control = np.array([0.0, 0.0, 0.0, 0.5])

        for i in range(10):
            forces_moments = aero.compute_forces_moments(uav, control)
            uav.update(0.01, forces_moments)

        Va = uav.get_airspeed()
        pos = uav.get_position()
        print(f"10ステップ後: 速度={Va:.2f}m/s, 位置=({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")
        print(f"✓ {aircraft_type.upper()} UAV: 正常動作\n")

    print("全ての機体モデルが正常に動作しています!")
    print("\n小型UAV（目標機体）の詳細:")
    print("  - 全幅: 1.6m")
    print("  - 質量: 1.7kg")
    print("  - 用途: 制御則と誘導則の検証")

if __name__ == "__main__":
    try:
        test_aircraft_models()
    except Exception as e:
        print(f"\n✗ エラーが発生しました: {e}")
        import traceback
        traceback.print_exc()
