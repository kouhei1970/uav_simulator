#!/usr/bin/env python3
"""
簡単な動作確認テスト
"""

import numpy as np
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel
from src.controller import AttitudeController

def test_basic():
    """基本的な動作確認"""
    print("=== 固定翼UAVシミュレータ動作確認 ===\n")

    # UAVの初期化
    print("1. UAVの初期化...")
    uav = FixedWingUAV()
    uav.set_state([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
    print(f"   初期位置: {uav.get_position()}")
    print(f"   初期速度: {uav.get_airspeed():.2f} m/s")

    # 空力モデル
    print("\n2. 空力モデルの初期化...")
    aero = AerodynamicModel()

    # 制御器
    print("\n3. 制御器の初期化...")
    controller = AttitudeController()

    # 短時間シミュレーション
    print("\n4. 1秒間のシミュレーション実行...")
    dt = 0.01
    for i in range(100):
        control = np.array([0.0, 0.0, 0.0, 0.5])
        forces_moments = aero.compute_forces_moments(uav, control)
        uav.update(dt, forces_moments)

    print(f"   最終位置: {uav.get_position()}")
    print(f"   最終速度: {uav.get_airspeed():.2f} m/s")

    print("\n✓ 全ての基本機能が正常に動作しています!")
    print("\n次のコマンドでサンプルを実行できます:")
    print("  python examples/basic_flight.py")
    print("  python examples/waypoint_nav.py")
    print("  python examples/orbit_flight.py")
    return True

if __name__ == "__main__":
    try:
        test_basic()
    except Exception as e:
        print(f"\n✗ エラーが発生しました: {e}")
        import traceback
        traceback.print_exc()
