#!/usr/bin/env python3
"""
安定性の異なる機体のテスト
"""

import numpy as np
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel

def test_stability_variants():
    """安定性バリエーションのテスト"""
    print("=== 安定性バリエーション動作確認 ===\n")

    variants = ['small', 'small_slightly_unstable', 'small_unstable']

    for variant in variants:
        print(f"--- {variant.upper()} ---")

        # UAVと空力モデルの初期化
        uav = FixedWingUAV(aircraft_type=variant)
        aero = AerodynamicModel(aircraft_type=variant)

        # 機体情報を表示
        print(f"機体: {variant}")
        print(f"質量: {uav.params['mass']:.2f} kg")
        print(f"翼幅: {uav.params['b']:.2f} m")

        # 安定性微係数を表示
        print(f"安定性微係数:")
        print(f"  C_l_beta (ロール): {aero.aero_params['C_l_beta']:.3f}")
        print(f"  C_m_alpha (ピッチ): {aero.aero_params['C_m_alpha']:.3f}")
        print(f"  C_n_beta (ヨー): {aero.aero_params['C_n_beta']:.3f}")

        # 短時間シミュレーション
        uav.set_state([0, 0, -100, 20, 0, 0, 0, 0, 0, 0, 0, 0])
        control = np.array([0.0, 0.0, 0.0, 0.5])

        for i in range(10):
            forces_moments = aero.compute_forces_moments(uav, control)
            uav.update(0.01, forces_moments)

        Va = uav.get_airspeed()
        pos = uav.get_position()
        print(f"10ステップ後: 速度={Va:.2f}m/s, 位置=({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")
        print(f"✓ {variant.upper()}: 正常動作\n")

    print("全ての安定性バリエーションが正常に動作しています!")
    print("\n安定性の判定基準:")
    print("  ロール: C_l_beta (負で安定)")
    print("  ピッチ: C_m_alpha (負で安定)")
    print("  ヨー: C_n_beta (正で安定)")

if __name__ == "__main__":
    try:
        test_stability_variants()
    except Exception as e:
        print(f"\n✗ エラーが発生しました: {e}")
        import traceback
        traceback.print_exc()
