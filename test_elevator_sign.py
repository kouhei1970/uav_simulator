"""
エレベータ舵角の符号を検証するテストプログラム

正のエレベータ偏角を与えた時のピッチモーメントの符号を確認
"""

import sys
import os
sys.path.append(os.path.dirname(__file__))

import numpy as np
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel

def test_elevator_sign():
    print("=" * 60)
    print("エレベータ舵角の符号テスト")
    print("=" * 60)
    print()

    # UAVと空力モデルを初期化
    uav = FixedWingUAV()
    aero = AerodynamicModel()

    # トリム条件を設定
    Va_trim = 15.0
    alpha_trim = np.deg2rad(1.9)
    theta_trim = np.deg2rad(5.0)

    u_trim = Va_trim * np.cos(alpha_trim)
    w_trim = Va_trim * np.sin(alpha_trim)

    # 初期状態を設定
    uav.set_state([
        0, 0, -100,  # Position
        u_trim, 0, w_trim,  # Velocity
        0, theta_trim, 0,  # Attitude
        0, 0, 0  # Angular velocity
    ])

    # エレベータトリムを計算
    C_m_0 = aero.aero_params['C_m_0']
    C_m_alpha = aero.aero_params['C_m_alpha']
    C_m_delta_e = aero.aero_params['C_m_delta_e']
    elevator_trim = -(C_m_0 + C_m_alpha * alpha_trim) / C_m_delta_e

    print(f"空力係数:")
    print(f"  C_m_0 = {C_m_0:.4f}")
    print(f"  C_m_alpha = {C_m_alpha:.4f}")
    print(f"  C_m_delta_e = {C_m_delta_e:.4f}")
    print()
    print(f"トリム条件:")
    print(f"  エレベータトリム = {np.rad2deg(elevator_trim):.2f}°")
    print()

    # テスト1: トリム舵角でのピッチモーメント
    print("-" * 60)
    print("テスト1: トリム舵角でのピッチモーメント")
    print("-" * 60)
    control_trim = np.array([elevator_trim, 0, 0, 0.305])
    forces_moments_trim = aero.compute_forces_moments(uav, control_trim)
    M_trim = forces_moments_trim[4]  # Pitch moment

    print(f"エレベータ = {np.rad2deg(elevator_trim):.2f}°")
    print(f"ピッチモーメント M = {M_trim:.6f} Nm")
    print(f"期待値: M ≈ 0 (トリム状態)")
    print()

    # テスト2: 正のエレベータ偏角
    print("-" * 60)
    print("テスト2: 正のエレベータ偏角(+5°)")
    print("-" * 60)
    delta_e_test = np.deg2rad(5.0)
    control_positive = np.array([delta_e_test, 0, 0, 0.305])
    forces_moments_positive = aero.compute_forces_moments(uav, control_positive)
    M_positive = forces_moments_positive[4]

    print(f"エレベータ = {np.rad2deg(delta_e_test):.2f}°")
    print(f"ピッチモーメント M = {M_positive:.6f} Nm")
    print()

    if M_positive > 0:
        print("✓ 正のエレベータ → 正のモーメント(機首上げ) [正しい]")
    else:
        print("✗ 正のエレベータ → 負のモーメント(機首下げ) [間違い!]")
    print()

    # テスト3: 負のエレベータ偏角
    print("-" * 60)
    print("テスト3: 負のエレベータ偏角(-5°)")
    print("-" * 60)
    delta_e_test = np.deg2rad(-5.0)
    control_negative = np.array([delta_e_test, 0, 0, 0.305])
    forces_moments_negative = aero.compute_forces_moments(uav, control_negative)
    M_negative = forces_moments_negative[4]

    print(f"エレベータ = {np.rad2deg(delta_e_test):.2f}°")
    print(f"ピッチモーメント M = {M_negative:.6f} Nm")
    print()

    if M_negative < 0:
        print("✓ 負のエレベータ → 負のモーメント(機首下げ) [正しい]")
    else:
        print("✗ 負のエレベータ → 正のモーメント(機首上げ) [間違い!]")
    print()

    # 結論
    print("=" * 60)
    print("結論")
    print("=" * 60)
    print()
    print(f"C_m_delta_e = {C_m_delta_e:.4f}")
    print()

    if C_m_delta_e < 0:
        print("【問題】C_m_delta_e が負です。")
        print()
        print("標準的な航空機の符号規約では:")
        print("  - 正のエレベータ偏角(後縁上げ) → 機首上げモーメント")
        print("  - したがって C_m_delta_e は正であるべき")
        print()
        print("【推奨される修正】")
        print(f"  C_m_delta_e = {-C_m_delta_e:.4f} (符号を反転)")
        print()
        print("【現在の対処法】")
        print("  cascade_control.py でピッチレート制御器のゲインを負にして補正")
        print("  (行98-99: kp=-10.0, ki=-1.0)")
        print()
        print("この対処法は以下の問題があります:")
        print("  1. 直感的でない")
        print("  2. ゲインチューニングが困難")
        print("  3. 制御理論の標準と矛盾")
        print()
    else:
        print("✓ C_m_delta_e が正です。符号規約は正しいです。")
    print()

if __name__ == "__main__":
    test_elevator_sign()
