#!/usr/bin/env python3
"""
トリム条件詳細探索プログラム

15 m/s付近の対気速度を維持しながら、高度変化が最小になる条件を探索
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel


def test_trim(elevator_deg, throttle, sim_time=30.0):
    """トリム値をテストして高度変化率と対気速度を計算"""
    uav = FixedWingUAV()
    aero = AerodynamicModel()

    # Initial condition
    Va_trim = 15.0
    alpha_trim = np.deg2rad(1.9)
    theta_trim = np.deg2rad(5.0)

    u_trim = Va_trim * np.cos(alpha_trim)
    w_trim = Va_trim * np.sin(alpha_trim)

    uav.set_state([
        0, 0, -100,
        u_trim, 0, w_trim,
        0, theta_trim, 0,
        0, 0, 0
    ])

    dt = 0.01
    steps = int(sim_time / dt)

    elevator_rad = np.deg2rad(elevator_deg)
    control = np.array([elevator_rad, 0.0, 0.0, throttle])

    altitudes = []
    times = []

    for step in range(steps):
        time = step * dt
        forces_moments = aero.compute_forces_moments(uav, control)
        uav.update(dt, forces_moments)

        if step % 10 == 0:
            alt = -uav.get_position()[2]
            altitudes.append(alt)
            times.append(time)

    # Calculate altitude change rate from second half
    mid_idx = len(altitudes) // 2
    final_alt = altitudes[-1]
    mid_alt = altitudes[mid_idx]
    time_span = times[-1] - times[mid_idx]

    alt_rate = (final_alt - mid_alt) / time_span
    final_Va = uav.get_airspeed()

    return alt_rate, final_alt, final_Va


def main():
    print("=== Fine Trim Search (15 m/s target) ===")
    print()

    # Fine search range around promising area
    elevator_range = np.linspace(-4.0, -3.0, 21)  # -4.0 to -3.0 deg, 0.05 deg steps
    throttle_range = np.linspace(0.300, 0.350, 11)  # 0.300 to 0.350, 0.005 steps

    print(f"Testing {len(elevator_range)} elevator values × {len(throttle_range)} throttle values")
    print(f"Elevator range: {elevator_range[0]:.2f} to {elevator_range[-1]:.2f} deg")
    print(f"Throttle range: {throttle_range[0]:.3f} to {throttle_range[-1]:.3f}")
    print()

    results = []

    print("Elevator  Throttle  Alt_rate   Final_alt  Final_Va  |Alt_rate| Va_error")
    print("  [deg]      [-]      [m/s]       [m]      [m/s]     [m/s]      [m/s]")
    print("-" * 76)

    target_Va = 15.0

    for elevator in elevator_range:
        for throttle in throttle_range:
            alt_rate, final_alt, final_Va = test_trim(elevator, throttle, sim_time=40.0)

            Va_error = abs(final_Va - target_Va)

            results.append({
                'elevator': elevator,
                'throttle': throttle,
                'alt_rate': alt_rate,
                'final_alt': final_alt,
                'final_Va': final_Va,
                'abs_alt_rate': abs(alt_rate),
                'Va_error': Va_error
            })

            print(f"  {elevator:5.2f}    {throttle:5.3f}   {alt_rate:+7.3f}    {final_alt:6.1f}    {final_Va:5.2f}     {abs(alt_rate):6.3f}    {Va_error:5.2f}")

    print()

    # Find best overall (minimize both alt_rate and Va_error)
    # Weight: prioritize alt_rate, but keep Va close to 15 m/s
    for r in results:
        r['score'] = r['abs_alt_rate'] + 0.1 * r['Va_error']  # Weighted score

    best = min(results, key=lambda x: x['score'])

    print("=" * 76)
    print("OPTIMAL TRIM (balanced for 15 m/s and minimal altitude change):")
    print(f"  Elevator:        {best['elevator']:+6.2f} deg")
    print(f"  Throttle:         {best['throttle']:.3f}")
    print(f"  Altitude rate:   {best['alt_rate']:+7.3f} m/s")
    print(f"  Final altitude:   {best['final_alt']:.1f} m")
    print(f"  Final airspeed:   {best['final_Va']:.2f} m/s (target: 15.00 m/s)")
    print(f"  Score:            {best['score']:.4f}")
    print("=" * 76)
    print()

    # Show top 10 candidates
    sorted_results = sorted(results, key=lambda x: x['score'])
    print("Top 10 trim candidates:")
    print("    Elevator  Throttle  |Alt_rate|  Va_error   Score")
    print("      [deg]      [-]      [m/s]      [m/s]")
    print("-" * 60)
    for i, r in enumerate(sorted_results[:10], 1):
        print(f"{i:2d}.  {r['elevator']:5.2f}    {r['throttle']:5.3f}    {r['abs_alt_rate']:6.4f}    {r['Va_error']:5.3f}   {r['score']:6.4f}")


if __name__ == "__main__":
    main()
