#!/usr/bin/env python3
"""
トリム条件探索プログラム

高度変化が最小になるエレベータとスロットルの組み合わせを探索します。
15 m/s の水平飛行でのトリム条件を求めます。
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel


def test_trim(elevator_deg, throttle, sim_time=30.0):
    """
    指定されたトリム値で飛行させて、高度変化率を計算

    Parameters:
        elevator_deg: エレベータ角度 [deg]
        throttle: スロットル [0-1]
        sim_time: シミュレーション時間 [s]

    Returns:
        alt_rate: 平均高度変化率 [m/s]
        final_alt: 最終高度 [m]
        final_Va: 最終対気速度 [m/s]
    """
    # Initialize UAV
    uav = FixedWingUAV()
    aero = AerodynamicModel()

    # Set initial condition
    Va_trim = 15.0
    alpha_trim = np.deg2rad(1.9)
    theta_trim = np.deg2rad(5.0)

    u_trim = Va_trim * np.cos(alpha_trim)
    w_trim = Va_trim * np.sin(alpha_trim)

    uav.set_state([
        0, 0, 0,  # Position: 100m altitude
        u_trim, 0, w_trim,  # Velocity
        0, theta_trim, 0,  # Attitude
        0, 0, 0  # Angular velocity
    ])

    # Simulation parameters
    dt = 0.01
    steps = int(sim_time / dt)

    # Convert elevator to radians
    elevator_rad = np.deg2rad(elevator_deg)

    # Fixed control inputs
    control = np.array([elevator_rad, 0.0, 0.0, throttle])

    # Track altitude
    initial_alt = -uav.get_position()[2]
    altitudes = []
    times = []

    # Simulation loop
    for step in range(steps):
        time = step * dt

        # Compute forces and update
        forces_moments = aero.compute_forces_moments(uav, control)
        uav.update(dt, forces_moments)

        # Record altitude every 0.1 seconds
        if step % 10 == 0:
            alt = -uav.get_position()[2]
            altitudes.append(alt)
            times.append(time)

    # Calculate altitude change rate (use last half of simulation for steady state)
    mid_idx = len(altitudes) // 2
    final_alt = altitudes[-1]
    mid_alt = altitudes[mid_idx]
    time_span = times[-1] - times[mid_idx]

    alt_rate = (final_alt - mid_alt) / time_span

    # Get final airspeed
    final_Va = uav.get_airspeed()

    return alt_rate, final_alt, final_Va


def main():
    print("=== Trim Condition Search ===")
    print("Finding elevator and throttle for minimal altitude change at 15 m/s")
    print()

    # Search ranges
    elevator_range = np.linspace(-6.0, 0.0, 20)  # -6 to 9 degrees
    throttle_range = np.linspace(0.25, 0.40, 20)  # 0.25 to 0.40

    print(f"Testing {len(elevator_range)} elevator values × {len(throttle_range)} throttle values")
    print(f"Elevator range: {elevator_range[0]:.1f} to {elevator_range[-1]:.1f} deg")
    print(f"Throttle range: {throttle_range[0]:.2f} to {throttle_range[-1]:.2f}")
    print()

    # Store results
    results = []

    print("Elevator  Throttle  Alt_rate   Final_alt  Final_Va")
    print("  [deg]      [-]      [m/s]       [m]      [m/s]")
    print("-" * 56)

    for elevator in elevator_range:
        for throttle in throttle_range:
            alt_rate, final_alt, final_Va = test_trim(elevator, throttle, sim_time=30.0)

            results.append({
                'elevator': elevator,
                'throttle': throttle,
                'alt_rate': alt_rate,
                'final_alt': final_alt,
                'final_Va': final_Va,
                'abs_alt_rate': abs(alt_rate)
            })

            print(f"  {elevator:5.2f}    {throttle:5.3f}   {alt_rate:+7.3f}    {final_alt:6.1f}    {final_Va:5.2f}")

    print()

    # Find best trim (minimum absolute altitude change rate)
    best = min(results, key=lambda x: x['abs_alt_rate'])

    print("=" * 56)
    print("OPTIMAL TRIM CONDITIONS:")
    print(f"  Elevator:        {best['elevator']:+6.2f} deg")
    print(f"  Throttle:         {best['throttle']:.3f}")
    print(f"  Altitude rate:   {best['alt_rate']:+7.3f} m/s")
    print(f"  Final altitude:   {best['final_alt']:.1f} m")
    print(f"  Final airspeed:   {best['final_Va']:.2f} m/s")
    print("=" * 56)
    print()

    # Show top 5 candidates
    sorted_results = sorted(results, key=lambda x: x['abs_alt_rate'])
    print("Top 5 trim candidates:")
    print("Elevator  Throttle  |Alt_rate|")
    print("  [deg]      [-]      [m/s]")
    print("-" * 35)
    for i, r in enumerate(sorted_results[:5], 1):
        print(f"{i}. {r['elevator']:5.2f}    {r['throttle']:5.3f}    {r['abs_alt_rate']:6.4f}")


if __name__ == "__main__":
    main()
