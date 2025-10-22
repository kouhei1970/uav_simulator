#!/usr/bin/env python3
"""
インナーループ（角速度制御）のテストプログラム

角速度の指令値を直接与えて、インナーループPIDコントローラの
追従性能を確認・調整します。

テスト内容：
- ロールレート (p) のステップ入力追従
- ピッチレート (q) のステップ入力追従
- ヨーレート (r) のステップ入力追従
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel
from src.controller import CascadeAttitudeController
from src.visualization import SimulationVisualizer


def main():
    print("=== Inner Loop (Rate Control) Test ===")
    print("Testing angular rate control (p, q, r)")
    print()

    # Initialize UAV
    uav = FixedWingUAV()

    # Set trim condition for stable 15 m/s flight
    print("Setting up trim condition for level flight at 15 m/s...")
    Va_trim = 15.0  # Target airspeed [m/s]
    alpha_trim = np.deg2rad(1.9)  # Angle of attack for trim
    theta_trim = np.deg2rad(5.0)  # Pitch angle for trim

    # Calculate velocity components
    u_trim = Va_trim * np.cos(alpha_trim)
    w_trim = Va_trim * np.sin(alpha_trim)

    # Aerodynamic model
    aero = AerodynamicModel()

    # Elevator trim
    C_m_0 = aero.aero_params['C_m_0']
    C_m_alpha = aero.aero_params['C_m_alpha']
    C_m_delta_e = aero.aero_params['C_m_delta_e']
    elevator_trim = -(C_m_0 + C_m_alpha * alpha_trim) / C_m_delta_e
    throttle_trim = 0.305  # Optimized trim for minimal altitude change at 15 m/s

    print(f"Trim conditions:")
    print(f"  Throttle: {throttle_trim:.3f}")
    print(f"  Elevator: {np.rad2deg(elevator_trim):.2f} deg")
    print()

    # Set initial condition at trim
    uav.set_state([
        0, 0, -100,  # Position: 100m altitude
        u_trim, 0, w_trim,  # Velocity
        0, theta_trim, 0,  # Attitude
        0, 0, 0  # Angular velocity
    ])

    # Initialize controller
    controller = CascadeAttitudeController()

    # Initialize visualizer
    viz = SimulationVisualizer()

    # Simulation parameters
    dt = 0.01  # 10ms time step
    sim_time = 60.0  # 60 seconds
    steps = int(sim_time / dt)

    print("Starting inner loop test...")
    print("Test sequence (conservative commands):")
    print("  0-10s:  Zero rate command (trim)")
    print("  10-20s: Roll rate +10 deg/s")
    print("  20-30s: Roll rate -10 deg/s")
    print("  30-40s: Pitch rate +5 deg/s")
    print("  40-50s: Pitch rate -5 deg/s")
    print("  50-60s: Yaw rate +10 deg/s")
    print()

    # Simulation loop
    for step in range(steps):
        time = step * dt

        # Define rate commands (インナーループの指令値)
        # より小さい指令値で安定性を確認
        if time < 10.0:
            # Trim condition
            p_c = 0.0
            q_c = 0.0
            r_c = 0.0
        elif time < 20.0:
            # Roll right
            p_c = np.deg2rad(10)  # 10 deg/s
            q_c = 0.0
            r_c = 0.0
        elif time < 30.0:
            # Roll left
            p_c = np.deg2rad(-10)
            q_c = 0.0
            r_c = 0.0
        elif time < 40.0:
            # Pitch up
            p_c = 0.0
            q_c = np.deg2rad(5)  # 5 deg/s
            r_c = 0.0
        elif time < 50.0:
            # Pitch down
            p_c = 0.0
            q_c = np.deg2rad(-5)
            r_c = 0.0
        else:
            # Yaw right
            p_c = 0.0
            q_c = 0.0
            r_c = np.deg2rad(10)  # 10 deg/s

        # Get current angular rates
        p, q, r = uav.get_angular_velocity()

        # Compute inner loop control (rate errors -> control surfaces)
        p_error = p_c - p
        delta_a = controller.roll_rate_controller.update(p_error, dt)

        q_error = q_c - q
        delta_e = controller.pitch_rate_controller.update(q_error, dt)

        r_error = r_c - r
        delta_r = controller.yaw_rate_controller.update(r_error, dt)

        # Use trim throttle
        delta_t = throttle_trim

        # Store control inputs
        control = np.array([delta_e, delta_a, delta_r, delta_t])

        # Compute aerodynamic forces and moments
        forces_moments = aero.compute_forces_moments(uav, control)

        # Update state
        uav.update(dt, forces_moments)

        # Record data
        if step % 10 == 0:  # Record every 0.1 seconds
            # Create command dict with rate commands
            command = {
                'p': p_c,
                'q': q_c,
                'r': r_c
            }
            viz.add_data(time, uav.get_state(), control, command=command)

        # Progress display
        if step % 1000 == 0:
            phi, theta, psi = uav.get_attitude()
            alt = -uav.get_position()[2]
            print(f"Time: {time:.1f}s, "
                  f"p: {np.rad2deg(p):+6.1f}°/s (cmd: {np.rad2deg(p_c):+6.1f}°/s), "
                  f"q: {np.rad2deg(q):+6.1f}°/s (cmd: {np.rad2deg(q_c):+6.1f}°/s), "
                  f"r: {np.rad2deg(r):+6.1f}°/s (cmd: {np.rad2deg(r_c):+6.1f}°/s), "
                  f"Alt: {alt:.1f}m")

    print("\nSimulation complete!")

    # Final tracking errors
    p, q, r = uav.get_angular_velocity()
    print(f"\nFinal rates:")
    print(f"  Roll rate:  {np.rad2deg(p):+6.2f}°/s (cmd: {np.rad2deg(p_c):+6.2f}°/s)")
    print(f"  Pitch rate: {np.rad2deg(q):+6.2f}°/s (cmd: {np.rad2deg(q_c):+6.2f}°/s)")
    print(f"  Yaw rate:   {np.rad2deg(r):+6.2f}°/s (cmd: {np.rad2deg(r_c):+6.2f}°/s)")

    # Show plots
    print("\nGenerating plots...")
    viz.plot_states()


if __name__ == "__main__":
    main()
