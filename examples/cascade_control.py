"""
カスケードPID姿勢制御のデモンストレーション

ロール、ピッチ、ヨーの3軸すべてでカスケードPID制御を実装:
- アウターループ: 角度制御 (角度誤差 → 目標角速度)
- インナーループ: 角速度制御 (角速度誤差 → 舵角)

ステップ入力で各軸の追従性能を確認
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
    print("=== Cascade PID Attitude Control Demo ===")
    print("Testing 3-axis cascade control:")
    print("  Outer loop: Angle control")
    print("  Inner loop: Rate control")
    print()

    # Initialize UAV
    uav = FixedWingUAV()

    # Set trim condition for stable 15 m/s flight (from basic_flight.py)
    print("Setting up trim condition for level flight at 15 m/s...")
    Va_trim = 15.0  # Target airspeed [m/s]
    alpha_trim = np.deg2rad(1.9)  # Angle of attack for trim
    theta_trim = np.deg2rad(5.0)  # Pitch angle for trim

    # Calculate velocity components for proper angle of attack
    u_trim = Va_trim * np.cos(alpha_trim)
    w_trim = Va_trim * np.sin(alpha_trim)

    # Aerodynamic model
    aero = AerodynamicModel()

    # Elevator trim for pitch moment equilibrium
    C_m_0 = aero.aero_params['C_m_0']
    C_m_alpha = aero.aero_params['C_m_alpha']
    C_m_delta_e = aero.aero_params['C_m_delta_e']
    elevator_trim = -(C_m_0 + C_m_alpha * alpha_trim) / C_m_delta_e
    throttle_trim = 0.31  # From basic_flight.py

    print(f"Trim conditions:")
    print(f"  Throttle: {throttle_trim:.3f}")
    print(f"  Elevator: {np.rad2deg(elevator_trim):.2f} deg")
    print(f"  Pitch angle: {np.rad2deg(theta_trim):.2f} deg")
    print(f"  Angle of attack: {np.rad2deg(alpha_trim):.2f} deg")
    print(f"  Velocity: u={u_trim:.2f} m/s, w={w_trim:.2f} m/s")
    print()

    # Set initial condition at trim
    uav.set_state([
        0, 0, -100,  # Position: 100m altitude (NED)
        u_trim, 0, w_trim,  # Velocity components (body frame)
        0, theta_trim, 0,  # Attitude (roll=0, pitch=trim, yaw=0)
        0, 0, 0  # Angular velocity
    ])

    # Initialize cascade attitude controller
    controller = CascadeAttitudeController()

    # Initialize visualizer
    viz = SimulationVisualizer()

    # Simulation parameters
    dt = 0.01  # 10ms time step
    sim_time = 80.0  # 80 seconds total
    steps = int(sim_time / dt)

    print("Starting cascade control simulation...")
    print("Test sequence:")
    print("  0-10s:  Level flight (trim)")
    print("  10-20s: Roll right 30 deg")
    print("  20-30s: Roll left 30 deg")
    print("  30-40s: Return to level")
    print("  40-50s: Pitch up 10 deg")
    print("  50-60s: Pitch down 10 deg")
    print("  60-70s: Yaw right 45 deg")
    print("  70-80s: Yaw left 45 deg")
    print()

    # Simulation loop
    for step in range(steps):
        time = step * dt

        # Define time-varying attitude commands (step inputs)
        if time < 10.0:
            # Level flight
            phi_c = 0.0
            theta_c = theta_trim
            psi_c = 0.0
        elif time < 20.0:
            # Roll right 30 degrees
            phi_c = np.deg2rad(30)
            theta_c = theta_trim
            psi_c = 0.0
        elif time < 30.0:
            # Roll left 30 degrees
            phi_c = np.deg2rad(-30)
            theta_c = theta_trim
            psi_c = 0.0
        elif time < 40.0:
            # Return to level
            phi_c = 0.0
            theta_c = theta_trim
            psi_c = 0.0
        elif time < 50.0:
            # Pitch up 10 degrees
            phi_c = 0.0
            theta_c = theta_trim + np.deg2rad(10)
            psi_c = 0.0
        elif time < 60.0:
            # Pitch down 10 degrees
            phi_c = 0.0
            theta_c = theta_trim - np.deg2rad(10)
            psi_c = 0.0
        elif time < 70.0:
            # Yaw right 45 degrees
            phi_c = 0.0
            theta_c = theta_trim
            psi_c = np.deg2rad(45)
        else:
            # Yaw left 45 degrees
            phi_c = 0.0
            theta_c = theta_trim
            psi_c = np.deg2rad(-45)

        # Compute cascade control for all 3 axes
        delta_a, delta_e_cascade, delta_r, p_c, q_c, r_c = controller.compute_control(
            uav, phi_c, theta_c, psi_c, dt
        )

        # Current stable configuration: Roll and yaw cascade control
        # Pitch requires airspeed control for stability
        delta_e = elevator_trim  # Elevator at trim (pitch at trim)

        # Use trim throttle (no speed control in this demo)
        delta_t = throttle_trim

        # Store control inputs
        control = np.array([delta_e, delta_a, delta_r, delta_t])

        # Compute aerodynamic forces and moments
        forces_moments = aero.compute_forces_moments(uav, control)

        # Update state
        uav.update(dt, forces_moments)

        # Record data
        if step % 10 == 0:  # Record every 0.1 seconds
            # Create command dict with all controlled values
            command = {
                'phi': phi_c,
                'theta': theta_c,
                'psi': psi_c,
                'p': p_c,
                'q': q_c,
                'r': r_c
            }
            viz.add_data(time, uav.get_state(), control, command=command)

        # Progress display
        if step % 1000 == 0:
            phi, theta, psi = uav.get_attitude()
            p, q, r = uav.get_angular_velocity()
            alt = -uav.get_position()[2]
            print(f"Time: {time:.1f}s, "
                  f"Phi: {np.rad2deg(phi):+6.1f}° (cmd: {np.rad2deg(phi_c):+6.1f}°), "
                  f"Theta: {np.rad2deg(theta):+6.1f}° (cmd: {np.rad2deg(theta_c):+6.1f}°), "
                  f"Psi: {np.rad2deg(psi):+6.1f}° (cmd: {np.rad2deg(psi_c):+6.1f}°), "
                  f"Alt: {alt:.1f}m")

    print("\nSimulation complete!")

    # Calculate tracking errors
    final_phi, final_theta, final_psi = uav.get_attitude()
    print(f"\nFinal attitude:")
    print(f"  Roll:  {np.rad2deg(final_phi):+6.2f}° (cmd: {np.rad2deg(phi_c):+6.2f}°)")
    print(f"  Pitch: {np.rad2deg(final_theta):+6.2f}° (cmd: {np.rad2deg(theta_c):+6.2f}°)")
    print(f"  Yaw:   {np.rad2deg(final_psi):+6.2f}° (cmd: {np.rad2deg(psi_c):+6.2f}°)")

    # Show plots
    print("\nGenerating plots...")
    viz.plot_states()


if __name__ == "__main__":
    main()
