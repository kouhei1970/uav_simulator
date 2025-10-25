#!/usr/bin/env python3
"""
Orbit Flight Simulation

Verification of guidance law for orbit flight with specified center and radius
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel
from src.controller import CascadeAttitudeController, AltitudeController
from src.guidance import ProportionalOrbitGuidance
from src.visualization import SimulationVisualizer


def main():
    print("=== Orbit Flight Simulation ===")

    # Simulation parameters
    dt = 0.001  # Time step [s] (same as cascade_control.py)
    T_sim = 120.0  # Simulation time [s]

    # Control switches
    enable_altitude_control = True  # Enable/disable altitude control (True: use AltitudeController, False: use trim pitch)
    use_derivative_on_measurement = True  # PID type (True: derivative-on-PV, False: conventional derivative-on-error)

    # Orbit parameters
    orbit_center = np.array([300, 300, -100])  # Orbit center [m]
    orbit_radius = 45.0  # Orbit radius [m] (optimal range for small UAV: 30-100m)
    orbit_direction = 'CW'  # Orbit direction (CW: clockwise, CCW: counter-clockwise)

    print(f"Orbit center: North={orbit_center[0]}m, East={orbit_center[1]}m, Altitude={-orbit_center[2]}m")
    print(f"Orbit radius: {orbit_radius}m (optimal range for small UAV: 30-100m)")
    print(f"Orbit direction: {orbit_direction}")
    print(f"Altitude control: {'ENABLED' if enable_altitude_control else 'DISABLED (using trim pitch)'}")
    print(f"PID type: {'Derivative-on-Measurement (微分先行型)' if use_derivative_on_measurement else 'Derivative-on-Error (従来型)'}")

    # Initialize UAV
    uav = FixedWingUAV()

    # Set trim condition for stable 15 m/s flight (same as basic_flight.py)
    Va_trim = 15.0  # Target airspeed [m/s]
    alpha_trim = np.deg2rad(1.9)  # Angle of attack for trim
    pitch_trim = np.deg2rad(5.0)  # Pitch angle for trim

    # Calculate velocity components for proper angle of attack
    u_trim = Va_trim * np.cos(alpha_trim)
    w_trim = Va_trim * np.sin(alpha_trim)

    # Start from very close to the orbit circle for smooth entry
    # Smaller radius needs closer initial position
    initial_pos = orbit_center + np.array([orbit_radius + 2, 0, 0])

    # Set initial heading tangent to orbit circle for smooth entry
    # Starting position is north of center [300+60, 300, -100]
    # For CCW (counter-clockwise from above): fly west initially (psi = -π/2)
    # For CW (clockwise from above): fly east initially (psi = +π/2)
    if orbit_direction == 'CCW':
        initial_psi = -np.pi / 2  # West (-90 deg = 270 deg)
    else:
        initial_psi = np.pi / 2  # East (90 deg)

    # Set initial state with trim condition
    uav.set_state([
        initial_pos[0], initial_pos[1], initial_pos[2],  # Position
        u_trim, 0, w_trim,  # Velocity components (body frame)
        0, pitch_trim, initial_psi,  # Attitude (roll, pitch, yaw)
        0, 0, 0  # Angular velocity
    ])

    # Aerodynamic model
    aero = AerodynamicModel()

    # Calculate trim control inputs
    # Elevator trim for pitch moment equilibrium
    C_m_0 = aero.aero_params['C_m_0']
    C_m_alpha = aero.aero_params['C_m_alpha']
    C_m_delta_e = aero.aero_params['C_m_delta_e']
    elevator_trim = -(C_m_0 + C_m_alpha * alpha_trim) / C_m_delta_e
    # Throttle trim for orbit flight - fine-tuned for minimal altitude change
    throttle_trim = 0.305  # Optimized for minimal altitude loss during orbit (same as level flight)

    # Initialize cascade attitude controller with elevator trim and PID type
    controller = CascadeAttitudeController(
        elevator_trim=elevator_trim,
        derivative_on_measurement=use_derivative_on_measurement
    )

    # Initialize altitude controller for orbit flight
    altitude_controller = AltitudeController()

    # Adjust altitude controller gains for smoother control (reduce elevator saturation and overshoot)
    altitude_controller.altitude_controller.kp = 0.025  # Adjusted for balanced convergence
    altitude_controller.altitude_controller.ki = 0.005  # Reduced from 0.01
    altitude_controller.altitude_controller.kd = 0.01  # Restored to original value

    # ============================================================
    # Cascade PID Gain Settings (from cascade_control.py)
    # ============================================================
    print("Configuring cascade PID gains...")

    # Roll cascade control
    controller.roll_angle_controller.kp = 10.0
    controller.roll_angle_controller.ki = 0
    controller.roll_angle_controller.kd = 0
    controller.roll_angle_controller.limit = (-2.0, 2.0)

    controller.roll_rate_controller.kp = 1.5
    controller.roll_rate_controller.ki = 0.5
    controller.roll_rate_controller.kd = 0.002
    controller.roll_rate_controller.limit = (-0.4, 0.4)

    # Pitch cascade control (higher Kp for orbit flight with large bank angles)
    controller.pitch_angle_controller.kp = 20.0
    controller.pitch_angle_controller.ki = 5.0  # Reduced from 10.0 to further avoid elevator saturation
    controller.pitch_angle_controller.kd = 2.0
    controller.pitch_angle_controller.limit = (-2.0, 2.0)

    controller.pitch_rate_controller.kp = 5.0  # Reduced from 10.0 to avoid saturation
    controller.pitch_rate_controller.ki = 0.5  # Reduced from 1.0 to avoid saturation
    controller.pitch_rate_controller.kd = 0.0
    controller.pitch_rate_controller.limit = (-0.12, 0.12)

    # Yaw cascade control (disabled - under adjustment in cascade_control.py)
    controller.yaw_angle_controller.kp = 0
    controller.yaw_angle_controller.ki = 0
    controller.yaw_angle_controller.kd = 0
    controller.yaw_angle_controller.limit = (-2.0, 2.0)

    controller.yaw_rate_controller.kp = 0
    controller.yaw_rate_controller.ki = 0
    controller.yaw_rate_controller.kd = 0
    controller.yaw_rate_controller.limit = (-0.5, 0.5)

    print("  Roll:  Outer(Kp={:.1f}, Ki={:.2f}, Kd={:.1f})  Inner(Kp={:.2f}, Ki={:.3f}, Kd={:.3f})".format(
        controller.roll_angle_controller.kp, controller.roll_angle_controller.ki, controller.roll_angle_controller.kd,
        controller.roll_rate_controller.kp, controller.roll_rate_controller.ki, controller.roll_rate_controller.kd))
    print("  Pitch: Outer(Kp={:.1f}, Ki={:.2f}, Kd={:.2f})  Inner(Kp={:.2f}, Ki={:.3f}, Kd={:.3f})".format(
        controller.pitch_angle_controller.kp, controller.pitch_angle_controller.ki, controller.pitch_angle_controller.kd,
        controller.pitch_rate_controller.kp, controller.pitch_rate_controller.ki, controller.pitch_rate_controller.kd))
    print("  Yaw:   Outer(Kp={:.1f}, Ki={:.1f}, Kd={:.1f})  Inner(Kp={:.2f}, Ki={:.2f}, Kd={:.3f})".format(
        controller.yaw_angle_controller.kp, controller.yaw_angle_controller.ki, controller.yaw_angle_controller.kd,
        controller.yaw_rate_controller.kp, controller.yaw_rate_controller.ki, controller.yaw_rate_controller.kd))
    print()
    # ============================================================

    # Initialize proportional orbit guidance
    # K_p controls convergence rate: tuned for stable convergence
    # Smaller radius requires careful tuning
    orbit_guidance = ProportionalOrbitGuidance(K_p=0.005, phi_max=np.deg2rad(35))

    # Visualization
    viz = SimulationVisualizer()

    # Simulation loop
    time = 0.0
    step = 0

    print("\nStarting simulation...")

    while time < T_sim:
        # Current state
        position = uav.get_position()
        phi, theta, psi = uav.get_attitude()
        Va = uav.get_airspeed()

        # Calculate roll command directly from proportional orbit guidance
        phi_c, radius_error, distance = orbit_guidance.compute_roll_command(
            position, orbit_center, orbit_radius, Va, direction=orbit_direction
        )
        h_c = orbit_center[2]

        # Pitch angle command: use altitude controller or trim pitch based on switch
        if enable_altitude_control:
            # Use altitude controller to compute pitch angle command for maintaining altitude
            # Altitude controller output is increment from trim, so add to trim pitch
            theta_c_increment = altitude_controller.compute_pitch_command(uav, h_c, dt)
            theta_c_base = pitch_trim + theta_c_increment

            # Compensate for bank angle to maintain altitude during orbit
            # During banked flight, need higher pitch angle due to reduced vertical lift component
            # Lift vertical component = L * cos(phi), so need pitch compensation
            # Note: Altitude controller feedback can compensate, so coefficient can be small
            theta_compensation = pitch_trim * (1.0 / np.cos(phi) - 1.0) * 0.3
            theta_c = theta_c_base + theta_compensation
        else:
            # Use trim pitch with bank angle compensation
            theta_compensation = pitch_trim * (1.0 / np.cos(phi) - 1.0) * 0.5
            theta_c = pitch_trim + theta_compensation

        # psi_c: no yaw control (let it follow the orbit naturally)
        psi_c = psi  # Follow current yaw (no yaw control)

        # Compute cascade control for all 3 axes
        delta_a, delta_e_increment, delta_r, p_c, q_c, r_c = controller.compute_control(
            uav, phi_c, theta_c, psi_c, dt
        )

        # Apply elevator control (trim + increment from pitch controller)
        delta_e = elevator_trim + delta_e_increment
        delta_t = throttle_trim

        # Set control inputs
        control = np.array([delta_e, delta_a, delta_r, delta_t])
        uav.set_control(control)

        # Calculate aerodynamic forces and moments
        forces_moments = aero.compute_forces_moments(uav, control)

        # Update state
        uav.update(dt, forces_moments)

        # Record data
        if step % 10 == 0:  # Record every 0.01 seconds (10 steps * 0.001s)
            # Create command dict with all controlled values (same as cascade_control.py)
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
            q = uav.get_angular_velocity()[1]
            print(f"Time: {time:.1f}s, Radius error: {radius_error:.1f}m, "
                  f"Phi_cmd: {np.rad2deg(phi_c):.1f}deg, Phi: {np.rad2deg(phi):.1f}deg, "
                  f"Theta_cmd: {np.rad2deg(theta_c):.2f}deg, Theta: {np.rad2deg(theta):.2f}deg, "
                  f"q_c: {np.rad2deg(q_c):.2f}deg/s, q: {np.rad2deg(q):.2f}deg/s, "
                  f"delta_e_inc: {np.rad2deg(delta_e_increment):.3f}deg, Alt: {-position[2]:.1f}m")

        time += dt
        step += 1

    print("Simulation complete")

    # Calculate final orbit metrics
    final_position = uav.get_position()
    final_distance = np.linalg.norm(final_position[0:2] - orbit_center[0:2])
    final_radius_error = final_distance - orbit_radius
    print(f"Final radius error: {final_radius_error:.2f} m")
    print(f"Final distance from center: {final_distance:.2f} m (target: {orbit_radius:.2f} m)")

    # Visualize results
    print("\nPlotting results...")
    viz.plot_3d_trajectory()
    viz.plot_2d_trajectory(orbit_center=orbit_center, orbit_radius=orbit_radius)
    viz.plot_states()
    viz.plot_controls()
    viz.plot_airdata()


if __name__ == "__main__":
    main()
