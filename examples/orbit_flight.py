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
# Controllers not needed - using simple proportional control instead
from src.guidance import ProportionalOrbitGuidance
from src.visualization import SimulationVisualizer


def main():
    print("=== Orbit Flight Simulation ===")

    # Simulation parameters
    dt = 0.01  # Time step [s]
    T_sim = 120.0  # Simulation time [s]

    # Orbit parameters
    orbit_center = np.array([300, 300, -100])  # Orbit center [m]
    orbit_radius = 45.0  # Orbit radius [m] (optimal range for small UAV: 30-100m)
    orbit_direction = 'CW'  # Orbit direction (CW: clockwise, CCW: counter-clockwise)

    print(f"Orbit center: North={orbit_center[0]}m, East={orbit_center[1]}m, Altitude={-orbit_center[2]}m")
    print(f"Orbit radius: {orbit_radius}m (optimal range for small UAV: 30-100m)")
    print(f"Orbit direction: {orbit_direction}")

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

    # Note: We use simple proportional control instead of full attitude controller
    # for better stability during orbit flight

    # Initialize proportional orbit guidance
    # K_p controls convergence rate: tuned for stable convergence
    # Smaller radius requires careful tuning
    orbit_guidance = ProportionalOrbitGuidance(K_p=0.005, phi_max=np.deg2rad(35))

    # Visualization
    viz = SimulationVisualizer()

    # Set target values
    Va_c = 15.0  # Target airspeed [m/s]

    # Calculate trim control inputs
    # Elevator trim for pitch moment equilibrium
    C_m_0 = aero.aero_params['C_m_0']
    C_m_alpha = aero.aero_params['C_m_alpha']
    C_m_delta_e = aero.aero_params['C_m_delta_e']
    elevator_trim = -(C_m_0 + C_m_alpha * alpha_trim) / C_m_delta_e
    throttle_trim = 0.31  # From basic_flight.py

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

        # Simple proportional control for roll angle (more stable than full attitude controller)
        phi_error = phi_c - phi
        # Wrap angle error to ±π
        while phi_error > np.pi:
            phi_error -= 2 * np.pi
        while phi_error < -np.pi:
            phi_error += 2 * np.pi

        # Simple proportional aileron control
        # Higher gain for faster roll response in tight turns
        k_phi = 1.2  # Roll angle gain (higher = faster response)
        delta_a = k_phi * phi_error
        delta_a = np.clip(delta_a, -0.4, 0.4)  # Limit aileron deflection

        # Use trim elevator and throttle for stable flight
        # Note: Altitude gradually increases (~0.3 m/s) due to banked flight,
        # but orbit tracking is stable
        delta_e = elevator_trim
        delta_t = throttle_trim
        delta_r = 0.0  # No rudder

        # Set control inputs
        control = np.array([delta_e, delta_a, delta_r, delta_t])
        uav.set_control(control)

        # Calculate aerodynamic forces and moments
        forces_moments = aero.compute_forces_moments(uav, control)

        # Update state
        uav.update(dt, forces_moments)

        # Record data
        if step % 10 == 0:  # Record every 0.1 seconds
            # Create command dict: only include values that are actually being controlled
            # We only control phi (roll angle)
            command = {'phi': phi_c}
            viz.add_data(time, uav.get_state(), control, command=command)

        # Progress display
        if step % 1000 == 0:
            print(f"Time: {time:.1f}s, Radius error: {radius_error:.1f}m, "
                  f"Distance: {distance:.1f}m, Phi_cmd: {np.rad2deg(phi_c):.1f}deg, "
                  f"Phi: {np.rad2deg(phi):.1f}deg, Alt: {-position[2]:.1f}m")

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
