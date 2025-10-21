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
from src.controller import AttitudeController, AltitudeController, AirspeedController
from src.guidance import OrbitGuidance, CoordinatedTurnGuidance
from src.visualization import SimulationVisualizer


def main():
    print("=== Orbit Flight Simulation ===")

    # Simulation parameters
    dt = 0.01  # Time step [s]
    T_sim = 120.0  # Simulation time [s]

    # Orbit parameters
    orbit_center = np.array([500, 500, -150])  # Orbit center [m]
    orbit_radius = 200.0  # Orbit radius [m]
    orbit_direction = 'CW'  # Orbit direction (CW: clockwise, CCW: counter-clockwise)

    print(f"Orbit center: North={orbit_center[0]}m, East={orbit_center[1]}m, Altitude={-orbit_center[2]}m")
    print(f"Orbit radius: {orbit_radius}m")
    print(f"Orbit direction: {orbit_direction}")

    # Initialize UAV
    uav = FixedWingUAV()
    # Start from outside the orbit circle
    initial_pos = orbit_center + np.array([orbit_radius + 100, 0, 0])
    uav.set_state([initial_pos[0], initial_pos[1], initial_pos[2], 25, 0, 0, 0, 0, 0, 0, 0, 0])

    # Aerodynamic model
    aero = AerodynamicModel()

    # Initialize controllers
    attitude_controller = AttitudeController()
    altitude_controller = AltitudeController()
    airspeed_controller = AirspeedController()

    # Initialize guidance laws
    orbit_guidance = OrbitGuidance(orbit_center, orbit_radius, direction=orbit_direction)
    turn_guidance = CoordinatedTurnGuidance(V_a=25.0)

    # Visualization
    viz = SimulationVisualizer()

    # Set target values
    Va_c = 25.0  # Target airspeed [m/s]

    # Simulation loop
    time = 0.0
    step = 0

    print("\nStarting simulation...")

    while time < T_sim:
        # Current state
        position = uav.get_position()
        phi, theta, psi = uav.get_attitude()
        Va = uav.get_airspeed()

        # Calculate target heading from guidance law
        psi_c = orbit_guidance.compute_heading_command(position, k_orbit=2.0)
        h_c = orbit_center[2]

        # Generate roll command from coordinated turn guidance
        phi_c = turn_guidance.compute_roll_command(psi, psi_c, k_psi=0.8)

        # Generate pitch command from altitude controller
        theta_c = altitude_controller.compute_pitch_command(uav, h_c, dt)

        # Generate throttle command from airspeed controller
        delta_t = airspeed_controller.compute_throttle_command(uav, Va_c, dt)

        # Generate control surface commands from attitude controller
        delta_a, delta_e, delta_r = attitude_controller.compute_control(uav, phi_c, theta_c, dt)

        # Set control inputs
        control = np.array([delta_e, delta_a, delta_r, delta_t])
        uav.set_control(control)

        # Calculate aerodynamic forces and moments
        forces_moments = aero.compute_forces_moments(uav, control)

        # Update state
        uav.update(dt, forces_moments)

        # Record data
        if step % 10 == 0:  # Record every 0.1 seconds
            viz.add_data(time, uav.get_state(), control)

        # Progress display
        if step % 1000 == 0:
            radius_error = orbit_guidance.compute_radius_error(position)
            print(f"Time: {time:.1f}s, Radius error: {radius_error:.1f}m, "
                  f"Roll angle: {np.rad2deg(phi):.1f}deg")

        time += dt
        step += 1

    print("Simulation complete")

    # Calculate final radius error
    final_radius_error = orbit_guidance.compute_radius_error(uav.get_position())
    print(f"Final radius error: {final_radius_error:.2f} m")

    # Visualize results
    print("\nPlotting results...")
    viz.plot_3d_trajectory()
    viz.plot_2d_trajectory(orbit_center=orbit_center, orbit_radius=orbit_radius)
    viz.plot_states()
    viz.plot_controls()
    viz.plot_airdata()


if __name__ == "__main__":
    main()
