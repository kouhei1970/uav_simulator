#!/usr/bin/env python3
"""
Basic Flight Simulation

Verification of basic attitude and altitude control
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel
from src.controller import AttitudeController, AltitudeController, AirspeedController
from src.visualization import SimulationVisualizer


def main():
    print("=== Basic Flight Simulation ===")

    # Simulation parameters
    dt = 0.01  # Time step [s]
    T_sim = 60.0  # Simulation time [s]

    # Initialize UAV
    uav = FixedWingUAV()
    uav.set_state([0, 0, -100, 15, 0, 0, 0, 0, 0, 0, 0, 0])  # Initial state

    # Aerodynamic model
    aero = AerodynamicModel()

    # Initialize controllers
    attitude_controller = AttitudeController()
    altitude_controller = AltitudeController()
    airspeed_controller = AirspeedController()

    # Visualization
    viz = SimulationVisualizer()

    # Set target values
    h_c = -150.0  # Target altitude [m] (NED frame)
    Va_c = 15.0   # Target airspeed [m/s]

    # Simulation loop
    time = 0.0
    step = 0

    print(f"Initial position: {uav.get_position()}")
    print(f"Target altitude: {-h_c} m")
    print(f"Target airspeed: {Va_c} m/s")
    print("Starting simulation...")

    while time < T_sim:
        # Current state
        position = uav.get_position()
        phi, theta, psi = uav.get_attitude()
        Va = uav.get_airspeed()

        # Generate pitch command from altitude controller
        theta_c = altitude_controller.compute_pitch_command(uav, h_c, dt)

        # Generate throttle command from airspeed controller
        delta_t = airspeed_controller.compute_throttle_command(uav, Va_c, dt)

        # Generate control surface commands from attitude controller
        phi_c = 0.0  # Level flight
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
            print(f"Time: {time:.1f}s, Altitude: {-position[2]:.1f}m, Airspeed: {Va:.1f}m/s")

        time += dt
        step += 1

    print("Simulation complete")
    print(f"Final position: {uav.get_position()}")
    print(f"Final airspeed: {uav.get_airspeed():.2f} m/s")

    # Visualize results
    print("\nPlotting results...")
    viz.plot_3d_trajectory()
    viz.plot_states()
    viz.plot_controls()
    viz.plot_airdata()


if __name__ == "__main__":
    main()
