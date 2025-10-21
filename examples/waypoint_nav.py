#!/usr/bin/env python3
"""
Waypoint Navigation Simulation

Verification of guidance law for sequential waypoint following
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel
from src.controller import AttitudeController, AltitudeController, AirspeedController
from src.guidance import WaypointGuidance, CoordinatedTurnGuidance
from src.visualization import SimulationVisualizer


def main():
    print("=== Waypoint Navigation Simulation ===")

    # Simulation parameters
    dt = 0.01  # Time step [s]
    T_sim = 200.0  # Simulation time [s]

    # Define waypoints (North, East, Down) [m]
    waypoints = np.array([
        [0, 0, -100],
        [500, 0, -100],
        [500, 500, -150],
        [0, 500, -150],
        [0, 0, -100]
    ])

    print("Waypoints:")
    for i, wp in enumerate(waypoints):
        print(f"  WP{i}: North={wp[0]}m, East={wp[1]}m, Altitude={-wp[2]}m")

    # Initialize UAV
    uav = FixedWingUAV()
    uav.set_state([0, 0, -100, 15, 0, 0, 0, 0, 0, 0, 0, 0])

    # Aerodynamic model
    aero = AerodynamicModel()

    # Initialize controllers
    attitude_controller = AttitudeController()
    altitude_controller = AltitudeController()
    airspeed_controller = AirspeedController()

    # Initialize guidance laws
    waypoint_guidance = WaypointGuidance(waypoints, R_min=50.0)
    turn_guidance = CoordinatedTurnGuidance(V_a=15.0)

    # Visualization
    viz = SimulationVisualizer()

    # Set target values
    Va_c = 15.0  # Target airspeed [m/s]

    # Simulation loop
    time = 0.0
    step = 0

    print("\nStarting simulation...")

    while time < T_sim and not waypoint_guidance.completed:
        # Current state
        position = uav.get_position()
        phi, theta, psi = uav.get_attitude()
        Va = uav.get_airspeed()

        # Calculate target heading and altitude from guidance law
        waypoint, completed = waypoint_guidance.update(position)
        psi_c = waypoint_guidance.compute_heading_command(position)
        h_c = waypoint[2]

        # Generate roll command from coordinated turn guidance
        phi_c = turn_guidance.compute_roll_command(psi, psi_c, k_psi=0.5)

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
            current_wp_idx = waypoint_guidance.current_waypoint_index
            print(f"Time: {time:.1f}s, WP{current_wp_idx}, "
                  f"Position: ({position[0]:.0f}, {position[1]:.0f}, {-position[2]:.0f})")

        time += dt
        step += 1

    print("Simulation complete")
    if waypoint_guidance.completed:
        print("All waypoints reached")
    else:
        print(f"Timeout: reached WP{waypoint_guidance.current_waypoint_index}")

    # Visualize results
    print("\nPlotting results...")
    viz.plot_3d_trajectory(waypoints=waypoints)
    viz.plot_2d_trajectory(waypoints=waypoints)
    viz.plot_states()
    viz.plot_controls()
    viz.plot_airdata()


if __name__ == "__main__":
    main()
