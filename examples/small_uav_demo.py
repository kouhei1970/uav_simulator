#!/usr/bin/env python3
"""
Small UAV (1.6m wingspan, 1.7kg mass) Demonstration

Waypoint following simulation with target aircraft
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
    print("=== Small UAV (1.6m, 1.7kg) Waypoint Navigation Simulation ===")

    # Simulation parameters
    dt = 0.01  # Time step [s]
    T_sim = 150.0  # Simulation time [s]

    # Define waypoints (North, East, Down) [m]
    # Scale appropriate for small aircraft
    waypoints = np.array([
        [0, 0, -80],
        [300, 0, -80],
        [300, 300, -100],
        [0, 300, -100],
        [0, 0, -80]
    ])

    print("\nWaypoints:")
    for i, wp in enumerate(waypoints):
        print(f"  WP{i}: North={wp[0]}m, East={wp[1]}m, Altitude={-wp[2]}m")

    # Initialize small UAV
    uav = FixedWingUAV(aircraft_type='small')
    uav.set_state([0, 0, -80, 15, 0, 0, 0, 0, 0, 0, 0, 0])

    # Display aircraft specifications
    print(f"\nAircraft Specifications:")
    print(f"  Mass: {uav.params['mass']:.2f} kg")
    print(f"  Wingspan: {uav.params['b']:.2f} m")
    print(f"  Wing Area: {uav.params['S_wing']:.3f} m^2")
    print(f"  Aspect Ratio: {uav.params['b']**2 / uav.params['S_wing']:.2f}")
    print(f"  Cruise Speed: {uav.params['V_cruise']:.1f} m/s")

    # Aerodynamic model
    aero = AerodynamicModel(aircraft_type='small')

    # Initialize controllers
    attitude_controller = AttitudeController()
    altitude_controller = AltitudeController()
    airspeed_controller = AirspeedController()

    # Initialize guidance laws
    waypoint_guidance = WaypointGuidance(waypoints, R_min=40.0)  # Capture radius for small aircraft
    turn_guidance = CoordinatedTurnGuidance(V_a=15.0)  # Cruise speed for small aircraft

    # Visualization
    viz = SimulationVisualizer()

    # Set target values
    Va_c = 15.0  # Cruise speed appropriate for small aircraft [m/s]

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
        phi_c = turn_guidance.compute_roll_command(psi, psi_c, k_psi=0.6)

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
                  f"Position: ({position[0]:.0f}, {position[1]:.0f}, {-position[2]:.0f}), "
                  f"Airspeed: {Va:.1f}m/s")

        time += dt
        step += 1

    print("\nSimulation complete")
    if waypoint_guidance.completed:
        print("All waypoints reached")
    else:
        print(f"Timeout: reached WP{waypoint_guidance.current_waypoint_index}")

    # Flight statistics
    states = np.array(viz.state_history)
    times = np.array(viz.time_history)

    # Calculate total flight distance
    total_distance = 0.0
    for i in range(1, len(states)):
        dx = states[i, 0] - states[i-1, 0]
        dy = states[i, 1] - states[i-1, 1]
        dz = states[i, 2] - states[i-1, 2]
        total_distance += np.sqrt(dx**2 + dy**2 + dz**2)

    print(f"\nFlight Statistics:")
    print(f"  Total flight time: {times[-1]:.1f} s")
    print(f"  Total distance: {total_distance:.1f} m")
    print(f"  Average speed: {total_distance/times[-1]:.1f} m/s")

    # Visualize results
    print("\nPlotting results...")
    viz.plot_3d_trajectory(waypoints=waypoints)
    viz.plot_2d_trajectory(waypoints=waypoints)
    viz.plot_states()
    viz.plot_controls()
    viz.plot_airdata()

    # Save data
    viz.save_data('small_uav_flight_data.npz')


if __name__ == "__main__":
    main()
