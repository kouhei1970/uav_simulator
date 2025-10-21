#!/usr/bin/env python3
"""
Multi-Aircraft Comparison Simulation

Compare flight characteristics of different aircraft types
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import matplotlib.pyplot as plt
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel
from src.controller import AttitudeController, AltitudeController, AirspeedController
from src.visualization import SimulationVisualizer


def simulate_aircraft(aircraft_type, target_altitude=-100.0, target_speed=None, sim_time=60.0):
    """
    Simulate with specified aircraft type

    Parameters:
        aircraft_type: Aircraft type ('small', 'medium', 'micro', 'large')
        target_altitude: Target altitude [m]
        target_speed: Target airspeed [m/s] (auto-set based on aircraft size if None)
        sim_time: Simulation time [s]

    Returns:
        viz: SimulationVisualizer object
    """
    print(f"\n=== {aircraft_type.upper()} UAV Simulation ===")

    # Set target speed based on aircraft size
    if target_speed is None:
        speed_map = {
            'micro': 12.0,
            'small': 15.0,
            'medium': 25.0,
            'large': 30.0
        }
        target_speed = speed_map.get(aircraft_type, 25.0)

    # Initialize UAV
    uav = FixedWingUAV(aircraft_type=aircraft_type)
    uav.set_state([0, 0, target_altitude, target_speed, 0, 0, 0, 0, 0, 0, 0, 0])

    # Aerodynamic model
    aero = AerodynamicModel(aircraft_type=aircraft_type)

    # Initialize controllers
    attitude_controller = AttitudeController()
    altitude_controller = AltitudeController()
    airspeed_controller = AirspeedController()

    # Visualization
    viz = SimulationVisualizer()

    # Display aircraft information
    print(f"Mass: {uav.params['mass']:.2f} kg")
    print(f"Wingspan: {uav.params['b']:.2f} m")
    print(f"Wing Area: {uav.params['S_wing']:.3f} m^2")
    print(f"Target airspeed: {target_speed:.1f} m/s")

    # Simulation parameters
    dt = 0.01  # Time step [s]

    # Target values
    h_c = target_altitude - 50.0  # Climb 50m more
    Va_c = target_speed

    # Simulation loop
    time = 0.0
    step = 0

    while time < sim_time:
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

        time += dt
        step += 1

    print(f"Final altitude: {-position[2]:.1f} m")
    print(f"Final airspeed: {Va:.1f} m/s")

    return viz


def main():
    print("=== Multi-Aircraft Comparison Simulation ===")

    # Simulation time
    sim_time = 60.0

    # Simulate each aircraft type
    aircraft_types = ['micro', 'small', 'medium', 'large']
    results = {}

    for aircraft_type in aircraft_types:
        viz = simulate_aircraft(aircraft_type, target_altitude=-100.0, sim_time=sim_time)
        results[aircraft_type] = viz

    # Plot comparison results
    print("\nPlotting comparison results...")

    # 3D trajectory comparison
    fig = plt.figure(figsize=(15, 10))
    ax = fig.add_subplot(111, projection='3d')

    colors = {'micro': 'r', 'small': 'g', 'medium': 'b', 'large': 'm'}

    for aircraft_type, viz in results.items():
        states = np.array(viz.state_history)
        x = states[:, 0]
        y = states[:, 1]
        z = -states[:, 2]  # Display altitude as positive
        ax.plot(x, y, z, colors[aircraft_type], linewidth=2, label=aircraft_type.upper())

    ax.set_xlabel('North [m]')
    ax.set_ylabel('East [m]')
    ax.set_zlabel('Altitude [m]')
    ax.set_title('3D Flight Trajectory Comparison by Aircraft Type')
    ax.legend()
    ax.grid(True)
    plt.tight_layout()

    # Time series comparison of altitude and airspeed
    fig, axes = plt.subplots(2, 1, figsize=(12, 8))

    for aircraft_type, viz in results.items():
        times = np.array(viz.time_history)
        states = np.array(viz.state_history)

        # Altitude
        altitude = -states[:, 2]
        axes[0].plot(times, altitude, colors[aircraft_type], linewidth=2, label=aircraft_type.upper())

        # Airspeed
        Va_list = []
        for state in states:
            u, v, w = state[3:6]
            Va = np.sqrt(u**2 + v**2 + w**2)
            Va_list.append(Va)
        axes[1].plot(times, Va_list, colors[aircraft_type], linewidth=2, label=aircraft_type.upper())

    axes[0].set_ylabel('Altitude [m]')
    axes[0].set_title('Altitude Comparison')
    axes[0].legend()
    axes[0].grid(True)

    axes[1].set_ylabel('Airspeed [m/s]')
    axes[1].set_xlabel('Time [s]')
    axes[1].set_title('Airspeed Comparison')
    axes[1].legend()
    axes[1].grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
