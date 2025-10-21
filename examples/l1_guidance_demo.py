#!/usr/bin/env python3
"""
L1 Adaptive Guidance Demonstration

Demonstrates L1 guidance for straight line and orbit path following
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import matplotlib.pyplot as plt
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel
from src.controller import AttitudeController, AltitudeController, AirspeedController
from src.guidance import L1Guidance
from src.visualization import SimulationVisualizer


def simulate_line_following(sim_time=100.0):
    """L1誘導による直線経路追従のシミュレーション"""
    print("=== L1 Guidance: Straight Line Following ===")

    # Simulation parameters
    dt = 0.01  # Time step [s]

    # Define straight line path
    start_point = np.array([0, 0, -100])
    end_point = np.array([800, 400, -100])

    print(f"Path: Start={start_point}, End={end_point}")

    # Initialize UAV
    uav = FixedWingUAV(aircraft_type='small')
    # Start offset from the path
    uav.set_state([0, -50, -100, 15, 0, 0, 0, 0, 0, 0, 0, 0])

    # Aerodynamic model
    aero = AerodynamicModel(aircraft_type='small')

    # Initialize controllers
    attitude_controller = AttitudeController()
    altitude_controller = AltitudeController()
    airspeed_controller = AirspeedController()

    # Initialize L1 guidance
    l1_guidance = L1Guidance(L1_damping=0.707, L1_period=15.0)

    # Path parameters
    path_params = {
        'start': start_point,
        'end': end_point
    }

    # Visualization
    viz = SimulationVisualizer()

    # Set target values
    Va_c = 15.0  # Target airspeed [m/s]
    h_c = -100.0  # Target altitude [m]

    # Simulation loop
    time = 0.0
    step = 0
    crosstrack_errors = []
    L1_distances = []

    print("Starting simulation...")

    while time < sim_time:
        # Current state
        position = uav.get_position()
        phi, theta, psi = uav.get_attitude()
        Va = uav.get_airspeed()
        u, v, w = uav.get_velocity()

        # Compute course angle (ground track)
        chi = np.arctan2(v, u)

        # L1 guidance: compute lateral acceleration command
        a_cmd, eta = l1_guidance.compute_lateral_acceleration(
            position, Va, chi, path_type='line', path_params=path_params
        )

        # Convert to roll command
        phi_c = l1_guidance.compute_roll_command(a_cmd, Va, phi_limit=np.pi/4)

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

            # Calculate crosstrack error
            r = position - start_point
            path_direction = end_point - start_point
            path_direction = path_direction / np.linalg.norm(path_direction)
            s = np.dot(r, path_direction)
            r_perpendicular = r - s * path_direction
            crosstrack_error = np.linalg.norm(r_perpendicular[0:2])
            crosstrack_errors.append(crosstrack_error)

            # Record L1 distance
            L1 = l1_guidance.compute_L1_distance(Va)
            L1_distances.append(L1)

        # Progress display
        if step % 1000 == 0:
            print(f"Time: {time:.1f}s, Position: ({position[0]:.0f}, {position[1]:.0f}, {-position[2]:.0f})")

        time += dt
        step += 1

    print("Simulation complete")
    print(f"Final crosstrack error: {crosstrack_errors[-1]:.2f} m")
    print(f"Mean L1 distance: {np.mean(L1_distances):.2f} m")

    return viz, crosstrack_errors, start_point, end_point


def simulate_orbit_following(sim_time=120.0):
    """L1誘導による円軌道追従のシミュレーション"""
    print("\n=== L1 Guidance: Orbit Following ===")

    # Simulation parameters
    dt = 0.01  # Time step [s]

    # Define orbit
    orbit_center = np.array([400, 400, -100])
    orbit_radius = 60.0
    orbit_direction = 'CW'

    print(f"Orbit: Center={orbit_center}, Radius={orbit_radius}m, Direction={orbit_direction}")

    # Initialize UAV
    uav = FixedWingUAV(aircraft_type='small')
    # Start outside the orbit
    initial_pos = orbit_center + np.array([orbit_radius + 100, 0, 0])
    uav.set_state([initial_pos[0], initial_pos[1], initial_pos[2], 15, 0, 0, 0, 0, 0, 0, 0, 0])

    # Aerodynamic model
    aero = AerodynamicModel(aircraft_type='small')

    # Initialize controllers
    attitude_controller = AttitudeController()
    altitude_controller = AltitudeController()
    airspeed_controller = AirspeedController()

    # Initialize L1 guidance
    l1_guidance = L1Guidance(L1_damping=0.707, L1_period=15.0)

    # Path parameters
    path_params = {
        'center': orbit_center,
        'radius': orbit_radius,
        'direction': orbit_direction
    }

    # Visualization
    viz = SimulationVisualizer()

    # Set target values
    Va_c = 15.0  # Target airspeed [m/s]
    h_c = orbit_center[2]  # Target altitude [m]

    # Simulation loop
    time = 0.0
    step = 0
    radius_errors = []

    print("Starting simulation...")

    while time < sim_time:
        # Current state
        position = uav.get_position()
        phi, theta, psi = uav.get_attitude()
        Va = uav.get_airspeed()
        u, v, w = uav.get_velocity()

        # Compute course angle (ground track)
        chi = np.arctan2(v, u)

        # L1 guidance: compute lateral acceleration command
        a_cmd, eta = l1_guidance.compute_lateral_acceleration(
            position, Va, chi, path_type='orbit', path_params=path_params
        )

        # Convert to roll command
        phi_c = l1_guidance.compute_roll_command(a_cmd, Va, phi_limit=np.pi/4)

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

            # Calculate radius error
            d_vec = position[0:2] - orbit_center[0:2]
            d = np.linalg.norm(d_vec)
            radius_error = d - orbit_radius
            radius_errors.append(radius_error)

        # Progress display
        if step % 1000 == 0:
            print(f"Time: {time:.1f}s, Position: ({position[0]:.0f}, {position[1]:.0f}, {-position[2]:.0f})")

        time += dt
        step += 1

    print("Simulation complete")
    print(f"Final radius error: {radius_errors[-1]:.2f} m")
    print(f"Mean radius error: {np.mean(radius_errors[-1000:]):.2f} m")  # Last 10 seconds

    return viz, radius_errors, orbit_center, orbit_radius


def main():
    print("=== L1 Adaptive Guidance Demonstration ===\n")

    # Run line following simulation
    viz_line, crosstrack_errors, start_point, end_point = simulate_line_following()

    # Run orbit following simulation
    viz_orbit, radius_errors, orbit_center, orbit_radius = simulate_orbit_following()

    # Plot results
    print("\nPlotting results...")

    # Line following: 3D trajectory
    fig1 = plt.figure(figsize=(15, 10))

    # 3D trajectory for line following
    ax1 = fig1.add_subplot(221, projection='3d')
    states = np.array(viz_line.state_history)
    x = states[:, 0]
    y = states[:, 1]
    z = -states[:, 2]
    ax1.plot(x, y, z, 'b-', linewidth=2, label='UAV trajectory')
    ax1.plot([start_point[0], end_point[0]], [start_point[1], end_point[1]],
             [-start_point[2], -end_point[2]], 'r--', linewidth=2, label='Desired path')
    ax1.scatter([start_point[0]], [start_point[1]], [-start_point[2]], c='g', s=100, label='Start')
    ax1.scatter([end_point[0]], [end_point[1]], [-end_point[2]], c='r', s=100, label='End')
    ax1.set_xlabel('North [m]')
    ax1.set_ylabel('East [m]')
    ax1.set_zlabel('Altitude [m]')
    ax1.set_title('L1 Guidance: Straight Line Following (3D)')
    ax1.legend()
    ax1.grid(True)

    # 2D trajectory for line following
    ax2 = fig1.add_subplot(222)
    ax2.plot(x, y, 'b-', linewidth=2, label='UAV trajectory')
    ax2.plot([start_point[0], end_point[0]], [start_point[1], end_point[1]],
             'r--', linewidth=2, label='Desired path')
    ax2.scatter([start_point[0]], [start_point[1]], c='g', s=100, marker='o', label='Start')
    ax2.scatter([end_point[0]], [end_point[1]], c='r', s=100, marker='x', label='End')
    ax2.set_xlabel('North [m]')
    ax2.set_ylabel('East [m]')
    ax2.set_title('L1 Guidance: Straight Line Following (2D)')
    ax2.legend()
    ax2.grid(True)
    ax2.axis('equal')

    # Crosstrack error
    ax3 = fig1.add_subplot(223)
    times = np.array(viz_line.time_history)
    ax3.plot(times, crosstrack_errors, 'b-', linewidth=2)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Crosstrack Error [m]')
    ax3.set_title('Crosstrack Error History')
    ax3.grid(True)

    # Roll angle
    ax4 = fig1.add_subplot(224)
    controls = np.array(viz_line.control_history)
    states = np.array(viz_line.state_history)
    phi = states[:, 6] * 180 / np.pi
    ax4.plot(times, phi, 'b-', linewidth=2)
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Roll Angle [deg]')
    ax4.set_title('Roll Angle History')
    ax4.grid(True)

    plt.tight_layout()

    # Orbit following: 3D trajectory
    fig2 = plt.figure(figsize=(15, 10))

    # 3D trajectory for orbit following
    ax5 = fig2.add_subplot(221, projection='3d')
    states = np.array(viz_orbit.state_history)
    x = states[:, 0]
    y = states[:, 1]
    z = -states[:, 2]
    ax5.plot(x, y, z, 'b-', linewidth=2, label='UAV trajectory')

    # Draw orbit circle
    theta = np.linspace(0, 2*np.pi, 100)
    orbit_x = orbit_center[0] + orbit_radius * np.cos(theta)
    orbit_y = orbit_center[1] + orbit_radius * np.sin(theta)
    orbit_z = np.ones_like(theta) * (-orbit_center[2])
    ax5.plot(orbit_x, orbit_y, orbit_z, 'r--', linewidth=2, label='Desired orbit')
    ax5.scatter([orbit_center[0]], [orbit_center[1]], [-orbit_center[2]], c='g', s=100, label='Center')

    ax5.set_xlabel('North [m]')
    ax5.set_ylabel('East [m]')
    ax5.set_zlabel('Altitude [m]')
    ax5.set_title('L1 Guidance: Orbit Following (3D)')
    ax5.legend()
    ax5.grid(True)

    # 2D trajectory for orbit following
    ax6 = fig2.add_subplot(222)
    ax6.plot(x, y, 'b-', linewidth=2, label='UAV trajectory')
    ax6.plot(orbit_x, orbit_y, 'r--', linewidth=2, label='Desired orbit')
    ax6.scatter([orbit_center[0]], [orbit_center[1]], c='g', s=100, marker='o', label='Center')
    ax6.set_xlabel('North [m]')
    ax6.set_ylabel('East [m]')
    ax6.set_title('L1 Guidance: Orbit Following (2D)')
    ax6.legend()
    ax6.grid(True)
    ax6.axis('equal')

    # Radius error
    ax7 = fig2.add_subplot(223)
    times = np.array(viz_orbit.time_history)
    ax7.plot(times, radius_errors, 'b-', linewidth=2)
    ax7.axhline(y=0, color='r', linestyle='--', linewidth=1)
    ax7.set_xlabel('Time [s]')
    ax7.set_ylabel('Radius Error [m]')
    ax7.set_title('Orbit Radius Error History')
    ax7.grid(True)

    # Roll angle
    ax8 = fig2.add_subplot(224)
    states = np.array(viz_orbit.state_history)
    phi = states[:, 6] * 180 / np.pi
    ax8.plot(times, phi, 'b-', linewidth=2)
    ax8.set_xlabel('Time [s]')
    ax8.set_ylabel('Roll Angle [deg]')
    ax8.set_title('Roll Angle History')
    ax8.grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
