#!/usr/bin/env python3
"""
L1 Adaptive Guidance Orbit Tracking with GPS Uncertainty

Demonstrates L1 guidance for circular orbit tracking using noisy GPS
measurements. Compares performance with proportional control approach.

L1 Control Law:
    a_cmd = (2*V_a²/L1) * sin(arctan2(λ*e_r, L1)) + λ*V_a²/R

where:
    L1: Adaptive look-ahead distance = (1/π)*ζ*T*V_a
    e_r: Radius error (actual distance - target radius)
    λ: ±1 for CW/CCW direction
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
from src.sensors import GPSSensor, OrbitCenterEstimator
from src.visualization import SimulationVisualizer


def main():
    print("=== L1 Adaptive Guidance Orbit Tracking with GPS ===")

    # Simulation parameters
    dt = 0.01  # Time step [s]
    T_sim = 180.0  # Simulation time [s]

    # True orbit parameters (unknown to the controller)
    orbit_center_true = np.array([300, 300, -100])
    orbit_radius_true = 50.0
    orbit_direction = 'CW'

    print(f"\nTrue Orbit Parameters:")
    print(f"  Center: North={orbit_center_true[0]}m, East={orbit_center_true[1]}m, Alt={-orbit_center_true[2]}m")
    print(f"  Radius: {orbit_radius_true}m")
    print(f"  Direction: {orbit_direction}")

    # Initialize UAV
    uav = FixedWingUAV(aircraft_type='small')
    # Start outside the orbit
    initial_pos = orbit_center_true + np.array([orbit_radius_true + 80, 0, 0])
    uav.set_state([initial_pos[0], initial_pos[1], initial_pos[2], 15, 0, 0, 0, 0, 0, 0, 0, 0])

    print(f"\nInitial Position: ({initial_pos[0]:.1f}, {initial_pos[1]:.1f}, {-initial_pos[2]:.1f})")

    # Aerodynamic model
    aero = AerodynamicModel(aircraft_type='small')

    # Initialize controllers
    attitude_controller = AttitudeController()
    altitude_controller = AltitudeController()
    airspeed_controller = AirspeedController()

    # Initialize GPS sensor with realistic errors
    gps_sensor = GPSSensor(
        noise_std_horizontal=2.5,   # 2.5m horizontal noise (σ)
        noise_std_vertical=3.0,     # 3.0m vertical noise (σ)
        drift_magnitude=1.0,        # 1.0m drift
        outlier_probability=0.002,  # 0.2% outlier rate
        outlier_magnitude=15.0      # 15m outlier magnitude
    )

    # Initialize orbit center and radius estimator
    orbit_estimator = OrbitCenterEstimator(
        process_variance=0.01,      # Assume orbit center is nearly stationary
        measurement_variance=6.25   # GPS horizontal noise variance (2.5^2)
    )

    # Initialize L1 guidance
    # L1 parameters for small UAV at 15 m/s
    L1_damping = 0.707  # Critical damping
    L1_period = 15.0    # Period [s]
    l1_guidance = L1Guidance(L1_damping=L1_damping, L1_period=L1_period)

    Va_c = 15.0
    L1_distance = l1_guidance.compute_L1_distance(Va_c)

    print(f"\nL1 Guidance Parameters:")
    print(f"  Damping ratio ζ: {L1_damping:.3f}")
    print(f"  Period T: {L1_period:.1f} s")
    print(f"  L1 distance (at V_a={Va_c} m/s): {L1_distance:.2f} m")
    print(f"  Max Bank Angle: {np.rad2deg(np.pi/4):.1f}°")

    # Visualization
    viz = SimulationVisualizer()

    # Set target values
    h_c = orbit_center_true[2]  # Target altitude [m]

    # Data recording
    gps_positions = []
    true_positions = []
    estimated_centers = []
    estimated_radii = []
    radius_errors = []
    lateral_accelerations = []
    bank_angles = []
    L1_distances_recorded = []
    gps_errors = []
    center_estimation_errors = []
    radius_estimation_errors = []
    course_angles = []
    heading_errors = []

    # For orbit center estimation, collect initial data
    warmup_time = 10.0  # seconds
    warmup_samples = []

    # Simulation loop
    time = 0.0
    step = 0
    orbit_established = False

    print("\nStarting simulation...")
    print(f"Warmup phase: {warmup_time}s (collecting GPS data for initial estimate)")

    while time < T_sim:
        # Current state
        position = uav.get_position()
        phi, theta, psi = uav.get_attitude()
        Va = uav.get_airspeed()
        u, v, w = uav.get_velocity()

        # Compute course angle (ground track)
        chi = np.arctan2(v, u)

        # GPS measurement
        gps_position, valid = gps_sensor.measure(position, time)

        # Update orbit center estimate
        orbit_estimator.update(gps_position, time)
        estimated_center = orbit_estimator.get_center()
        estimated_radius = orbit_estimator.get_radius()

        # Warmup phase: just fly towards general area
        if time < warmup_time:
            # Simple navigation towards approximate center during warmup
            warmup_samples.append(gps_position)

            # Just maintain altitude and airspeed, gentle turn towards center
            approx_center = np.mean(warmup_samples, axis=0) if len(warmup_samples) > 1 else gps_position
            direction_to_center = approx_center - position
            target_heading = np.arctan2(direction_to_center[1], direction_to_center[0])

            # Gentle turn
            heading_error = target_heading - psi
            while heading_error > np.pi:
                heading_error -= 2 * np.pi
            while heading_error < -np.pi:
                heading_error += 2 * np.pi
            phi_c = np.clip(0.3 * heading_error, -np.pi/6, np.pi/6)

            # Record placeholder values
            if step % 10 == 0:
                radius_errors.append(0)
                lateral_accelerations.append(0)
                bank_angles.append(phi_c)
                L1_distances_recorded.append(L1_distance)
                course_angles.append(chi)
                heading_errors.append(0)

        else:
            # Main guidance: L1 adaptive guidance
            if not orbit_established:
                print(f"\nL1 orbit guidance engaged at t={time:.1f}s")
                print(f"  Estimated center: ({estimated_center[0]:.1f}, {estimated_center[1]:.1f}, {-estimated_center[2]:.1f})")
                print(f"  Estimated radius: {estimated_radius:.1f}m")
                orbit_established = True

            # Path parameters for L1 guidance
            path_params = {
                'center': estimated_center,
                'radius': estimated_radius,
                'direction': orbit_direction
            }

            # Compute lateral acceleration using L1 guidance
            a_cmd, eta = l1_guidance.compute_lateral_acceleration(
                position, Va, chi, path_type='orbit', path_params=path_params
            )

            # Convert to roll command
            phi_c = l1_guidance.compute_roll_command(a_cmd, Va, phi_limit=np.pi/4)

            # Calculate actual radius error for analysis
            d_vec = position[0:2] - estimated_center[0:2]
            d = np.linalg.norm(d_vec)
            e_r = d - estimated_radius

            # Compute current L1 distance
            L1_current = l1_guidance.compute_L1_distance(Va)

            # Record data
            if step % 10 == 0:
                radius_errors.append(e_r)
                lateral_accelerations.append(a_cmd)
                bank_angles.append(phi_c)
                L1_distances_recorded.append(L1_current)
                course_angles.append(chi)
                heading_errors.append(eta)

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
            true_positions.append(position.copy())
            gps_positions.append(gps_position.copy())
            estimated_centers.append(estimated_center.copy())
            estimated_radii.append(estimated_radius)

            # Compute errors
            gps_error = np.linalg.norm(gps_position - position)
            gps_errors.append(gps_error)

            center_error = np.linalg.norm(estimated_center - orbit_center_true)
            center_estimation_errors.append(center_error)

            radius_error_est = abs(estimated_radius - orbit_radius_true)
            radius_estimation_errors.append(radius_error_est)

        # Progress display
        if step % 1000 == 0:
            if time >= warmup_time:
                print(f"Time: {time:.1f}s, Pos: ({position[0]:.0f}, {position[1]:.0f}, {-position[2]:.0f}), "
                      f"R_err: {e_r:.2f}m, a_cmd: {a_cmd:.2f}m/s², φ: {np.rad2deg(phi_c):.1f}°")
            else:
                print(f"Time: {time:.1f}s (warmup), Collecting GPS data...")

        time += dt
        step += 1

    print("\n=== Simulation Complete ===")

    # Calculate final statistics (last 30 seconds)
    final_samples = int(30.0 / (dt * 10))  # 30 seconds of data
    warmup_samples_count = int(warmup_time / (dt * 10))

    if len(radius_errors) > final_samples + warmup_samples_count:
        final_radius_errors = radius_errors[-final_samples:]
        final_gps_errors = gps_errors[-final_samples:]
        final_center_errors = center_estimation_errors[-final_samples:]
        final_radius_est_errors = radius_estimation_errors[-final_samples:]

        print(f"\nFinal Performance (last 30s):")
        print(f"  Mean radius error: {np.mean(np.abs(final_radius_errors)):.2f}m")
        print(f"  RMS radius error: {np.sqrt(np.mean(np.array(final_radius_errors)**2)):.2f}m")
        print(f"  Max radius error: {np.max(np.abs(final_radius_errors)):.2f}m")
        print(f"\nGPS Performance:")
        print(f"  Mean GPS error: {np.mean(final_gps_errors):.2f}m")
        print(f"  RMS GPS error: {np.sqrt(np.mean(np.array(final_gps_errors)**2)):.2f}m")
        print(f"\nEstimation Performance:")
        print(f"  Mean center estimation error: {np.mean(final_center_errors):.2f}m")
        print(f"  Mean radius estimation error: {np.mean(final_radius_est_errors):.2f}m")

    # Plot results
    print("\nGenerating plots...")

    # Figure 1: Trajectory and errors
    fig1 = plt.figure(figsize=(15, 10))

    # 2D trajectory
    ax1 = fig1.add_subplot(221)
    true_pos_array = np.array(true_positions)
    gps_pos_array = np.array(gps_positions)
    est_centers_array = np.array(estimated_centers)

    ax1.plot(true_pos_array[:, 0], true_pos_array[:, 1], 'b-', linewidth=2, label='True trajectory', alpha=0.7)
    ax1.plot(gps_pos_array[:, 0], gps_pos_array[:, 1], 'r.', markersize=1, label='GPS measurements', alpha=0.3)

    # True orbit
    theta = np.linspace(0, 2*np.pi, 100)
    orbit_x = orbit_center_true[0] + orbit_radius_true * np.cos(theta)
    orbit_y = orbit_center_true[1] + orbit_radius_true * np.sin(theta)
    ax1.plot(orbit_x, orbit_y, 'g--', linewidth=2, label='True orbit')
    ax1.scatter([orbit_center_true[0]], [orbit_center_true[1]], c='g', s=100, marker='x', label='True center')

    # Estimated orbit (final)
    if len(estimated_centers) > 0:
        final_est_center = est_centers_array[-1]
        final_est_radius = estimated_radii[-1]
        est_orbit_x = final_est_center[0] + final_est_radius * np.cos(theta)
        est_orbit_y = final_est_center[1] + final_est_radius * np.sin(theta)
        ax1.plot(est_orbit_x, est_orbit_y, 'm:', linewidth=2, label='Estimated orbit (final)')
        ax1.scatter([final_est_center[0]], [final_est_center[1]], c='m', s=100, marker='+', label='Est. center (final)')

    ax1.set_xlabel('North [m]')
    ax1.set_ylabel('East [m]')
    ax1.set_title('2D Trajectory - L1 Adaptive Guidance with GPS')
    ax1.legend()
    ax1.grid(True)
    ax1.axis('equal')

    # Radius error over time
    ax2 = fig1.add_subplot(222)
    times = np.array(viz.time_history)
    error_times = times[warmup_samples_count:]  # Skip warmup
    radius_errors_plot = radius_errors[warmup_samples_count:]
    ax2.plot(error_times, radius_errors_plot, 'b-', linewidth=1.5, label='Radius error')
    ax2.axhline(y=0, color='k', linestyle='--', linewidth=1)
    ax2.fill_between(error_times, -2, 2, alpha=0.2, color='g', label='±2m tolerance')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Radius Error [m]')
    ax2.set_title('Radius Error History')
    ax2.legend()
    ax2.grid(True)

    # Lateral acceleration and bank angle
    ax3 = fig1.add_subplot(223)
    lateral_accel_plot = lateral_accelerations[warmup_samples_count:]
    bank_angles_plot = bank_angles[warmup_samples_count:]
    ax3_twin = ax3.twinx()

    line1 = ax3.plot(error_times, lateral_accel_plot, 'b-', linewidth=1.5, label='Lateral accel (a_cmd)')
    ax3.axhline(y=0, color='k', linestyle='--', linewidth=1)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Lateral Acceleration [m/s²]', color='b')
    ax3.tick_params(axis='y', labelcolor='b')

    line2 = ax3_twin.plot(error_times, np.rad2deg(bank_angles_plot), 'r-', linewidth=1.5, label='Bank angle (φ_c)')
    ax3_twin.set_ylabel('Bank Angle [deg]', color='r')
    ax3_twin.tick_params(axis='y', labelcolor='r')

    # Combine legends
    lines = line1 + line2
    labels = [l.get_label() for l in lines]
    ax3.legend(lines, labels, loc='upper right')
    ax3.set_title('L1 Control Outputs')
    ax3.grid(True)

    # L1 distance and heading error
    ax4 = fig1.add_subplot(224)
    L1_plot = L1_distances_recorded[warmup_samples_count:]
    heading_errors_plot = heading_errors[warmup_samples_count:]
    ax4_twin = ax4.twinx()

    line3 = ax4.plot(error_times, L1_plot, 'g-', linewidth=2, label='L1 distance')
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('L1 Distance [m]', color='g')
    ax4.tick_params(axis='y', labelcolor='g')

    line4 = ax4_twin.plot(error_times, np.rad2deg(heading_errors_plot), 'm-', linewidth=1.5, label='Heading error (η)')
    ax4_twin.axhline(y=0, color='k', linestyle='--', linewidth=1)
    ax4_twin.set_ylabel('Heading Error [deg]', color='m')
    ax4_twin.tick_params(axis='y', labelcolor='m')

    lines = line3 + line4
    labels = [l.get_label() for l in lines]
    ax4.legend(lines, labels, loc='upper right')
    ax4.set_title('L1 Parameters')
    ax4.grid(True)

    plt.tight_layout()

    # Figure 2: Comparison and analysis
    fig2 = plt.figure(figsize=(15, 10))

    # GPS error histogram
    ax5 = fig2.add_subplot(231)
    ax5.hist(gps_errors, bins=50, density=True, alpha=0.7, edgecolor='black')
    ax5.axvline(x=np.mean(gps_errors), color='r', linestyle='--', linewidth=2,
                label=f'Mean: {np.mean(gps_errors):.2f}m')
    ax5.axvline(x=2.5, color='g', linestyle=':', linewidth=2, label='Expected σ: 2.5m')
    ax5.set_xlabel('GPS Error [m]')
    ax5.set_ylabel('Probability Density')
    ax5.set_title('GPS Error Distribution')
    ax5.legend()
    ax5.grid(True)

    # Radius error histogram (steady state)
    ax6 = fig2.add_subplot(232)
    if len(final_radius_errors) > 0:
        ax6.hist(final_radius_errors, bins=30, density=True, alpha=0.7, edgecolor='black', color='blue')
        ax6.axvline(x=np.mean(final_radius_errors), color='r', linestyle='--', linewidth=2,
                   label=f'Mean: {np.mean(final_radius_errors):.2f}m')
        ax6.axvline(x=0, color='k', linestyle=':', linewidth=2, label='Perfect tracking')
        ax6.set_xlabel('Radius Error [m]')
        ax6.set_ylabel('Probability Density')
        ax6.set_title('Steady-State Radius Error Distribution (L1)')
        ax6.legend()
        ax6.grid(True)

    # Estimation errors over time
    ax7 = fig2.add_subplot(233)
    ax7.plot(times, center_estimation_errors, 'b-', linewidth=1.5, label='Center error')
    ax7.plot(times, radius_estimation_errors, 'r-', linewidth=1.5, label='Radius error')
    ax7.axvline(x=warmup_time, color='k', linestyle=':', linewidth=1, label='Guidance start')
    ax7.set_xlabel('Time [s]')
    ax7.set_ylabel('Estimation Error [m]')
    ax7.set_title('Orbit Parameter Estimation Errors')
    ax7.legend()
    ax7.grid(True)

    # 3D trajectory
    ax8 = fig2.add_subplot(234, projection='3d')
    x = true_pos_array[:, 0]
    y = true_pos_array[:, 1]
    z = -true_pos_array[:, 2]
    ax8.plot(x, y, z, 'b-', linewidth=2, label='UAV trajectory')

    # Draw orbit circle
    orbit_z = np.ones_like(theta) * (-orbit_center_true[2])
    ax8.plot(orbit_x, orbit_y, orbit_z, 'r--', linewidth=2, label='Desired orbit')
    ax8.scatter([orbit_center_true[0]], [orbit_center_true[1]], [-orbit_center_true[2]],
                c='g', s=100, label='Center')

    ax8.set_xlabel('North [m]')
    ax8.set_ylabel('East [m]')
    ax8.set_zlabel('Altitude [m]')
    ax8.set_title('3D Trajectory - L1 Guidance')
    ax8.legend()
    ax8.grid(True)

    # Course angle vs time
    ax9 = fig2.add_subplot(235)
    course_plot = course_angles[warmup_samples_count:]
    ax9.plot(error_times, np.rad2deg(course_plot), 'b-', linewidth=1.5)
    ax9.set_xlabel('Time [s]')
    ax9.set_ylabel('Course Angle [deg]')
    ax9.set_title('Ground Track Course Angle')
    ax9.grid(True)

    # GPS position error over time
    ax10 = fig2.add_subplot(236)
    ax10.plot(times, gps_errors, 'r-', linewidth=1, alpha=0.7)
    ax10.axhline(y=np.mean(gps_errors), color='b', linestyle='--', linewidth=2,
                 label=f'Mean: {np.mean(gps_errors):.2f}m')
    ax10.axhline(y=2.5, color='g', linestyle=':', linewidth=2, label='Expected σ: 2.5m')
    ax10.set_xlabel('Time [s]')
    ax10.set_ylabel('GPS Position Error [m]')
    ax10.set_title('GPS Measurement Error')
    ax10.legend()
    ax10.grid(True)

    plt.tight_layout()

    # Standard plots from visualizer
    viz.plot_states()
    viz.plot_controls()

    plt.show()

    print("\nVisualization complete!")


if __name__ == "__main__":
    main()
