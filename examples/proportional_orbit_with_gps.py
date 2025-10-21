#!/usr/bin/env python3
"""
Proportional Control Orbit Guidance with GPS Uncertainty

Demonstrates robust circular orbit tracking using proportional control
with noisy GPS measurements of orbit center and radius.

Control Law:
    φ_c = φ_ff + K_p * e_r

where:
    φ_ff: Feedforward bank angle for steady turn
    e_r: Radius error (actual distance - target radius)
    K_p: Proportional gain
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import matplotlib.pyplot as plt
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel
from src.controller import AttitudeController, AltitudeController, AirspeedController
from src.guidance import ProportionalOrbitGuidance
from src.sensors import GPSSensor, OrbitCenterEstimator
from src.visualization import SimulationVisualizer


def main():
    print("=== Proportional Control Orbit Guidance with GPS ===")

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
        process_noise=0.1,
        measurement_noise=2.5,
        initial_center_uncertainty=10.0
    )

    # Initialize proportional orbit guidance
    # Tune K_p for desired response
    # Higher K_p → Faster convergence but more aggressive
    # Lower K_p → Slower convergence but smoother
    K_p = 0.08  # [rad/m]
    prop_guidance = ProportionalOrbitGuidance(K_p=K_p, phi_max=np.pi/4)

    print(f"\nGuidance Parameters:")
    print(f"  Proportional Gain K_p: {K_p:.3f} rad/m")
    print(f"  Max Bank Angle: {np.rad2deg(np.pi/4):.1f}°")

    # Analyze stability
    Va_c = 15.0
    eigenvalue, time_constant, damping = prop_guidance.analyze_stability(Va_c, orbit_radius_true)
    print(f"\nStability Analysis:")
    print(f"  Eigenvalue: {eigenvalue:.4f} (negative = stable)")
    print(f"  Time Constant: {time_constant:.2f} s")
    print(f"  Expected 63% convergence time: {time_constant:.2f} s")
    print(f"  Expected 95% convergence time: {3*time_constant:.2f} s")

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
    bank_angles = []
    feedforward_angles = []
    proportional_angles = []
    gps_errors = []
    center_estimation_errors = []
    radius_estimation_errors = []

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

        # GPS measurement
        gps_position = gps_sensor.measure(position, time)

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

        else:
            # Main guidance: proportional control
            if not orbit_established:
                print(f"\nOrbit guidance engaged at t={time:.1f}s")
                print(f"  Estimated center: ({estimated_center[0]:.1f}, {estimated_center[1]:.1f}, {-estimated_center[2]:.1f})")
                print(f"  Estimated radius: {estimated_radius:.1f}m")
                orbit_established = True

            # Compute roll command using proportional control
            phi_c, e_r, d = prop_guidance.compute_roll_command(
                position, estimated_center, estimated_radius, Va, orbit_direction
            )

            # For analysis, compute feedforward and proportional components separately
            phi_ff = prop_guidance.compute_feedforward_roll(Va, estimated_radius, orbit_direction)
            phi_p = phi_c - phi_ff

            # Record data
            if step % 10 == 0:
                radius_errors.append(e_r)
                bank_angles.append(phi_c)
                feedforward_angles.append(phi_ff)
                proportional_angles.append(phi_p)

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

            radius_error = abs(estimated_radius - orbit_radius_true)
            radius_estimation_errors.append(radius_error)

        # Progress display
        if step % 1000 == 0:
            if time >= warmup_time:
                print(f"Time: {time:.1f}s, Pos: ({position[0]:.0f}, {position[1]:.0f}, {-position[2]:.0f}), "
                      f"R_err: {e_r:.2f}m, φ: {np.rad2deg(phi_c):.1f}°")
            else:
                print(f"Time: {time:.1f}s (warmup), Collecting GPS data...")

        time += dt
        step += 1

    print("\n=== Simulation Complete ===")

    # Calculate final statistics (last 30 seconds)
    final_samples = int(30.0 / (dt * 10))  # 30 seconds of data
    if len(radius_errors) > final_samples:
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

    # Figure 1: Trajectory comparison
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
    ax1.set_title('2D Trajectory - Proportional Control Orbit Guidance')
    ax1.legend()
    ax1.grid(True)
    ax1.axis('equal')

    # Radius error over time
    ax2 = fig1.add_subplot(222)
    times = np.array(viz.time_history)
    error_times = times[int(warmup_time/(dt*10)):]  # Skip warmup
    ax2.plot(error_times, radius_errors, 'b-', linewidth=1.5, label='Radius error')
    ax2.axhline(y=0, color='k', linestyle='--', linewidth=1)
    ax2.fill_between(error_times, -2, 2, alpha=0.2, color='g', label='±2m tolerance')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Radius Error [m]')
    ax2.set_title('Radius Error History')
    ax2.legend()
    ax2.grid(True)

    # Bank angle components
    ax3 = fig1.add_subplot(223)
    ax3.plot(error_times, np.rad2deg(feedforward_angles), 'g-', linewidth=1.5, label='Feedforward (φ_ff)')
    ax3.plot(error_times, np.rad2deg(proportional_angles), 'r-', linewidth=1.5, label='Proportional (K_p*e_r)')
    ax3.plot(error_times, np.rad2deg(bank_angles), 'b-', linewidth=2, label='Total (φ_c)', alpha=0.7)
    ax3.axhline(y=0, color='k', linestyle='--', linewidth=1)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Bank Angle [deg]')
    ax3.set_title('Bank Angle Components')
    ax3.legend()
    ax3.grid(True)

    # Center estimation error
    ax4 = fig1.add_subplot(224)
    ax4.plot(times, center_estimation_errors, 'b-', linewidth=1.5, label='Center error')
    ax4.plot(times, radius_estimation_errors, 'r-', linewidth=1.5, label='Radius error')
    ax4.axvline(x=warmup_time, color='k', linestyle=':', linewidth=1, label='Guidance start')
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Estimation Error [m]')
    ax4.set_title('Orbit Parameter Estimation Errors')
    ax4.legend()
    ax4.grid(True)

    plt.tight_layout()

    # Figure 2: GPS error analysis
    fig2 = plt.figure(figsize=(15, 5))

    # GPS position error
    ax5 = fig2.add_subplot(131)
    ax5.plot(times, gps_errors, 'r-', linewidth=1, alpha=0.7, label='GPS error')
    ax5.axhline(y=np.mean(gps_errors), color='b', linestyle='--', linewidth=2, label=f'Mean: {np.mean(gps_errors):.2f}m')
    ax5.axhline(y=2.5, color='g', linestyle=':', linewidth=2, label='Expected σ: 2.5m')
    ax5.set_xlabel('Time [s]')
    ax5.set_ylabel('GPS Position Error [m]')
    ax5.set_title('GPS Measurement Error')
    ax5.legend()
    ax5.grid(True)

    # Error histogram
    ax6 = fig2.add_subplot(132)
    ax6.hist(gps_errors, bins=50, density=True, alpha=0.7, edgecolor='black')
    ax6.axvline(x=np.mean(gps_errors), color='r', linestyle='--', linewidth=2, label=f'Mean: {np.mean(gps_errors):.2f}m')
    ax6.set_xlabel('GPS Error [m]')
    ax6.set_ylabel('Probability Density')
    ax6.set_title('GPS Error Distribution')
    ax6.legend()
    ax6.grid(True)

    # Radius error histogram (steady state)
    ax7 = fig2.add_subplot(133)
    if len(radius_errors) > final_samples:
        ax7.hist(final_radius_errors, bins=30, density=True, alpha=0.7, edgecolor='black', color='green')
        ax7.axvline(x=np.mean(final_radius_errors), color='r', linestyle='--', linewidth=2,
                   label=f'Mean: {np.mean(final_radius_errors):.2f}m')
        ax7.axvline(x=0, color='k', linestyle=':', linewidth=2, label='Perfect tracking')
        ax7.set_xlabel('Radius Error [m]')
        ax7.set_ylabel('Probability Density')
        ax7.set_title('Steady-State Radius Error Distribution')
        ax7.legend()
        ax7.grid(True)

    plt.tight_layout()

    # Standard plots from visualizer
    viz.plot_3d_trajectory()
    viz.plot_states()
    viz.plot_controls()

    plt.show()

    print("\nVisualization complete!")


if __name__ == "__main__":
    main()
