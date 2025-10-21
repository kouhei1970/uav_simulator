#!/usr/bin/env python3
"""
Orbit Flight Simulation with GPS Noise

Verification of orbit guidance algorithm with realistic GPS measurements.
This simulation demonstrates:
- GPS position measurement with noise, drift, and outliers
- Orbit center estimation from noisy GPS data
- Robust orbit guidance under measurement uncertainty
- Kalman filtering for orbit center estimation
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import matplotlib.pyplot as plt
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel
from src.controller import AttitudeController, AltitudeController, AirspeedController
from src.guidance import OrbitGuidance, CoordinatedTurnGuidance
from src.visualization import SimulationVisualizer
from src.sensors import GPSSensor, OrbitCenterEstimator, OutlierDetector


def main():
    print("=" * 70)
    print("=== Orbit Flight Simulation with GPS Noise ===")
    print("=" * 70)

    # Simulation parameters
    dt = 0.01  # Time step [s]
    T_sim = 180.0  # Simulation time [s] - longer to see orbit stability

    # True orbit parameters (what we want to achieve)
    orbit_center_true = np.array([300, 300, -100])  # True orbit center [m]
    orbit_radius = 50.0  # Desired orbit radius [m] (30-100m range for small UAV)
    orbit_direction = 'CW'  # Orbit direction (CW: clockwise, CCW: counter-clockwise)

    print(f"\nTrue Orbit Parameters:")
    print(f"  Center: North={orbit_center_true[0]:.1f}m, East={orbit_center_true[1]:.1f}m, "
          f"Altitude={-orbit_center_true[2]:.1f}m")
    print(f"  Radius: {orbit_radius:.1f}m (optimal range for small UAV: 30-100m)")
    print(f"  Direction: {orbit_direction}")

    # GPS sensor configuration
    print(f"\nGPS Sensor Configuration:")
    gps_sensor = GPSSensor(
        noise_std_horizontal=2.5,      # 2.5m horizontal noise (typical consumer GPS)
        noise_std_vertical=4.0,        # 4.0m vertical noise (worse than horizontal)
        drift_magnitude=1.5,           # 1.5m slow drift
        drift_time_constant=15.0,      # 15s drift time constant
        outlier_probability=0.002,     # 0.2% chance of outlier per measurement
        outlier_magnitude=25.0,        # 25m typical outlier magnitude
        update_rate=5.0                # 5 Hz GPS update rate
    )
    print(f"  Horizontal accuracy: ±{gps_sensor.noise_std_horizontal:.1f}m (1σ)")
    print(f"  Vertical accuracy: ±{gps_sensor.noise_std_vertical:.1f}m (1σ)")
    print(f"  Update rate: {gps_sensor.update_rate:.0f} Hz")
    print(f"  Outlier probability: {gps_sensor.outlier_probability*100:.2f}%")

    # Orbit center estimator (Kalman filter)
    orbit_center_estimator = OrbitCenterEstimator(
        process_variance=0.005,     # Assume orbit center is nearly stationary
        measurement_variance=6.25   # GPS horizontal noise variance (2.5^2)
    )

    # Outlier detector
    outlier_detector = OutlierDetector(
        threshold_sigma=3.0,        # 3-sigma threshold
        window_size=15              # Use 15 recent measurements (3 seconds at 5 Hz)
    )

    # Initialize UAV (small type with 15 m/s cruise speed)
    uav = FixedWingUAV()
    # Start from outside the orbit circle
    initial_pos = orbit_center_true + np.array([orbit_radius + 50, 0, 0])
    uav.set_state([initial_pos[0], initial_pos[1], initial_pos[2], 15, 0, 0, 0, 0, 0, 0, 0, 0])

    # Aerodynamic model
    aero = AerodynamicModel()

    # Initialize controllers
    attitude_controller = AttitudeController()
    altitude_controller = AltitudeController()
    airspeed_controller = AirspeedController()

    # Initialize guidance with TRUE center (will be updated with GPS measurements)
    orbit_center_estimated = orbit_center_true.copy()
    orbit_guidance = OrbitGuidance(orbit_center_estimated, orbit_radius, direction=orbit_direction)
    turn_guidance = CoordinatedTurnGuidance(V_a=15.0)

    # Visualization
    viz = SimulationVisualizer()

    # Data logging for GPS analysis
    gps_log = {
        'time': [],
        'true_center': [],
        'measured_center': [],
        'estimated_center': [],
        'gps_position': [],
        'true_position': [],
        'radius_error': [],
        'outlier_detected': []
    }

    # Set target values
    Va_c = 15.0  # Target airspeed [m/s] (cruise speed for small UAV)

    # Simulation loop
    time = 0.0
    step = 0
    gps_outlier_count = 0
    gps_update_count = 0

    print("\n" + "=" * 70)
    print("Starting simulation...")
    print("=" * 70)
    print(f"{'Time':>6s} {'Alt':>6s} {'R_err':>7s} {'GPS_err':>8s} {'Est_err':>8s} {'Status':>12s}")
    print("-" * 70)

    while time < T_sim:
        # Get true position
        position_true = uav.get_position()
        phi, theta, psi = uav.get_attitude()
        Va = uav.get_airspeed()

        # Simulate GPS measurement of orbit center
        # In practice, orbit center would be provided by ground station or mission planner
        # Here we simulate measuring it with GPS
        orbit_center_measured, gps_updated = gps_sensor.measure(orbit_center_true, time)

        if gps_updated:
            gps_update_count += 1

            # Detect outliers
            is_outlier = outlier_detector.is_outlier(orbit_center_measured)

            if is_outlier:
                gps_outlier_count += 1
                status = "OUTLIER"
                # Don't update estimator with outliers
            else:
                # Update orbit center estimate with Kalman filter
                orbit_center_estimated = orbit_center_estimator.update(orbit_center_measured)

                # Update guidance law with estimated center
                orbit_guidance = OrbitGuidance(orbit_center_estimated, orbit_radius,
                                              direction=orbit_direction)
                status = "GPS_UPDATE"

            # Log data
            gps_log['time'].append(time)
            gps_log['true_center'].append(orbit_center_true.copy())
            gps_log['measured_center'].append(orbit_center_measured.copy())
            gps_log['estimated_center'].append(orbit_center_estimated.copy())
            gps_log['outlier_detected'].append(is_outlier)
        else:
            status = ""

        # Also measure own position with GPS (for demonstration)
        position_measured, _ = gps_sensor.measure(position_true, time)

        # Calculate guidance commands using ESTIMATED orbit center
        psi_c = orbit_guidance.compute_heading_command(position_true, k_orbit=2.5)
        h_c = orbit_center_estimated[2]

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

            # Log radius error
            radius_error = orbit_guidance.compute_radius_error(position_true)
            gps_log['radius_error'].append(radius_error)

        # Progress display (every 5 seconds)
        if step % 500 == 0:
            radius_error = orbit_guidance.compute_radius_error(position_true)
            gps_error = np.linalg.norm(orbit_center_measured[:2] - orbit_center_true[:2])
            est_error = np.linalg.norm(orbit_center_estimated[:2] - orbit_center_true[:2])

            print(f"{time:6.1f} {-position_true[2]:6.1f} {radius_error:7.1f} "
                  f"{gps_error:8.2f} {est_error:8.2f} {status:>12s}")

        time += dt
        step += 1

    print("-" * 70)
    print("Simulation complete")
    print("=" * 70)

    # Calculate final statistics
    final_position = uav.get_position()
    final_radius_error = orbit_guidance.compute_radius_error(final_position)

    print(f"\nFinal Results:")
    print(f"  Final radius error: {final_radius_error:.2f} m")
    print(f"  GPS updates: {gps_update_count}")
    print(f"  GPS outliers detected: {gps_outlier_count} ({gps_outlier_count/gps_update_count*100:.1f}%)")

    # Calculate RMS errors
    gps_errors = [np.linalg.norm(m[:2] - t[:2])
                  for m, t in zip(gps_log['measured_center'], gps_log['true_center'])]
    est_errors = [np.linalg.norm(e[:2] - t[:2])
                  for e, t in zip(gps_log['estimated_center'], gps_log['true_center'])]

    print(f"\nOrbit Center Estimation Performance:")
    print(f"  GPS measurement RMS error: {np.sqrt(np.mean(np.array(gps_errors)**2)):.2f} m")
    print(f"  Kalman filter RMS error: {np.sqrt(np.mean(np.array(est_errors)**2)):.2f} m")
    print(f"  Error reduction: {(1 - np.std(est_errors)/np.std(gps_errors))*100:.1f}%")

    # Visualize results
    print("\nGenerating plots...")

    # Standard trajectory plots
    viz.plot_3d_trajectory()
    viz.plot_2d_trajectory(orbit_center=orbit_center_true, orbit_radius=orbit_radius)
    viz.plot_states()
    viz.plot_controls()

    # GPS-specific analysis plots
    plot_gps_analysis(gps_log, orbit_center_true, orbit_radius)

    print("\nPlots displayed. Close windows to exit.")
    plt.show()


def plot_gps_analysis(gps_log, orbit_center_true, orbit_radius):
    """
    Create detailed plots of GPS performance and orbit center estimation.
    """
    times = np.array(gps_log['time'])
    measured_centers = np.array(gps_log['measured_center'])
    estimated_centers = np.array(gps_log['estimated_center'])
    outliers = np.array(gps_log['outlier_detected'])

    # Figure 1: Orbit center estimation over time
    fig1, axes = plt.subplots(3, 1, figsize=(12, 10))

    for i, label in enumerate(['North', 'East', 'Down']):
        ax = axes[i]

        # Plot measurements
        ax.plot(times, measured_centers[:, i], 'r.', alpha=0.3, markersize=3,
                label='GPS Measurement')

        # Highlight outliers
        outlier_times = times[outliers]
        outlier_values = measured_centers[outliers, i]
        if len(outlier_times) > 0:
            ax.plot(outlier_times, outlier_values, 'rx', markersize=10,
                    markeredgewidth=2, label='Outliers')

        # Plot estimate
        ax.plot(times, estimated_centers[:, i], 'b-', linewidth=2,
                label='Kalman Filter Estimate')

        # Plot true value
        ax.axhline(orbit_center_true[i], color='g', linestyle='--', linewidth=2,
                   label='True Value')

        ax.set_ylabel(f'{label} Position [m]')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best')

    axes[2].set_xlabel('Time [s]')
    axes[0].set_title('Orbit Center Estimation Over Time', fontsize=14, fontweight='bold')
    plt.tight_layout()

    # Figure 2: Horizontal position errors
    fig2, axes = plt.subplots(2, 1, figsize=(12, 8))

    # GPS measurement errors
    gps_errors_north = measured_centers[:, 0] - orbit_center_true[0]
    gps_errors_east = measured_centers[:, 1] - orbit_center_true[1]
    gps_errors_horiz = np.sqrt(gps_errors_north**2 + gps_errors_east**2)

    # Kalman filter errors
    est_errors_north = estimated_centers[:, 0] - orbit_center_true[0]
    est_errors_east = estimated_centers[:, 1] - orbit_center_true[1]
    est_errors_horiz = np.sqrt(est_errors_north**2 + est_errors_east**2)

    # Plot horizontal errors
    axes[0].plot(times, gps_errors_horiz, 'r-', alpha=0.5, linewidth=1,
                 label='GPS Measurement')
    axes[0].plot(times, est_errors_horiz, 'b-', linewidth=2,
                 label='Kalman Filter')
    axes[0].set_ylabel('Horizontal Error [m]')
    axes[0].set_title('Orbit Center Horizontal Position Error', fontsize=14, fontweight='bold')
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    # Plot 2D error scatter
    axes[1].plot(gps_errors_north, gps_errors_east, 'r.', alpha=0.3, markersize=3,
                 label='GPS Measurement')
    axes[1].plot(est_errors_north, est_errors_east, 'b.', markersize=4,
                 label='Kalman Filter')

    # Draw error circles
    circle_radii = [2.5, 5.0, 10.0]  # 1σ, 2σ, 4σ for 2.5m GPS noise
    for r in circle_radii:
        circle = plt.Circle((0, 0), r, fill=False, edgecolor='gray',
                           linestyle='--', alpha=0.5)
        axes[1].add_patch(circle)

    axes[1].set_xlabel('North Error [m]')
    axes[1].set_ylabel('East Error [m]')
    axes[1].set_title('Horizontal Position Error Distribution', fontsize=12, fontweight='bold')
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()
    axes[1].axis('equal')

    plt.tight_layout()

    # Figure 3: Orbit tracking performance
    if len(gps_log['radius_error']) > 0:
        fig3, ax = plt.subplots(figsize=(12, 6))

        radius_errors = np.array(gps_log['radius_error'])
        time_radius = np.linspace(0, times[-1], len(radius_errors))

        ax.plot(time_radius, radius_errors, 'b-', linewidth=2)
        ax.axhline(0, color='g', linestyle='--', linewidth=2, alpha=0.5,
                   label='Perfect Orbit')
        ax.set_xlabel('Time [s]')
        ax.set_ylabel('Radius Error [m]')
        ax.set_title('Orbit Radius Tracking Error', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)

        # Add statistics
        rms_radius_error = np.sqrt(np.mean(radius_errors**2))
        ax.text(0.02, 0.98, f'RMS Error: {rms_radius_error:.2f} m',
                transform=ax.transAxes, verticalalignment='top',
                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

        plt.tight_layout()


if __name__ == "__main__":
    main()
