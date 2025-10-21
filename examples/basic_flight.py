#!/usr/bin/env python3
"""
Basic Flight Simulation

Verification of basic attitude and altitude control with conservative gains
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

    # Initialize UAV with stable cruise speed
    uav = FixedWingUAV()

    # Aerodynamic model
    aero = AerodynamicModel()

    # Set up trim condition for stable level flight
    print("Setting up trim condition for level flight at 25 m/s...")
    Va_trim = 25.0
    altitude_trim = -100.0

    # Typical trim values for small UAV in level flight
    throttle_trim = 0.55  # Moderate-high throttle for sustained flight
    pitch_trim = np.deg2rad(1.0)  # Slight nose-up attitude

    # Set initial state at near-trim condition
    uav.set_state([
        0, 0, altitude_trim,  # Position
        Va_trim, 0, 0,  # Velocity (u, v, w)
        0, pitch_trim, 0,  # Attitude (phi, theta, psi)
        0, 0, 0  # Angular velocity (p, q, r)
    ])

    # Initial trim control
    control_trim = np.array([0.0, 0.0, 0.0, throttle_trim])  # [elevator, aileron, rudder, throttle]

    print(f"Trim throttle: {throttle_trim:.3f}, Trim pitch: {np.rad2deg(pitch_trim):.2f} deg")

    # Initialize controllers
    attitude_controller = AttitudeController()
    altitude_controller = AltitudeController()
    airspeed_controller = AirspeedController()

    # Visualization
    viz = SimulationVisualizer()

    # Set initial target values - maintain current state first
    h_c = altitude_trim  # Start at current altitude [m] (NED frame)
    Va_c = Va_trim   # Target airspeed [m/s] - maintain cruise speed

    # Simulation loop
    time = 0.0
    step = 0

    print(f"Initial position: {uav.get_position()}")
    print(f"Initial target altitude: {-h_c} m")
    print(f"Target airspeed: {Va_c} m/s")
    print("Starting simulation...")
    print("Phase 1 (0-30s): Maintain current altitude with trim controls")
    print("Phase 2 (30-60s): Gentle climb to 110m")

    while time < T_sim:
        # Current state
        position = uav.get_position()
        phi, theta, psi = uav.get_attitude()
        Va = uav.get_airspeed()

        # Check for numerical stability
        if np.isnan(Va) or np.isinf(Va) or Va > 100.0 or Va < 5.0:
            print(f"WARNING: Numerical instability detected at t={time:.1f}s")
            print(f"Airspeed: {Va:.2f} m/s, Altitude: {-position[2]:.2f} m")
            break

        # Gradual altitude change after stabilization phase
        if time >= 30.0:
            # Gradually change target altitude to 110m (gentle climb)
            h_c = -110.0

        # Altitude controller - generate pitch command
        theta_c = altitude_controller.compute_pitch_command(uav, h_c, dt)

        # Limit pitch command to prevent aggressive maneuvers
        theta_c = np.clip(theta_c, -np.deg2rad(15), np.deg2rad(15))

        # Generate control surface commands from attitude controller
        phi_c = 0.0  # Level flight
        delta_a, delta_e, delta_r = attitude_controller.compute_control(uav, phi_c, theta_c, dt)

        # Throttle control to maintain airspeed
        delta_t_airspeed = airspeed_controller.compute_throttle_command(uav, Va_c, dt)

        # Use trim throttle as baseline with airspeed correction
        if time < 30.0:
            # Phase 1: Maintain trim throttle with small airspeed corrections
            delta_t = control_trim[3] + 0.5 * (delta_t_airspeed - control_trim[3])
        else:
            # Phase 2: Use airspeed controller for climb
            delta_t = delta_t_airspeed

        # Clamp throttle to safe range
        delta_t = np.clip(delta_t, 0.3, 0.8)

        # Set control inputs with safety limits
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
            altitude_error = -position[2] - (-h_c)
            airspeed_error = Va - Va_c
            print(f"Time: {time:.1f}s, Alt: {-position[2]:.1f}m (err: {altitude_error:+.1f}m), "
                  f"Va: {Va:.1f}m/s (err: {airspeed_error:+.1f}m/s), "
                  f"Throttle: {delta_t:.2f}")

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
