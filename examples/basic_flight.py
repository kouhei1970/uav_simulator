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

    # Set up trim condition for stable level flight at 15 m/s
    print("Setting up trim condition for level flight at 15 m/s...")
    Va_trim = 15.0  # Target cruise speed for small UAV
    altitude_trim = -100.0

    # Trim values optimized for 15 m/s cruise with minimal altitude change
    # Determined through trim_search_fine.py:
    # - Altitude change rate: -0.017 m/s (nearly zero)
    # - Airspeed: 15.00 m/s (exact match)
    throttle_trim = 0.305  # Optimized throttle for minimal altitude change at 15 m/s
    alpha_trim = np.deg2rad(1.9)  # Required angle of attack
    pitch_trim = np.deg2rad(5.0)  # Pitch = alpha + flight path angle (adjusted for level flight)

    # Calculate velocity components for desired angle of attack
    # alpha = arctan(w/u), so w = u * tan(alpha)
    u_trim = Va_trim * np.cos(alpha_trim)
    w_trim = Va_trim * np.sin(alpha_trim)

    # Elevator trim optimized through trim search
    # Previous calculation: -(C_m_0 + C_m_alpha*alpha) / C_m_delta_e ≈ -4.33 deg
    # Optimized value from trim_search_fine.py: -3.95 deg
    elevator_trim = np.deg2rad(-3.95)  # Optimized for minimal altitude change

    # Set initial state at trim condition
    uav.set_state([
        0, 0, altitude_trim,  # Position
        u_trim, 0, w_trim,  # Velocity (u, v, w) for correct angle of attack
        0, pitch_trim, 0,  # Attitude (phi, theta, psi)
        0, 0, 0  # Angular velocity (p, q, r)
    ])

    # Initial trim control
    control_trim = np.array([elevator_trim, 0.0, 0.0, throttle_trim])  # [elevator, aileron, rudder, throttle]

    print(f"Trim conditions:")
    print(f"  Throttle: {throttle_trim:.3f}")
    print(f"  Elevator: {np.rad2deg(elevator_trim):.2f} deg")
    print(f"  Pitch angle: {np.rad2deg(pitch_trim):.2f} deg")
    print(f"  Angle of attack: {np.rad2deg(alpha_trim):.2f} deg")
    print(f"  Velocity: u={u_trim:.2f} m/s, w={w_trim:.2f} m/s")

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
    print("Testing trim condition with constant controls (no active feedback)")

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
        if time >= 40.0:
            # Gradually change target altitude to 105m (very gentle climb at low speed)
            h_c = -105.0

        # TEST: Use trim controls for entire flight to verify trim correctness
        delta_e = control_trim[0]
        delta_a = 0.0
        delta_r = 0.0
        delta_t = control_trim[3]

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
