"""
Integration tests for UAV simulator

Tests the integration of multiple modules working together:
- Dynamics + Aerodynamics
- Dynamics + Aerodynamics + Control
- Full simulation loop with guidance
- Sensor-in-the-loop simulations
"""

import pytest
import numpy as np
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel
from src.controller import AttitudeController, AltitudeController, AirspeedController
from src.guidance import OrbitGuidance, ProportionalOrbitGuidance, L1Guidance
from src.sensors import GPSSensor, OrbitCenterEstimator


class TestDynamicsAerodynamicsIntegration:
    """Test dynamics and aerodynamics working together"""

    def test_basic_simulation_step(self, default_uav, default_aero):
        """Test basic simulation step with dynamics and aerodynamics"""
        # Initial state
        state = np.array([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        # Control input
        control = np.array([0, 0, 0, 0.5])
        default_uav.set_control(control)

        # Compute aero forces
        forces_moments = default_aero.compute_forces_moments(default_uav, control)

        # Update dynamics
        default_uav.update(0.01, forces_moments)

        # State should change
        new_state = default_uav.get_state()
        assert not np.allclose(new_state, state)

        # State should not have NaN
        assert not np.any(np.isnan(new_state))

    def test_simulation_maintains_reasonable_values(
        self, default_uav, default_aero
    ):
        """Test that simulation maintains reasonable values over time"""
        state = np.array([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        control = np.array([0, 0, 0, 0.5])

        # Run for 10 seconds
        dt = 0.01
        for _ in range(1000):
            default_uav.set_control(control)
            forces_moments = default_aero.compute_forces_moments(default_uav, control)
            default_uav.update(dt, forces_moments)

            # Check values remain reasonable
            current_state = default_uav.get_state()
            assert not np.any(np.isnan(current_state))
            assert not np.any(np.isinf(current_state))

            # Velocity should be reasonable
            Va = default_uav.get_airspeed()
            assert 0 < Va < 100  # m/s

    def test_energy_change_reasonable(self, default_uav, default_aero):
        """Test that energy changes are reasonable"""
        state = np.array([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        # Initial energy
        Va_0 = default_uav.get_airspeed()
        h_0 = -default_uav.get_position()[2]
        E_0 = 0.5 * default_uav.mass * Va_0**2 + default_uav.mass * 9.81 * h_0

        # Run with constant throttle
        control = np.array([0, 0, 0, 0.5])
        dt = 0.01

        for _ in range(500):
            default_uav.set_control(control)
            forces_moments = default_aero.compute_forces_moments(default_uav, control)
            default_uav.update(dt, forces_moments)

        # Final energy
        Va_f = default_uav.get_airspeed()
        h_f = -default_uav.get_position()[2]
        E_f = 0.5 * default_uav.mass * Va_f**2 + default_uav.mass * 9.81 * h_f

        # Energy should have changed (thrust adds energy, drag removes)
        # But should be same order of magnitude
        assert 0.1 * E_0 < E_f < 10 * E_0


class TestControlledFlight:
    """Test controlled flight scenarios"""

    def test_altitude_hold(
        self, default_uav, default_aero,
        altitude_controller, airspeed_controller, attitude_controller
    ):
        """Test altitude hold control"""
        # Start at 100m
        state = np.array([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        h_cmd = -120  # Command 120m altitude
        Va_cmd = 25.0
        dt = 0.01

        # Run for 20 seconds
        for _ in range(2000):
            # Compute control commands
            theta_cmd = altitude_controller.compute_pitch_command(
                default_uav, h_cmd, dt
            )
            delta_t = airspeed_controller.compute_throttle_command(
                default_uav, Va_cmd, dt
            )
            delta_a, delta_e, delta_r = attitude_controller.compute_control(
                default_uav, 0, theta_cmd, dt
            )

            # Apply control
            control = np.array([delta_e, delta_a, delta_r, delta_t])
            default_uav.set_control(control)

            # Update dynamics
            forces_moments = default_aero.compute_forces_moments(default_uav, control)
            default_uav.update(dt, forces_moments)

        # Check altitude is closer to commanded value
        final_altitude = -default_uav.get_position()[2]
        # Should be within 20m of commanded (generous tolerance for this test)
        assert abs(final_altitude - 120) < 20

    def test_level_flight_stability(
        self, default_uav, default_aero, attitude_controller
    ):
        """Test that level flight is maintained with attitude control"""
        state = np.array([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        dt = 0.01
        altitudes = []

        # Run for 10 seconds
        for _ in range(1000):
            # Command level flight
            delta_a, delta_e, delta_r = attitude_controller.compute_control(
                default_uav, 0, 0, dt
            )

            control = np.array([delta_e, delta_a, delta_r, 0.5])
            default_uav.set_control(control)

            forces_moments = default_aero.compute_forces_moments(default_uav, control)
            default_uav.update(dt, forces_moments)

            altitudes.append(-default_uav.get_position()[2])

        # Altitude should not drift excessively
        altitude_change = abs(altitudes[-1] - altitudes[0])
        assert altitude_change < 50  # Within 50m over 10 seconds


class TestGuidedFlight:
    """Test flight with guidance laws"""

    def test_orbit_tracking_proportional(
        self, default_uav, default_aero,
        attitude_controller, altitude_controller, airspeed_controller
    ):
        """Test orbit tracking with proportional guidance"""
        # Set up orbit
        orbit_center = np.array([200, 200, -100])
        orbit_radius = 50.0

        # Start outside orbit
        state = np.array([200 + 80, 200, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        # Guidance
        prop_guidance = ProportionalOrbitGuidance(K_p=0.08, phi_max=np.pi/4)

        dt = 0.01
        radius_errors = []

        # Run for 60 seconds
        for _ in range(6000):
            position = default_uav.get_position()
            Va = default_uav.get_airspeed()

            # Guidance command
            phi_cmd, e_r, d = prop_guidance.compute_roll_command(
                position, orbit_center, orbit_radius, Va, 'CW'
            )

            # Altitude and airspeed control
            theta_cmd = altitude_controller.compute_pitch_command(
                default_uav, orbit_center[2], dt
            )
            delta_t = airspeed_controller.compute_throttle_command(
                default_uav, 25.0, dt
            )

            # Attitude control
            delta_a, delta_e, delta_r = attitude_controller.compute_control(
                default_uav, phi_cmd, theta_cmd, dt
            )

            # Apply control
            control = np.array([delta_e, delta_a, delta_r, delta_t])
            default_uav.set_control(control)

            # Update
            forces_moments = default_aero.compute_forces_moments(default_uav, control)
            default_uav.update(dt, forces_moments)

            if _ % 100 == 0:
                radius_errors.append(abs(e_r))

        # After 60 seconds, should be tracking orbit reasonably well
        final_error = np.mean(radius_errors[-10:])
        assert final_error < 15.0  # Within 15m (generous for integration test)

    def test_orbit_tracking_l1(
        self, default_uav, default_aero,
        attitude_controller, altitude_controller, airspeed_controller
    ):
        """Test orbit tracking with L1 guidance"""
        orbit_center = np.array([200, 200, -100])
        orbit_radius = 50.0

        state = np.array([200 + 80, 200, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        l1_guidance = L1Guidance(damping=0.707, period=15.0)

        dt = 0.01
        radius_errors = []

        # Run for 60 seconds
        for _ in range(6000):
            position = default_uav.get_position()
            Va = default_uav.get_airspeed()
            phi, theta, psi = default_uav.get_attitude()

            # Course angle (simplified as heading)
            chi = psi

            # L1 guidance
            path_params = {
                'center': orbit_center,
                'radius': orbit_radius,
                'direction': 'CW'
            }

            a_cmd, eta = l1_guidance.compute_lateral_acceleration(
                position, Va, chi, 'orbit', path_params
            )
            phi_cmd = l1_guidance.compute_roll_command(a_cmd, Va, np.pi/4)

            # Other controllers
            theta_cmd = altitude_controller.compute_pitch_command(
                default_uav, orbit_center[2], dt
            )
            delta_t = airspeed_controller.compute_throttle_command(
                default_uav, 25.0, dt
            )

            delta_a, delta_e, delta_r = attitude_controller.compute_control(
                default_uav, phi_cmd, theta_cmd, dt
            )

            control = np.array([delta_e, delta_a, delta_r, delta_t])
            default_uav.set_control(control)

            forces_moments = default_aero.compute_forces_moments(default_uav, control)
            default_uav.update(dt, forces_moments)

            # Track radius error
            if _ % 100 == 0:
                distance = np.linalg.norm(position[0:2] - orbit_center[0:2])
                radius_errors.append(abs(distance - orbit_radius))

        # Should achieve reasonable tracking
        final_error = np.mean(radius_errors[-10:])
        assert final_error < 15.0


class TestSensorInTheLoop:
    """Test simulations with sensors in the loop"""

    def test_gps_based_orbit_tracking(
        self, default_uav, default_aero,
        attitude_controller, altitude_controller, airspeed_controller
    ):
        """Test orbit tracking using GPS-based orbit estimation"""
        # True orbit parameters
        true_center = np.array([200, 200, -100])
        true_radius = 50.0

        # Start on orbit
        state = np.array([200 + 50, 200, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        # GPS sensor
        gps = GPSSensor(
            noise_std_horizontal=2.5,
            noise_std_vertical=3.0,
            drift_magnitude=1.0,
            outlier_probability=0.002,
            random_seed=42
        )

        # Orbit estimator
        estimator = OrbitCenterEstimator(
            process_variance=0.01,
            measurement_variance=6.25
        )

        # Guidance
        prop_guidance = ProportionalOrbitGuidance(K_p=0.08, phi_max=np.pi/4)

        dt = 0.01
        warmup_time = 10.0  # seconds
        time = 0.0

        # Run for 60 seconds
        for step in range(6000):
            time = step * dt
            position = default_uav.get_position()
            Va = default_uav.get_airspeed()

            # GPS measurement
            gps_pos, valid = gps.measure(position, time)

            # Update orbit estimate
            if valid:
                estimator.update(gps_pos, time)

            # After warmup, use estimated parameters
            if time > warmup_time:
                estimated_center = estimator.get_center()
                estimated_radius = estimator.get_radius()

                # Guidance command
                phi_cmd, e_r, d = prop_guidance.compute_roll_command(
                    position, estimated_center, estimated_radius, Va, 'CW'
                )
            else:
                # During warmup, just maintain level flight
                phi_cmd = 0

            # Other controllers
            theta_cmd = altitude_controller.compute_pitch_command(
                default_uav, true_center[2], dt
            )
            delta_t = airspeed_controller.compute_throttle_command(
                default_uav, 25.0, dt
            )

            delta_a, delta_e, delta_r = attitude_controller.compute_control(
                default_uav, phi_cmd, theta_cmd, dt
            )

            control = np.array([delta_e, delta_a, delta_r, delta_t])
            default_uav.set_control(control)

            forces_moments = default_aero.compute_forces_moments(default_uav, control)
            default_uav.update(dt, forces_moments)

        # After 60 seconds with GPS, should have reasonable orbit parameters
        final_center = estimator.get_center()
        final_radius = estimator.get_radius()

        # Center estimate should be reasonable (within 20m given GPS noise)
        center_error = np.linalg.norm(final_center - true_center)
        assert center_error < 20.0

        # Radius estimate should be reasonable
        radius_error = abs(final_radius - true_radius)
        assert radius_error < 20.0


class TestNumericalStability:
    """Test numerical stability over long simulations"""

    def test_long_duration_simulation(
        self, default_uav, default_aero, attitude_controller
    ):
        """Test that simulation remains stable over long duration"""
        state = np.array([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        dt = 0.01

        # Run for 120 seconds (12000 steps)
        for _ in range(12000):
            delta_a, delta_e, delta_r = attitude_controller.compute_control(
                default_uav, 0, 0, dt
            )

            control = np.array([delta_e, delta_a, delta_r, 0.5])
            default_uav.set_control(control)

            forces_moments = default_aero.compute_forces_moments(default_uav, control)
            default_uav.update(dt, forces_moments)

            # Check for numerical issues
            current_state = default_uav.get_state()
            assert not np.any(np.isnan(current_state))
            assert not np.any(np.isinf(current_state))

    def test_extreme_maneuvers_stable(
        self, default_uav, default_aero, attitude_controller
    ):
        """Test stability during aggressive maneuvers"""
        state = np.array([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        dt = 0.01

        # Command aggressive bank angles
        for i in range(3000):
            # Alternate between left and right banks
            if (i // 500) % 2 == 0:
                phi_cmd = np.deg2rad(40)
            else:
                phi_cmd = np.deg2rad(-40)

            delta_a, delta_e, delta_r = attitude_controller.compute_control(
                default_uav, phi_cmd, 0, dt
            )

            control = np.array([delta_e, delta_a, delta_r, 0.6])
            default_uav.set_control(control)

            forces_moments = default_aero.compute_forces_moments(default_uav, control)
            default_uav.update(dt, forces_moments)

            # Should remain stable
            current_state = default_uav.get_state()
            assert not np.any(np.isnan(current_state))


class TestMultiAircraftTypes:
    """Test that different aircraft types work correctly"""

    def test_small_medium_large_aircraft(self):
        """Test that all aircraft types can be simulated"""
        aircraft_types = ['small', 'medium', 'large']

        for aircraft_type in aircraft_types:
            uav = FixedWingUAV(aircraft_type=aircraft_type)
            aero = AerodynamicModel(aircraft_type=aircraft_type)

            state = np.array([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
            uav.set_state(state)

            control = np.array([0, 0, 0, 0.5])

            # Run brief simulation
            for _ in range(100):
                uav.set_control(control)
                forces_moments = aero.compute_forces_moments(uav, control)
                uav.update(0.01, forces_moments)

                # Should work without errors
                assert not np.any(np.isnan(uav.get_state()))

    def test_aircraft_performance_differences(self):
        """Test that different aircraft have different performance"""
        small_uav = FixedWingUAV(aircraft_type='small')
        large_uav = FixedWingUAV(aircraft_type='large')

        # Mass should be different
        assert small_uav.mass < large_uav.mass

        # Moments of inertia should be different
        assert small_uav.Jx < large_uav.Jx
        assert small_uav.Jy < large_uav.Jy
        assert small_uav.Jz < large_uav.Jz
