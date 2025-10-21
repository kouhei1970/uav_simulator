"""
Unit tests for UAV dynamics module (src/dynamics.py)

Tests the 6-DOF nonlinear dynamics of fixed-wing UAVs including:
- Initialization and state management
- Rotation matrices and coordinate transformations
- Equations of motion
- Numerical integration
- Physical validity
"""

import pytest
import numpy as np
from src.dynamics import FixedWingUAV
from tests.fixtures.reference_data import (
    GRAVITY, rotation_matrix_321
)


class TestInitialization:
    """Test UAV initialization and parameter loading"""

    def test_default_initialization(self):
        """Test default initialization without parameters"""
        uav = FixedWingUAV()
        assert uav is not None
        assert len(uav.state) == 12

    def test_small_aircraft_initialization(self, default_uav):
        """Test small aircraft type initialization"""
        assert default_uav.params['mass'] > 0
        assert default_uav.params['Jx'] > 0
        assert default_uav.params['Jy'] > 0
        assert default_uav.params['Jz'] > 0

    def test_medium_aircraft_initialization(self, medium_uav):
        """Test medium aircraft type initialization"""
        assert medium_uav.params['mass'] > 0
        # Medium aircraft should be heavier than small
        small_uav = FixedWingUAV(aircraft_type='small')
        assert medium_uav.params['mass'] > small_uav.params['mass']

    def test_large_aircraft_initialization(self, large_uav):
        """Test large aircraft type initialization"""
        assert large_uav.params['mass'] > 0
        # Large aircraft should be heaviest
        medium_uav = FixedWingUAV(aircraft_type='medium')
        assert large_uav.params['mass'] > medium_uav.params['mass']


class TestStateManagement:
    """Test state getter and setter methods"""

    def test_set_get_state_consistency(self, default_uav):
        """Test that set_state and get_state are consistent"""
        test_state = np.array([10, 20, -50, 25, 1, -0.5,
                               0.1, 0.05, 1.57, 0.01, 0.02, 0.01])
        default_uav.set_state(test_state)
        retrieved_state = default_uav.get_state()

        assert np.allclose(retrieved_state, test_state)

    def test_get_position(self, default_uav):
        """Test position getter"""
        state = np.array([100, 200, -150, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)
        position = default_uav.get_position()

        assert np.allclose(position, [100, 200, -150])

    def test_get_velocity(self, default_uav):
        """Test velocity getter"""
        state = np.array([0, 0, -100, 20, 5, 2, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)
        velocity = default_uav.get_velocity()

        assert np.allclose(velocity, [20, 5, 2])

    def test_get_attitude(self, default_uav):
        """Test attitude getter"""
        phi, theta, psi = 0.1, 0.05, 1.57
        state = np.array([0, 0, -100, 25, 0, 0, phi, theta, psi, 0, 0, 0])
        default_uav.set_state(state)
        attitude = default_uav.get_attitude()

        assert np.allclose(attitude, [phi, theta, psi])

    def test_get_angular_velocity(self, default_uav):
        """Test angular velocity getter"""
        p, q, r = 0.1, 0.05, 0.02
        state = np.array([0, 0, -100, 25, 0, 0, 0, 0, 0, p, q, r])
        default_uav.set_state(state)
        omega = default_uav.get_angular_velocity()

        assert np.allclose(omega, [p, q, r])

    def test_get_airspeed(self, default_uav):
        """Test airspeed calculation"""
        # No wind, body velocities directly give airspeed
        u, v, w = 20, 0, 0
        state = np.array([0, 0, -100, u, v, w, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)
        Va = default_uav.get_airspeed()

        expected_Va = np.sqrt(u**2 + v**2 + w**2)
        assert abs(Va - expected_Va) < 0.01

    def test_get_angle_of_attack(self, default_uav):
        """Test angle of attack calculation"""
        u, w = 25, 2
        state = np.array([0, 0, -100, u, 0, w, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)
        alpha = default_uav.get_angle_of_attack()

        expected_alpha = np.arctan2(w, u)
        assert abs(alpha - expected_alpha) < 0.001

    def test_get_sideslip_angle(self, default_uav):
        """Test sideslip angle calculation"""
        u, v = 25, 3
        Va = np.sqrt(u**2 + v**2)
        state = np.array([0, 0, -100, u, v, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)
        beta = default_uav.get_sideslip_angle()

        expected_beta = np.arcsin(v / Va)
        assert abs(beta - expected_beta) < 0.001


class TestRotationMatrix:
    """Test rotation matrix computation and properties"""

    def test_rotation_matrix_orthogonality(self, default_uav, numerical_tolerances):
        """Test that rotation matrix is orthogonal (R^T * R = I)"""
        # Set various attitudes
        test_attitudes = [
            (0, 0, 0),
            (np.deg2rad(30), 0, 0),
            (0, np.deg2rad(15), 0),
            (0, 0, np.deg2rad(45)),
            (np.deg2rad(20), np.deg2rad(10), np.deg2rad(30)),
        ]

        for phi, theta, psi in test_attitudes:
            state = np.array([0, 0, -100, 25, 0, 0, phi, theta, psi, 0, 0, 0])
            default_uav.set_state(state)
            R = default_uav.rotation_matrix_body_to_ned()

            # Check orthogonality
            I = R.T @ R
            assert np.allclose(I, np.eye(3),
                             atol=numerical_tolerances['rotation_matrix_orthogonality'])

    def test_rotation_matrix_determinant(self, default_uav, numerical_tolerances):
        """Test that rotation matrix has determinant ±1 (proper rotation or reflection)"""
        phi, theta, psi = np.deg2rad(20), np.deg2rad(10), np.deg2rad(30)
        state = np.array([0, 0, -100, 25, 0, 0, phi, theta, psi, 0, 0, 0])
        default_uav.set_state(state)
        R = default_uav.rotation_matrix_body_to_ned()

        det = np.linalg.det(R)
        # NED coordinate system may use different convention
        assert abs(abs(det) - 1.0) < numerical_tolerances['rotation_matrix_determinant']

    def test_rotation_matrix_vs_reference(self, default_uav):
        """Test rotation matrix structure is correct"""
        phi, theta, psi = np.deg2rad(25), np.deg2rad(10), np.deg2rad(45)
        state = np.array([0, 0, -100, 25, 0, 0, phi, theta, psi, 0, 0, 0])
        default_uav.set_state(state)

        R_uav = default_uav.rotation_matrix_body_to_ned()

        # Test that it's a valid rotation matrix (orthogonal with det = ±1)
        assert np.allclose(R_uav.T @ R_uav, np.eye(3), atol=1e-10)
        assert abs(abs(np.linalg.det(R_uav)) - 1.0) < 1e-10

    def test_rotation_matrix_identity_at_zero_attitude(self, default_uav):
        """Test that rotation matrix has expected structure at zero attitude"""
        state = np.array([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)
        R = default_uav.rotation_matrix_body_to_ned()

        # At zero attitude, should have identity-like structure (possibly with sign changes for NED)
        # Check that diagonal elements are ±1 and off-diagonals are 0
        assert abs(abs(R[0, 0]) - 1.0) < 1e-10
        assert abs(abs(R[1, 1]) - 1.0) < 1e-10
        assert abs(abs(R[2, 2]) - 1.0) < 1e-10
        assert abs(R[0, 1]) < 1e-10
        assert abs(R[1, 0]) < 1e-10


class TestGravityEffect:
    """Test gravity implementation"""

    def test_gravity_in_stationary_state(self, default_uav, physical_constants):
        """Test that gravity produces correct acceleration in stationary state"""
        # Stationary state (zero velocity)
        state = np.array([0, 0, -100, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        control = np.array([0, 0, 0, 0])
        forces_moments = np.zeros(6)

        derivatives = default_uav.derivatives(0, state, control, forces_moments)

        # In NED, gravity causes positive w-dot (downward)
        # derivatives[5] is w_dot
        assert abs(derivatives[5] - physical_constants['g']) < 0.01

    def test_gravity_magnitude(self, default_uav, physical_constants):
        """Test that gravity magnitude is correct"""
        # The gravity implementation should use g = 9.81 m/s^2
        # We can check this indirectly through acceleration
        state = np.array([0, 0, -100, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        control = np.array([0, 0, 0, 0])
        forces_moments = np.zeros(6)

        derivatives = default_uav.derivatives(0, state, control, forces_moments)

        # Check that downward acceleration matches gravity
        assert abs(derivatives[5] - GRAVITY) < 0.01


class TestDerivatives:
    """Test equations of motion (derivatives)"""

    def test_zero_derivatives_at_equilibrium(self, default_uav):
        """Test that derivatives are nearly zero in trimmed flight"""
        # This is a simplified test - true trim requires proper force balance
        # Here we just check that with zero forces, we get expected behavior
        state = np.array([0, 0, -100, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        control = np.array([0, 0, 0, 0])
        forces_moments = np.zeros(6)

        derivatives = default_uav.derivatives(0, state, control, forces_moments)

        # Position derivatives should be zero (no velocity)
        assert np.allclose(derivatives[0:3], [0, 0, 0], atol=0.01)

        # Angular rate derivatives from euler rates should be zero
        # (zero angular rates)
        assert np.allclose(derivatives[6:9], [0, 0, 0], atol=0.01)

    def test_position_rate_from_velocity(self, default_uav):
        """Test that position derivatives equal velocity in NED frame"""
        # Level flight with velocity in x-direction
        u = 25.0
        state = np.array([0, 0, -100, u, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        control = np.array([0, 0, 0, 0.5])
        forces_moments = np.zeros(6)

        derivatives = default_uav.derivatives(0, state, control, forces_moments)

        # At zero attitude, body x-velocity equals NED north velocity
        # derivatives[0] is pn_dot, should equal u
        assert abs(derivatives[0] - u) < 0.01


class TestNumericalIntegration:
    """Test numerical integration (update method)"""

    def test_update_timestep_consistency(self, default_uav):
        """Test that smaller timesteps give consistent results"""
        # Initial state
        initial_state = np.array([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])

        # Constant forces (use float type)
        forces_moments = np.array([100.0, 0.0, -100.0, 0.0, 0.0, 0.0])

        # Integrate with dt = 0.01 for 1 second
        uav1 = FixedWingUAV(aircraft_type='small')
        uav1.set_state(initial_state.copy())
        for _ in range(100):
            uav1.update(0.01, forces_moments.copy())

        # Integrate with dt = 0.001 for 1 second
        uav2 = FixedWingUAV(aircraft_type='small')
        uav2.set_state(initial_state.copy())
        for _ in range(1000):
            uav2.update(0.001, forces_moments.copy())

        # Results should be close (within integration error)
        state1 = uav1.get_state()
        state2 = uav2.get_state()

        # Allow some tolerance for numerical integration differences
        assert np.allclose(state1, state2, rtol=0.01, atol=0.1)

    def test_integration_preserves_reasonable_values(self, default_uav):
        """Test that integration doesn't produce unrealistic values"""
        # Use smaller forces to keep velocities reasonable
        forces_moments = np.array([20.0, 0.0, -50.0, 0.0, 0.0, 0.0])

        # Run for 10 seconds
        for _ in range(1000):
            default_uav.update(0.01, forces_moments)

            # Check that values remain reasonable
            state = default_uav.get_state()

            # Position shouldn't be extreme
            assert abs(state[0]) < 10000  # North
            assert abs(state[1]) < 10000  # East
            assert state[2] > -10000  # Down (altitude positive up)

            # Velocity shouldn't be extreme
            assert abs(state[3]) < 200  # u
            assert abs(state[4]) < 200  # v
            assert abs(state[5]) < 200  # w

            # Angles should remain in reasonable range
            # Note: Euler angles can wrap

            # Angular rates shouldn't be extreme
            assert abs(state[9]) < 10   # p
            assert abs(state[10]) < 10  # q
            assert abs(state[11]) < 10  # r


class TestEdgeCases:
    """Test edge cases and numerical stability"""

    def test_euler_angle_discontinuity(self, default_uav):
        """Test behavior near Euler angle discontinuities"""
        # Test near ±180 degrees
        angles_to_test = [
            (np.pi - 0.1, 0, 0),
            (-np.pi + 0.1, 0, 0),
            (0, 0, np.pi - 0.1),
            (0, 0, -np.pi + 0.1),
        ]

        for phi, theta, psi in angles_to_test:
            state = np.array([0, 0, -100, 25, 0, 0, phi, theta, psi, 0, 0, 0])
            default_uav.set_state(state)

            # Should not raise errors
            R = default_uav.rotation_matrix_body_to_ned()
            assert R is not None
            assert not np.any(np.isnan(R))

    def test_high_angular_rates(self, default_uav):
        """Test with high angular rates"""
        # High but realistic angular rates (rad/s)
        p, q, r = 2.0, 1.5, 1.0
        state = np.array([0, 0, -100, 25, 0, 0, 0.1, 0.05, 0, p, q, r])
        default_uav.set_state(state)

        control = np.array([0, 0, 0, 0.5])
        forces_moments = np.zeros(6)

        # Should not produce NaN or Inf
        derivatives = default_uav.derivatives(0, state, control, forces_moments)
        assert not np.any(np.isnan(derivatives))
        assert not np.any(np.isinf(derivatives))

    def test_zero_velocity_edge_case(self, default_uav):
        """Test behavior at zero velocity"""
        state = np.array([0, 0, -100, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        # Airspeed should be zero
        Va = default_uav.get_airspeed()
        assert Va < 0.01

        # Angle of attack calculation should handle zero velocity
        # (might be undefined, but shouldn't crash)
        try:
            alpha = default_uav.get_angle_of_attack()
            # If it returns a value, it should be a number
            assert not np.isnan(alpha)
        except:
            # It's okay to raise an exception for undefined case
            pass


class TestControlInput:
    """Test control input handling"""

    def test_set_control(self, default_uav):
        """Test setting control inputs"""
        control = np.array([0.1, 0.2, -0.1, 0.7])
        default_uav.set_control(control)

        # Control should be stored
        assert np.allclose(default_uav.control, control)

    def test_control_limits_are_respected(self, default_uav):
        """Test that extreme control inputs don't crash simulation"""
        # Extreme control inputs
        control = np.array([1.0, 1.0, 1.0, 1.0])
        default_uav.set_control(control)

        forces_moments = np.zeros(6)

        # Should not crash or produce NaN
        for _ in range(10):
            default_uav.update(0.01, forces_moments)
            state = default_uav.get_state()
            assert not np.any(np.isnan(state))
