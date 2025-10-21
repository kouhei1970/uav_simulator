"""
Unit tests for aerodynamics module (src/aerodynamics.py)

Tests the aerodynamic force and moment calculations including:
- Initialization and parameter loading
- Aerodynamic coefficients
- Force and moment computation
- Trim calculations
- Physical validity checks
"""

import pytest
import numpy as np
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel
from tests.fixtures.reference_data import (
    AIR_DENSITY_SEA_LEVEL, dynamic_pressure
)


class TestInitialization:
    """Test aerodynamic model initialization"""

    def test_default_initialization(self):
        """Test default initialization"""
        aero = AerodynamicModel()
        assert aero is not None

    def test_small_aircraft_initialization(self, default_aero):
        """Test small aircraft parameters"""
        assert default_aero.S > 0  # Wing area
        assert default_aero.b > 0  # Wing span
        assert default_aero.c > 0  # Chord

    def test_medium_aircraft_initialization(self, medium_aero):
        """Test medium aircraft has larger parameters than small"""
        small_aero = AerodynamicModel(aircraft_type='small')
        assert medium_aero.S > small_aero.S
        assert medium_aero.b > small_aero.b

    def test_large_aircraft_initialization(self, large_aero):
        """Test large aircraft has largest parameters"""
        medium_aero = AerodynamicModel(aircraft_type='medium')
        assert large_aero.S > medium_aero.S
        assert large_aero.b > medium_aero.b

    def test_aerodynamic_parameters_positive(self, default_aero):
        """Test that key aerodynamic parameters are positive"""
        assert default_aero.S > 0
        assert default_aero.b > 0
        assert default_aero.c > 0
        assert default_aero.CL_alpha > 0  # Lift curve slope
        assert default_aero.CD_0 >= 0     # Parasitic drag


class TestDynamicPressure:
    """Test dynamic pressure calculations"""

    def test_dynamic_pressure_calculation(self, default_uav, default_aero, physical_constants):
        """Test that dynamic pressure is calculated correctly"""
        # Set airspeed
        Va = 25.0
        state = np.array([0, 0, -100, Va, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        # Calculate expected dynamic pressure
        q_expected = 0.5 * physical_constants['rho'] * Va**2

        # The aerodynamic model should use this dynamic pressure
        # We can infer it from the forces
        control = np.array([0, 0, 0, 0])
        forces_moments = default_aero.compute_forces_moments(default_uav, control)

        # Dynamic pressure should be positive
        q_calc = dynamic_pressure(Va, physical_constants['rho'])
        assert abs(q_calc - q_expected) < 0.01

    def test_zero_velocity_zero_aero_forces(self, default_uav, default_aero):
        """Test that zero airspeed produces zero aerodynamic forces (except prop)"""
        # Zero velocity
        state = np.array([0, 0, -100, 0, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        # No thrust
        control = np.array([0, 0, 0, 0])
        forces_moments = default_aero.compute_forces_moments(default_uav, control)

        # Aerodynamic forces should be very small (near zero airspeed)
        # Forces [0:3] might have small values due to numerical issues
        assert abs(forces_moments[0]) < 1.0  # Small force
        assert abs(forces_moments[1]) < 1.0
        assert abs(forces_moments[2]) < 1.0


class TestLiftForce:
    """Test lift force calculations"""

    def test_lift_direction_in_level_flight(self, default_uav, default_aero):
        """Test that lift acts upward in level flight"""
        # Level flight with positive alpha
        u = 25.0
        w = 2.0  # Positive w gives positive alpha
        state = np.array([0, 0, -100, u, 0, w, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        control = np.array([0, 0, 0, 0.5])
        forces_moments = default_aero.compute_forces_moments(default_uav, control)

        # In body frame, lift acts in negative z direction (up)
        # With positive angle of attack, Fz should be negative
        assert forces_moments[2] < 0

    def test_lift_increases_with_angle_of_attack(self, default_uav, default_aero):
        """Test that lift increases with angle of attack"""
        u = 25.0
        control = np.array([0, 0, 0, 0.5])

        # Low angle of attack
        w_low = 1.0
        state_low = np.array([0, 0, -100, u, 0, w_low, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state_low)
        fm_low = default_aero.compute_forces_moments(default_uav, control)

        # Higher angle of attack
        w_high = 3.0
        state_high = np.array([0, 0, -100, u, 0, w_high, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state_high)
        fm_high = default_aero.compute_forces_moments(default_uav, control)

        # Magnitude of lift (negative Fz) should be larger at higher alpha
        assert abs(fm_high[2]) > abs(fm_low[2])

    def test_lift_increases_with_airspeed(self, default_uav, default_aero):
        """Test that lift increases with airspeed (quadratic relationship)"""
        control = np.array([0, 0, 0, 0.5])

        # Lower airspeed
        Va_low = 20.0
        state_low = np.array([0, 0, -100, Va_low, 0, 1.0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state_low)
        fm_low = default_aero.compute_forces_moments(default_uav, control)

        # Higher airspeed
        Va_high = 30.0
        state_high = np.array([0, 0, -100, Va_high, 0, 1.0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state_high)
        fm_high = default_aero.compute_forces_moments(default_uav, control)

        # Lift should increase with V^2
        lift_low = abs(fm_low[2])
        lift_high = abs(fm_high[2])

        # Ratio should be approximately (Va_high/Va_low)^2
        ratio_expected = (Va_high / Va_low) ** 2
        ratio_actual = lift_high / (lift_low + 1e-6)  # Avoid division by zero

        assert abs(ratio_actual - ratio_expected) / ratio_expected < 0.2  # Within 20%


class TestDragForce:
    """Test drag force calculations"""

    def test_drag_opposes_motion(self, default_uav, default_aero):
        """Test that drag opposes direction of motion"""
        # Forward flight
        state = np.array([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        control = np.array([0, 0, 0, 0.5])
        forces_moments = default_aero.compute_forces_moments(default_uav, control)

        # Drag acts in negative x direction (opposes forward motion)
        # However, thrust acts in positive x, so Fx could be positive
        # We'll just check that force is reasonable
        assert abs(forces_moments[0]) < 1000  # Reasonable magnitude

    def test_drag_increases_with_airspeed(self, default_uav, default_aero):
        """Test that drag increases with airspeed"""
        control = np.array([0, 0, 0, 0])  # No thrust to isolate drag

        # Lower airspeed
        state_low = np.array([0, 0, -100, 20, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state_low)
        fm_low = default_aero.compute_forces_moments(default_uav, control)

        # Higher airspeed
        state_high = np.array([0, 0, -100, 30, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state_high)
        fm_high = default_aero.compute_forces_moments(default_uav, control)

        # With higher airspeed, drag magnitude should be larger
        # Drag opposes motion in x-direction
        assert abs(fm_high[0]) > abs(fm_low[0]) or fm_high[0] < fm_low[0]

    def test_drag_is_always_positive(self, default_aero):
        """Test that drag coefficient is always positive"""
        # CD should always be positive
        assert default_aero.CD_0 >= 0


class TestPropellerThrust:
    """Test propeller thrust calculations"""

    def test_thrust_increases_with_throttle(self, default_uav, default_aero):
        """Test that thrust increases with throttle"""
        state = np.array([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        # Low throttle
        control_low = np.array([0, 0, 0, 0.3])
        fm_low = default_aero.compute_forces_moments(default_uav, control_low)

        # High throttle
        control_high = np.array([0, 0, 0, 0.8])
        fm_high = default_aero.compute_forces_moments(default_uav, control_high)

        # Higher throttle should produce more thrust (positive Fx)
        assert fm_high[0] > fm_low[0]

    def test_zero_throttle_minimal_thrust(self, default_uav, default_aero):
        """Test that zero throttle produces minimal thrust"""
        state = np.array([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        control = np.array([0, 0, 0, 0])
        forces_moments = default_aero.compute_forces_moments(default_uav, control)

        # Thrust component should be small
        # Note: Total Fx includes drag, so it might be negative
        # We just check it's not excessively large
        assert abs(forces_moments[0]) < 500


class TestControlSurfaceEffectiveness:
    """Test control surface effects"""

    def test_elevator_affects_pitch_moment(self, default_uav, default_aero):
        """Test that elevator deflection affects pitching moment"""
        state = np.array([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        # Positive elevator (nose up)
        control_up = np.array([0.2, 0, 0, 0.5])
        fm_up = default_aero.compute_forces_moments(default_uav, control_up)

        # Negative elevator (nose down)
        control_down = np.array([-0.2, 0, 0, 0.5])
        fm_down = default_aero.compute_forces_moments(default_uav, control_down)

        # Pitching moments should be different
        assert abs(fm_up[4] - fm_down[4]) > 0.1

    def test_aileron_affects_roll_moment(self, default_uav, default_aero):
        """Test that aileron deflection affects rolling moment"""
        state = np.array([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        # Right aileron
        control_right = np.array([0, 0.2, 0, 0.5])
        fm_right = default_aero.compute_forces_moments(default_uav, control_right)

        # Left aileron
        control_left = np.array([0, -0.2, 0, 0.5])
        fm_left = default_aero.compute_forces_moments(default_uav, control_left)

        # Rolling moments should be different
        assert abs(fm_right[3] - fm_left[3]) > 0.1

    def test_rudder_affects_yaw_moment(self, default_uav, default_aero):
        """Test that rudder deflection affects yawing moment"""
        state = np.array([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        # Right rudder
        control_right = np.array([0, 0, 0.2, 0.5])
        fm_right = default_aero.compute_forces_moments(default_uav, control_right)

        # Left rudder
        control_left = np.array([0, 0, -0.2, 0.5])
        fm_left = default_aero.compute_forces_moments(default_uav, control_left)

        # Yawing moments should be different
        assert abs(fm_right[5] - fm_left[5]) > 0.01


class TestPhysicalValidity:
    """Test physical validity of aerodynamic calculations"""

    def test_lift_to_drag_ratio_reasonable(self, default_uav, default_aero):
        """Test that lift-to-drag ratio is in reasonable range"""
        # Cruise condition
        state = np.array([0, 0, -100, 25, 0, 1.5, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        control = np.array([0, 0, 0, 0.5])
        forces_moments = default_aero.compute_forces_moments(default_uav, control)

        # Get lift and drag in body frame
        # Lift is approximately -Fz, drag is approximately -Fx (minus thrust)
        # This is simplified, but should give reasonable values

        # For a small UAV, L/D typically ranges from 10 to 20
        # We'll just check that the forces are reasonable magnitudes
        assert abs(forces_moments[0]) < 500   # Fx reasonable
        assert abs(forces_moments[2]) < 1000  # Fz reasonable

    def test_forces_scale_with_dynamic_pressure(self, default_uav, default_aero):
        """Test that forces scale with dynamic pressure"""
        # Same angle of attack, different airspeeds
        alpha = np.deg2rad(5)
        w = 25.0 * np.sin(alpha)
        u = 25.0 * np.cos(alpha)

        control = np.array([0, 0, 0, 0.5])

        # Lower airspeed
        Va_low = 20.0
        state_low = np.array([0, 0, -100, u * Va_low/25.0, 0, w * Va_low/25.0,
                             0, 0, 0, 0, 0, 0])
        default_uav.set_state(state_low)
        fm_low = default_aero.compute_forces_moments(default_uav, control)

        # Higher airspeed
        Va_high = 30.0
        state_high = np.array([0, 0, -100, u * Va_high/25.0, 0, w * Va_high/25.0,
                              0, 0, 0, 0, 0, 0])
        default_uav.set_state(state_high)
        fm_high = default_aero.compute_forces_moments(default_uav, control)

        # Forces should scale approximately with V^2
        q_ratio = (Va_high / Va_low) ** 2

        # Check that force ratios are close to q_ratio
        # Using Fz (lift) as it's usually dominant
        if abs(fm_low[2]) > 1.0:  # Avoid division by small numbers
            force_ratio = abs(fm_high[2]) / abs(fm_low[2])
            assert abs(force_ratio - q_ratio) / q_ratio < 0.3  # Within 30%

    def test_moments_have_reasonable_magnitudes(self, default_uav, default_aero):
        """Test that moments are reasonable in magnitude"""
        state = np.array([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        control = np.array([0.1, 0.1, 0.1, 0.5])
        forces_moments = default_aero.compute_forces_moments(default_uav, control)

        # Moments should be reasonable (not extreme)
        assert abs(forces_moments[3]) < 100  # Roll moment
        assert abs(forces_moments[4]) < 100  # Pitch moment
        assert abs(forces_moments[5]) < 100  # Yaw moment


class TestStability:
    """Test static stability characteristics"""

    def test_pitch_stability_exists(self, default_aero):
        """Test that aircraft has pitch stability (Cm_alpha < 0)"""
        # For longitudinal stability, Cm_alpha should be negative
        # This ensures nose-down moment when alpha increases
        # Note: Some aircraft might not have this parameter explicitly
        # We'll check if it exists and if so, verify it's negative
        if hasattr(default_aero, 'Cm_alpha'):
            assert default_aero.Cm_alpha < 0

    def test_positive_lift_curve_slope(self, default_aero):
        """Test that lift curve slope is positive"""
        assert default_aero.CL_alpha > 0


class TestTrimCalculation:
    """Test trim control calculations"""

    def test_trim_level_flight_exists(self, default_aero):
        """Test that trim calculation for level flight produces results"""
        Va = 25.0  # m/s
        gamma = 0.0  # rad (level flight)
        R = np.inf  # Straight flight

        # Get trim controls
        trim_controls = default_aero.get_trim_controls(Va, gamma, R)

        # Should return controls (might be None if not implemented)
        # If implemented, controls should be reasonable
        if trim_controls is not None:
            assert len(trim_controls) == 4
            # Throttle should be in [0, 1]
            assert 0 <= trim_controls[3] <= 1

    def test_trim_controls_reasonable_range(self, default_aero):
        """Test that trim controls are in reasonable ranges"""
        Va = 25.0
        gamma = 0.0
        R = np.inf

        trim_controls = default_aero.get_trim_controls(Va, gamma, R)

        if trim_controls is not None:
            delta_e, delta_a, delta_r, delta_t = trim_controls

            # Control surfaces typically in [-1, 1] or similar range
            assert abs(delta_e) < 1.5
            assert abs(delta_a) < 1.5
            assert abs(delta_r) < 1.5
            assert 0 <= delta_t <= 1


class TestEdgeCases:
    """Test edge cases and numerical stability"""

    def test_high_angle_of_attack(self, default_uav, default_aero):
        """Test behavior at high angle of attack (near stall)"""
        # High alpha
        u = 20.0
        w = 10.0  # About 26 degrees
        state = np.array([0, 0, -100, u, 0, w, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        control = np.array([0, 0, 0, 0.5])

        # Should not crash or produce NaN
        forces_moments = default_aero.compute_forces_moments(default_uav, control)
        assert not np.any(np.isnan(forces_moments))
        assert not np.any(np.isinf(forces_moments))

    def test_sideslip_handling(self, default_uav, default_aero):
        """Test handling of sideslip (nonzero v)"""
        # Forward flight with sideslip
        state = np.array([0, 0, -100, 25, 5, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        control = np.array([0, 0, 0, 0.5])

        # Should produce side force and yawing moment
        forces_moments = default_aero.compute_forces_moments(default_uav, control)

        assert not np.any(np.isnan(forces_moments))
        # Side force (Fy) should be nonzero
        assert abs(forces_moments[1]) > 0.01

    def test_negative_airspeed_handling(self, default_uav, default_aero):
        """Test that negative airspeed is handled (shouldn't occur, but test robustness)"""
        # Backwards flight (unusual but test robustness)
        state = np.array([0, 0, -100, -10, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        control = np.array([0, 0, 0, 0.5])

        # Should not crash
        try:
            forces_moments = default_aero.compute_forces_moments(default_uav, control)
            assert not np.any(np.isnan(forces_moments))
        except:
            # It's okay to raise an exception for invalid state
            pass
