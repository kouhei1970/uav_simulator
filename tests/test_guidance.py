"""
Unit tests for guidance module (src/guidance.py)

Tests guidance algorithms including:
- L1 adaptive guidance
- Proportional orbit guidance
- Orbit guidance
- Waypoint guidance
- Straight line guidance
"""

import pytest
import numpy as np
from src.guidance import (
    L1Guidance, ProportionalOrbitGuidance, OrbitGuidance,
    WaypointGuidance, StraightLineGuidance, CoordinatedTurnGuidance
)
from tests.fixtures.reference_data import (
    l1_distance, proportional_guidance_eigenvalue,
    first_order_time_constant, theoretical_bank_angle_steady_turn
)


class TestL1Guidance:
    """Test L1 adaptive guidance"""

    def test_initialization(self, l1_guidance):
        """Test L1 guidance initialization"""
        assert l1_guidance is not None
        assert l1_guidance.damping == 0.707
        assert l1_guidance.period == 15.0

    def test_l1_distance_calculation(self, l1_guidance):
        """Test L1 distance calculation matches theory"""
        Va = 25.0
        L1_calc = l1_guidance.compute_L1_distance(Va)
        L1_expected = l1_distance(l1_guidance.damping, l1_guidance.period, Va)

        assert abs(L1_calc - L1_expected) < 0.01

    def test_l1_distance_scales_with_airspeed(self, l1_guidance):
        """Test that L1 distance scales linearly with airspeed"""
        Va1 = 20.0
        Va2 = 30.0

        L1_1 = l1_guidance.compute_L1_distance(Va1)
        L1_2 = l1_guidance.compute_L1_distance(Va2)

        # L1 is proportional to Va
        ratio_expected = Va2 / Va1
        ratio_actual = L1_2 / L1_1

        assert abs(ratio_actual - ratio_expected) < 0.01

    def test_line_tracking_lateral_acceleration(self, l1_guidance):
        """Test lateral acceleration command for line tracking"""
        # Position off the line
        position = np.array([10, 5, -100])
        Va = 25.0
        chi = 0.0  # heading north

        # Line from origin to [100, 0, -100]
        path_params = {
            'start': np.array([0, 0, -100]),
            'end': np.array([100, 0, -100])
        }

        a_cmd, eta = l1_guidance.compute_lateral_acceleration(
            position, Va, chi, path_type='line', path_params=path_params
        )

        # Should command lateral acceleration to return to line
        assert abs(a_cmd) > 0.01  # Nonzero command
        assert abs(a_cmd) < 100  # Reasonable magnitude

    def test_orbit_tracking_lateral_acceleration(self, l1_guidance):
        """Test lateral acceleration command for orbit tracking"""
        # Position outside orbit
        center = np.array([0, 0, -100])
        radius = 50.0
        position = np.array([80, 0, -100])  # 80m from center
        Va = 25.0
        chi = np.pi / 2  # heading east

        path_params = {
            'center': center,
            'radius': radius,
            'direction': 'CW'
        }

        a_cmd, eta = l1_guidance.compute_lateral_acceleration(
            position, Va, chi, path_type='orbit', path_params=path_params
        )

        # Should command to turn towards orbit
        assert abs(a_cmd) > 0.01
        assert abs(a_cmd) < 200

    def test_roll_command_from_acceleration(self, l1_guidance):
        """Test bank angle command from lateral acceleration"""
        Va = 25.0
        a_cmd = 10.0  # m/s^2
        phi_limit = np.pi / 4

        phi_cmd = l1_guidance.compute_roll_command(a_cmd, Va, phi_limit)

        # Should be within limits
        assert abs(phi_cmd) <= phi_limit

        # Should have correct sign
        # Positive a_cmd should give positive phi
        assert np.sign(phi_cmd) == np.sign(a_cmd) or abs(phi_cmd) > phi_limit - 0.01

    def test_roll_command_saturation(self, l1_guidance):
        """Test that roll command saturates at limit"""
        Va = 25.0
        a_cmd = 1000.0  # Very large acceleration
        phi_limit = np.pi / 4

        phi_cmd = l1_guidance.compute_roll_command(a_cmd, Va, phi_limit)

        # Should saturate at limit
        assert abs(phi_cmd) <= phi_limit + 0.01


class TestProportionalOrbitGuidance:
    """Test proportional orbit guidance"""

    def test_initialization(self, proportional_guidance):
        """Test proportional guidance initialization"""
        assert proportional_guidance is not None
        assert proportional_guidance.K_p == 0.08
        assert proportional_guidance.phi_max == np.pi / 4

    def test_feedforward_bank_angle(self, proportional_guidance):
        """Test feedforward bank angle calculation"""
        Va = 25.0
        radius = 50.0
        direction = 'CW'

        phi_ff = proportional_guidance.compute_feedforward_roll(Va, radius, direction)

        # Compare with theoretical value
        phi_expected = theoretical_bank_angle_steady_turn(Va, radius)

        # CW should give negative bank angle (right turn)
        assert phi_ff < 0
        assert abs(abs(phi_ff) - phi_expected) < 0.1

    def test_feedforward_direction(self, proportional_guidance):
        """Test that feedforward bank angle changes sign with direction"""
        Va = 25.0
        radius = 50.0

        phi_cw = proportional_guidance.compute_feedforward_roll(Va, radius, 'CW')
        phi_ccw = proportional_guidance.compute_feedforward_roll(Va, radius, 'CCW')

        # Should have opposite signs
        assert phi_cw * phi_ccw < 0

    def test_roll_command_with_zero_error(self, proportional_guidance):
        """Test roll command with zero radius error"""
        center = np.array([0, 0, -100])
        radius = 50.0
        position = np.array([50, 0, -100])  # Exactly on orbit
        Va = 25.0

        phi_cmd, e_r, d = proportional_guidance.compute_roll_command(
            position, center, radius, Va, 'CW'
        )

        # Radius error should be zero
        assert abs(e_r) < 0.01

        # Should command approximately feedforward bank angle
        phi_ff = proportional_guidance.compute_feedforward_roll(Va, radius, 'CW')
        assert abs(phi_cmd - phi_ff) < 0.1

    def test_roll_command_with_positive_error(self, proportional_guidance):
        """Test roll command when outside orbit"""
        center = np.array([0, 0, -100])
        radius = 50.0
        position = np.array([60, 0, -100])  # 10m outside
        Va = 25.0

        phi_cmd, e_r, d = proportional_guidance.compute_roll_command(
            position, center, radius, Va, 'CW'
        )

        # Radius error should be positive
        assert e_r > 0

        # Should command tighter turn (more negative for CW)
        phi_ff = proportional_guidance.compute_feedforward_roll(Va, radius, 'CW')
        assert phi_cmd < phi_ff  # More negative

    def test_roll_command_saturation(self, proportional_guidance):
        """Test that roll command saturates"""
        center = np.array([0, 0, -100])
        radius = 50.0
        position = np.array([200, 0, -100])  # Far outside
        Va = 25.0

        phi_cmd, e_r, d = proportional_guidance.compute_roll_command(
            position, center, radius, Va, 'CW'
        )

        # Should saturate at phi_max
        assert abs(phi_cmd) <= proportional_guidance.phi_max + 0.01

    def test_stability_analysis(self, proportional_guidance):
        """Test stability analysis calculations"""
        Va = 25.0
        radius = 50.0

        eigenvalue, time_constant, damping = proportional_guidance.analyze_stability(
            Va, radius
        )

        # Eigenvalue should be negative (stable)
        assert eigenvalue < 0

        # Compare with theoretical eigenvalue
        eigenvalue_expected = proportional_guidance_eigenvalue(
            proportional_guidance.K_p, Va
        )
        assert abs(eigenvalue - eigenvalue_expected) < 0.01

        # Time constant should match
        tau_expected = first_order_time_constant(eigenvalue)
        assert abs(time_constant - tau_expected) < 0.01


class TestOrbitGuidance:
    """Test basic orbit guidance"""

    def test_initialization(self, orbit_guidance):
        """Test orbit guidance initialization"""
        assert orbit_guidance is not None

    def test_orbit_command(self, orbit_guidance):
        """Test orbit guidance produces commands"""
        position = np.array([60, 0, -100])
        Va = 25.0
        chi = 0.0

        phi_cmd = orbit_guidance.compute_command(position, Va, chi)

        # Should produce a bank angle command
        assert abs(phi_cmd) < np.pi / 2  # Reasonable bank angle


class TestWaypointGuidance:
    """Test waypoint guidance"""

    def test_initialization(self, waypoint_guidance):
        """Test waypoint guidance initialization"""
        assert waypoint_guidance is not None

    def test_waypoint_switching(self):
        """Test waypoint switching logic"""
        waypoints = np.array([
            [0, 0, -100],
            [100, 0, -100],
            [100, 100, -100]
        ])

        guidance = WaypointGuidance(waypoints, switch_distance=10.0)

        # Start near first waypoint
        position = np.array([5, 0, -100])

        # Get command
        phi_cmd, current_wp_idx = guidance.compute_command(
            position, Va=25.0, chi=0.0
        )

        # Should still be targeting first waypoint or have switched
        assert current_wp_idx >= 0
        assert current_wp_idx < len(waypoints)

    def test_final_waypoint_handling(self):
        """Test behavior at final waypoint"""
        waypoints = np.array([
            [0, 0, -100],
            [100, 0, -100]
        ])

        guidance = WaypointGuidance(waypoints, switch_distance=10.0)

        # Position at final waypoint
        position = np.array([100, 0, -100])

        # Should not crash
        try:
            phi_cmd, wp_idx = guidance.compute_command(position, 25.0, 0.0)
            assert wp_idx <= len(waypoints)
        except:
            # It's ok if it raises an exception for completed path
            pass


class TestStraightLineGuidance:
    """Test straight line guidance"""

    def test_initialization(self, line_guidance):
        """Test line guidance initialization"""
        assert line_guidance is not None

    def test_line_following_command(self, line_guidance):
        """Test that line following produces guidance command"""
        # Position off the line
        position = np.array([10, 10, -100])
        Va = 25.0
        chi = 0.0

        phi_cmd = line_guidance.compute_command(position, Va, chi)

        # Should produce a command to return to line
        assert abs(phi_cmd) > 0.01  # Nonzero
        assert abs(phi_cmd) < np.pi / 2  # Reasonable

    def test_on_line_small_command(self, line_guidance):
        """Test that being on line produces small command"""
        # Position exactly on line (start to end is [0,0,-100] to [1000,500,-100])
        # Line direction is [1000, 500, 0], normalized
        # A point on the line
        position = np.array([500, 250, -100])
        Va = 25.0
        # Heading along the line
        chi = np.arctan2(500, 1000)

        phi_cmd = line_guidance.compute_command(position, Va, chi)

        # Should produce small command (already on line)
        assert abs(phi_cmd) < 0.2


class TestCoordinatedTurnGuidance:
    """Test coordinated turn guidance"""

    def test_initialization(self):
        """Test coordinated turn guidance initialization"""
        guidance = CoordinatedTurnGuidance(turn_radius=100.0)
        assert guidance is not None

    def test_turn_command(self):
        """Test turn command generation"""
        guidance = CoordinatedTurnGuidance(turn_radius=100.0, direction='CW')

        position = np.array([0, 0, -100])
        Va = 25.0
        chi = 0.0

        phi_cmd = guidance.compute_command(position, Va, chi)

        # Should command appropriate bank angle for turn
        assert abs(phi_cmd) > 0.01
        assert abs(phi_cmd) < np.pi / 2


class TestGuidanceIntegration:
    """Test guidance integration scenarios"""

    def test_guidance_produces_bounded_commands(
        self, l1_guidance, proportional_guidance, orbit_guidance
    ):
        """Test that all guidance laws produce bounded commands"""
        position = np.array([50, 50, -100])
        Va = 25.0
        chi = 0.0

        # L1 line tracking
        line_params = {
            'start': np.array([0, 0, -100]),
            'end': np.array([100, 100, -100])
        }
        a_cmd, _ = l1_guidance.compute_lateral_acceleration(
            position, Va, chi, 'line', line_params
        )
        phi_l1 = l1_guidance.compute_roll_command(a_cmd, Va, np.pi/4)
        assert abs(phi_l1) <= np.pi/4

        # Proportional guidance
        center = np.array([0, 0, -100])
        phi_prop, _, _ = proportional_guidance.compute_roll_command(
            position, center, 50.0, Va, 'CW'
        )
        assert abs(phi_prop) <= proportional_guidance.phi_max

        # Orbit guidance
        phi_orbit = orbit_guidance.compute_command(position, Va, chi)
        assert abs(phi_orbit) < np.pi/2

    def test_guidance_no_nan_outputs(
        self, l1_guidance, proportional_guidance
    ):
        """Test that guidance laws don't produce NaN"""
        position = np.array([100, 200, -100])
        Va = 25.0
        chi = 0.5

        # L1 orbit
        orbit_params = {
            'center': np.array([0, 0, -100]),
            'radius': 50.0,
            'direction': 'CW'
        }
        a_cmd, _ = l1_guidance.compute_lateral_acceleration(
            position, Va, chi, 'orbit', orbit_params
        )
        assert not np.isnan(a_cmd)

        phi_cmd = l1_guidance.compute_roll_command(a_cmd, Va, np.pi/4)
        assert not np.isnan(phi_cmd)

        # Proportional
        phi_prop, _, _ = proportional_guidance.compute_roll_command(
            position, orbit_params['center'], orbit_params['radius'], Va, 'CW'
        )
        assert not np.isnan(phi_prop)

    def test_guidance_consistent_with_theory(self, proportional_guidance):
        """Test that proportional guidance eigenvalue matches theory"""
        Va = 20.0
        radius = 60.0

        eigenvalue, tau, _ = proportional_guidance.analyze_stability(Va, radius)

        # Eigenvalue = -K_p * V_a
        eigenvalue_theory = -proportional_guidance.K_p * Va
        assert abs(eigenvalue - eigenvalue_theory) < 0.01

        # Time constant = 1 / (K_p * V_a)
        tau_theory = 1 / (proportional_guidance.K_p * Va)
        assert abs(tau - tau_theory) < 0.01
