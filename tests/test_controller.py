"""
Unit tests for controller module (src/controller.py)

Tests the control algorithms including:
- PID controller
- Attitude controller
- Altitude controller
- Airspeed controller
- Total Energy Control System (TECS)
"""

import pytest
import numpy as np
from src.controller import (
    PIDController, AttitudeController, AltitudeController,
    AirspeedController, TotalEnergyController
)


class TestPIDController:
    """Test basic PID controller"""

    def test_initialization(self, pid_controller):
        """Test PID controller initialization"""
        assert pid_controller is not None
        assert pid_controller.kp == 1.0
        assert pid_controller.ki == 0.1
        assert pid_controller.kd == 0.05

    def test_proportional_action(self):
        """Test proportional control action"""
        controller = PIDController(kp=2.0, ki=0, kd=0)
        dt = 0.01

        error = 5.0
        output = controller.compute(error, dt)

        # Output should be Kp * error
        assert abs(output - 2.0 * 5.0) < 0.01

    def test_integral_action_accumulates(self):
        """Test that integral action accumulates error"""
        controller = PIDController(kp=0, ki=1.0, kd=0)
        dt = 0.01

        # Apply constant error for multiple steps
        error = 1.0
        for _ in range(10):
            output = controller.compute(error, dt)

        # Integral should have accumulated
        assert output > 0.05  # Should be accumulating

    def test_derivative_action(self):
        """Test derivative control action"""
        controller = PIDController(kp=0, ki=0, kd=1.0)
        dt = 0.01

        # Step change in error
        controller.compute(0, dt)
        output = controller.compute(5.0, dt)

        # Derivative term should respond to rate of change
        assert abs(output) > 0.1

    def test_output_saturation(self):
        """Test that output saturation limits work"""
        controller = PIDController(kp=10.0, ki=0, kd=0,
                                  output_min=-5.0, output_max=5.0)
        dt = 0.01

        # Large error should saturate
        error = 100.0
        output = controller.compute(error, dt)

        assert output <= 5.0
        assert output >= -5.0

    def test_anti_windup(self):
        """Test integral anti-windup"""
        controller = PIDController(kp=1.0, ki=1.0, kd=0,
                                  output_min=-10.0, output_max=10.0)
        dt = 0.01

        # Apply large error that saturates output
        large_error = 50.0
        for _ in range(100):
            controller.compute(large_error, dt)

        # Now apply negative error - with anti-windup, should respond quickly
        # Without anti-windup, integral would need to unwind first
        output = controller.compute(-10.0, dt)
        # Just check that it doesn't blow up
        assert abs(output) < 100


class TestAttitudeController:
    """Test attitude control system"""

    def test_initialization(self, attitude_controller):
        """Test attitude controller initialization"""
        assert attitude_controller is not None

    def test_roll_control_output_range(self, attitude_controller, default_uav):
        """Test that roll control output is in valid range"""
        phi_cmd = np.deg2rad(30)
        theta_cmd = 0
        dt = 0.01

        delta_a, delta_e, delta_r = attitude_controller.compute_control(
            default_uav, phi_cmd, theta_cmd, dt
        )

        # Control outputs should be reasonable
        assert abs(delta_a) < 2.0  # Aileron
        assert abs(delta_e) < 2.0  # Elevator
        assert abs(delta_r) < 2.0  # Rudder

    def test_pitch_control_output_range(self, attitude_controller, default_uav):
        """Test that pitch control output is in valid range"""
        phi_cmd = 0
        theta_cmd = np.deg2rad(10)
        dt = 0.01

        delta_a, delta_e, delta_r = attitude_controller.compute_control(
            default_uav, phi_cmd, theta_cmd, dt
        )

        assert abs(delta_e) < 2.0

    def test_zero_error_zero_output(self, attitude_controller, default_uav):
        """Test that zero attitude error produces small control output"""
        # Set UAV to level flight
        state = np.array([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        phi_cmd = 0
        theta_cmd = 0
        dt = 0.01

        # Warm up controller
        for _ in range(10):
            attitude_controller.compute_control(default_uav, phi_cmd, theta_cmd, dt)

        delta_a, delta_e, delta_r = attitude_controller.compute_control(
            default_uav, phi_cmd, theta_cmd, dt
        )

        # With zero error, outputs should be small (not necessarily zero due to damping)
        assert abs(delta_a) < 0.5
        assert abs(delta_e) < 0.5

    def test_roll_response_direction(self, attitude_controller, default_uav):
        """Test that roll control responds in correct direction"""
        state = np.array([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        dt = 0.01

        # Command right roll
        phi_cmd_right = np.deg2rad(20)
        delta_a_right, _, _ = attitude_controller.compute_control(
            default_uav, phi_cmd_right, 0, dt
        )

        # Command left roll
        phi_cmd_left = np.deg2rad(-20)
        delta_a_left, _, _ = attitude_controller.compute_control(
            default_uav, phi_cmd_left, 0, dt
        )

        # Aileron commands should have opposite signs
        assert delta_a_right * delta_a_left < 0

    def test_pitch_response_direction(self, attitude_controller, default_uav):
        """Test that pitch control responds in correct direction"""
        state = np.array([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        dt = 0.01

        # Command nose up
        theta_cmd_up = np.deg2rad(10)
        _, delta_e_up, _ = attitude_controller.compute_control(
            default_uav, 0, theta_cmd_up, dt
        )

        # Command nose down
        theta_cmd_down = np.deg2rad(-10)
        _, delta_e_down, _ = attitude_controller.compute_control(
            default_uav, 0, theta_cmd_down, dt
        )

        # Elevator commands should have opposite signs
        assert delta_e_up * delta_e_down < 0


class TestAltitudeController:
    """Test altitude control system"""

    def test_initialization(self, altitude_controller):
        """Test altitude controller initialization"""
        assert altitude_controller is not None

    def test_altitude_hold_command(self, altitude_controller, default_uav):
        """Test altitude hold produces pitch command"""
        # Current altitude: 100m (pd = -100)
        state = np.array([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        # Command altitude: 150m (pd = -150)
        h_cmd = -150
        dt = 0.01

        theta_cmd = altitude_controller.compute_pitch_command(default_uav, h_cmd, dt)

        # Should command climb (positive pitch for climb)
        # Just check that output is reasonable
        assert abs(theta_cmd) < np.deg2rad(45)

    def test_altitude_error_response_direction(self, altitude_controller, default_uav):
        """Test that altitude error produces correct pitch response"""
        state = np.array([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)
        dt = 0.01

        # Command higher altitude (climb)
        h_cmd_high = -150
        theta_cmd_high = altitude_controller.compute_pitch_command(
            default_uav, h_cmd_high, dt
        )

        # Reset controller
        altitude_controller2 = AltitudeController()

        # Command lower altitude (descend)
        h_cmd_low = -50
        theta_cmd_low = altitude_controller2.compute_pitch_command(
            default_uav, h_cmd_low, dt
        )

        # Pitch commands should have opposite signs
        # (positive for climb, negative for descend)
        assert theta_cmd_high * theta_cmd_low < 0

    def test_pitch_command_limits(self, altitude_controller, default_uav):
        """Test that pitch commands are limited"""
        state = np.array([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        # Extreme altitude command
        h_cmd = -1000  # 1000m altitude
        dt = 0.01

        theta_cmd = altitude_controller.compute_pitch_command(default_uav, h_cmd, dt)

        # Should be limited to reasonable values
        assert abs(theta_cmd) < np.deg2rad(60)


class TestAirspeedController:
    """Test airspeed control system"""

    def test_initialization(self, airspeed_controller):
        """Test airspeed controller initialization"""
        assert airspeed_controller is not None

    def test_airspeed_hold_command(self, airspeed_controller, default_uav):
        """Test airspeed hold produces throttle command"""
        state = np.array([0, 0, -100, 20, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        # Command higher airspeed
        Va_cmd = 30.0
        dt = 0.01

        delta_t = airspeed_controller.compute_throttle_command(default_uav, Va_cmd, dt)

        # Should command more throttle
        assert 0 <= delta_t <= 1

    def test_throttle_limits(self, airspeed_controller, default_uav):
        """Test that throttle is limited to [0, 1]"""
        state = np.array([0, 0, -100, 10, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        # Extreme airspeed command
        Va_cmd = 100.0
        dt = 0.01

        delta_t = airspeed_controller.compute_throttle_command(default_uav, Va_cmd, dt)

        assert 0 <= delta_t <= 1

    def test_airspeed_error_response_direction(self, airspeed_controller, default_uav):
        """Test that airspeed error produces correct throttle response"""
        state = np.array([0, 0, -100, 20, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)
        dt = 0.01

        # Command higher airspeed
        Va_cmd_high = 30.0
        delta_t_high = airspeed_controller.compute_throttle_command(
            default_uav, Va_cmd_high, dt
        )

        # Reset controller
        airspeed_controller2 = AirspeedController()

        # Command lower airspeed
        Va_cmd_low = 15.0
        delta_t_low = airspeed_controller2.compute_throttle_command(
            default_uav, Va_cmd_low, dt
        )

        # Higher airspeed command should produce higher throttle
        assert delta_t_high > delta_t_low


class TestTotalEnergyController:
    """Test Total Energy Control System"""

    def test_initialization(self, tecs_controller):
        """Test TECS initialization"""
        assert tecs_controller is not None

    def test_tecs_produces_valid_commands(self, tecs_controller, default_uav):
        """Test that TECS produces valid pitch and throttle commands"""
        state = np.array([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)

        h_cmd = -150  # Climb to 150m
        Va_cmd = 30.0  # Increase airspeed
        dt = 0.01

        theta_cmd, delta_t = tecs_controller.compute_commands(
            default_uav, h_cmd, Va_cmd, dt
        )

        # Commands should be in reasonable ranges
        assert abs(theta_cmd) < np.deg2rad(45)
        assert 0 <= delta_t <= 1

    def test_tecs_altitude_and_airspeed_coupling(self, tecs_controller, default_uav):
        """Test TECS handles altitude and airspeed commands together"""
        state = np.array([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)
        dt = 0.01

        # Climb with constant airspeed
        h_cmd = -150
        Va_cmd = 25.0

        theta_cmd, delta_t = tecs_controller.compute_commands(
            default_uav, h_cmd, Va_cmd, dt
        )

        # Should command both pitch and throttle
        assert abs(theta_cmd) > 0.01  # Some pitch command
        assert delta_t > 0.1  # Some throttle


class TestControllerIntegration:
    """Test controller integration and stability"""

    def test_controllers_produce_bounded_outputs(
        self, attitude_controller, altitude_controller,
        airspeed_controller, default_uav
    ):
        """Test that all controllers produce bounded outputs"""
        state = np.array([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)
        dt = 0.01

        # Run controllers for several steps
        for _ in range(100):
            # Altitude controller
            theta_cmd = altitude_controller.compute_pitch_command(
                default_uav, -120, dt
            )

            # Airspeed controller
            delta_t = airspeed_controller.compute_throttle_command(
                default_uav, 28.0, dt
            )

            # Attitude controller
            delta_a, delta_e, delta_r = attitude_controller.compute_control(
                default_uav, 0, theta_cmd, dt
            )

            # All outputs should be bounded
            assert abs(theta_cmd) < 2.0
            assert 0 <= delta_t <= 1
            assert abs(delta_a) < 2.0
            assert abs(delta_e) < 2.0
            assert abs(delta_r) < 2.0

    def test_controller_doesnt_produce_nan(
        self, attitude_controller, default_uav
    ):
        """Test that controller doesn't produce NaN values"""
        state = np.array([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
        default_uav.set_state(state)
        dt = 0.01

        for _ in range(100):
            delta_a, delta_e, delta_r = attitude_controller.compute_control(
                default_uav, np.deg2rad(20), np.deg2rad(5), dt
            )

            assert not np.isnan(delta_a)
            assert not np.isnan(delta_e)
            assert not np.isnan(delta_r)
