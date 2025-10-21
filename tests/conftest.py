"""
pytest configuration and fixtures for UAV simulator tests

This module provides common fixtures for testing UAV simulator components.
"""

import pytest
import numpy as np
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel
from src.controller import (
    PIDController, AttitudeController, AltitudeController,
    AirspeedController, TotalEnergyController
)
from src.sensors import GPSSensor, SimpleKalmanFilter, OrbitCenterEstimator
from src.guidance import (
    L1Guidance, ProportionalOrbitGuidance, OrbitGuidance,
    WaypointGuidance, StraightLineGuidance
)


# ============================================================================
# UAV and Aerodynamics Fixtures
# ============================================================================

@pytest.fixture
def default_uav():
    """Default small UAV instance in steady level flight"""
    uav = FixedWingUAV(aircraft_type='small')
    # State: [pn, pe, pd, u, v, w, phi, theta, psi, p, q, r]
    # Initialize at 100m altitude, 25 m/s airspeed, level flight
    uav.set_state([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])
    return uav


@pytest.fixture
def medium_uav():
    """Medium UAV instance"""
    uav = FixedWingUAV(aircraft_type='medium')
    uav.set_state([0, 0, -100, 30, 0, 0, 0, 0, 0, 0, 0, 0])
    return uav


@pytest.fixture
def large_uav():
    """Large UAV instance"""
    uav = FixedWingUAV(aircraft_type='large')
    uav.set_state([0, 0, -100, 35, 0, 0, 0, 0, 0, 0, 0, 0])
    return uav


@pytest.fixture
def default_aero():
    """Default aerodynamic model for small aircraft"""
    return AerodynamicModel(aircraft_type='small')


@pytest.fixture
def medium_aero():
    """Aerodynamic model for medium aircraft"""
    return AerodynamicModel(aircraft_type='medium')


@pytest.fixture
def large_aero():
    """Aerodynamic model for large aircraft"""
    return AerodynamicModel(aircraft_type='large')


# ============================================================================
# Controller Fixtures
# ============================================================================

@pytest.fixture
def pid_controller():
    """Basic PID controller"""
    return PIDController(kp=1.0, ki=0.1, kd=0.05)


@pytest.fixture
def attitude_controller():
    """Attitude controller"""
    return AttitudeController()


@pytest.fixture
def altitude_controller():
    """Altitude controller"""
    return AltitudeController()


@pytest.fixture
def airspeed_controller():
    """Airspeed controller"""
    return AirspeedController()


@pytest.fixture
def tecs_controller():
    """Total Energy Control System controller"""
    return TotalEnergyController()


# ============================================================================
# Sensor Fixtures
# ============================================================================

@pytest.fixture
def gps_sensor():
    """GPS sensor with realistic noise parameters"""
    return GPSSensor(
        noise_std_horizontal=2.5,
        noise_std_vertical=3.0,
        drift_magnitude=1.0,
        outlier_probability=0.002,
        outlier_magnitude=15.0
    )


@pytest.fixture
def kalman_filter():
    """Simple Kalman filter"""
    return SimpleKalmanFilter(
        process_variance=0.01,
        measurement_variance=6.25
    )


@pytest.fixture
def orbit_estimator():
    """Orbit center and radius estimator"""
    return OrbitCenterEstimator(
        process_variance=0.01,
        measurement_variance=6.25
    )


# ============================================================================
# Guidance Fixtures
# ============================================================================

@pytest.fixture
def l1_guidance():
    """L1 adaptive guidance"""
    return L1Guidance(L1_damping=0.707, L1_period=15.0)


@pytest.fixture
def proportional_guidance():
    """Proportional orbit guidance"""
    return ProportionalOrbitGuidance(K_p=0.08, phi_max=np.pi/4)


@pytest.fixture
def orbit_guidance():
    """Basic orbit guidance"""
    center = np.array([0, 0, -100])
    radius = 50.0
    return OrbitGuidance(center, radius)


@pytest.fixture
def waypoint_guidance():
    """Waypoint guidance"""
    waypoints = np.array([
        [0, 0, -100],
        [100, 0, -100],
        [100, 100, -100],
        [0, 100, -100]
    ])
    return WaypointGuidance(waypoints)


@pytest.fixture
def line_guidance():
    """Straight line guidance"""
    start = np.array([0, 0, -100])
    end = np.array([1000, 500, -100])
    return StraightLineGuidance(start, end)


# ============================================================================
# Reference Data Fixtures
# ============================================================================

@pytest.fixture
def physical_constants():
    """Physical constants"""
    return {
        'g': 9.81,          # Gravity [m/s^2]
        'rho': 1.225,       # Air density at sea level [kg/m^3]
    }


@pytest.fixture
def test_scenarios():
    """Common test scenarios"""
    return {
        'level_flight': {
            'state': [0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0],
            'description': 'Steady level flight at 100m altitude, 25 m/s'
        },
        'climbing': {
            'state': [0, 0, -100, 25, 0, 2, 0, np.deg2rad(5), 0, 0, 0, 0],
            'description': 'Climbing flight with 5 deg pitch'
        },
        'descending': {
            'state': [0, 0, -100, 25, 0, -2, 0, np.deg2rad(-5), 0, 0, 0, 0],
            'description': 'Descending flight with -5 deg pitch'
        },
        'banked_turn': {
            'state': [0, 0, -100, 25, 0, 0, np.deg2rad(30), 0, 0, 0, 0, 0],
            'description': '30 degree banked turn'
        }
    }


@pytest.fixture
def numerical_tolerances():
    """Numerical tolerance criteria for various tests"""
    return {
        'rotation_matrix_orthogonality': 1e-10,
        'rotation_matrix_determinant': 1e-10,
        'energy_conservation': 1e-6,
        'theoretical_comparison': 0.05,  # 5%
        'control_performance': 0.10,     # 10%
        'integration_accuracy': 1e-4,
        'steady_state_error': 0.05,      # 5%
    }
