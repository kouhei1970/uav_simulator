"""
Reference data and theoretical values for validation tests

This module contains expected values, theoretical calculations, and
reference data for validating the UAV simulator implementation.
"""

import numpy as np


# ============================================================================
# Physical Constants
# ============================================================================

GRAVITY = 9.81  # m/s^2
AIR_DENSITY_SEA_LEVEL = 1.225  # kg/m^3


# ============================================================================
# Theoretical Calculations
# ============================================================================

def theoretical_bank_angle_steady_turn(airspeed, radius, gravity=GRAVITY):
    """
    Calculate theoretical bank angle for steady coordinated turn

    φ = arctan(V^2 / (g * R))

    Parameters:
        airspeed: Airspeed [m/s]
        radius: Turn radius [m]
        gravity: Gravitational acceleration [m/s^2]

    Returns:
        Bank angle [rad]
    """
    return np.arctan(airspeed**2 / (gravity * radius))


def theoretical_turn_rate(airspeed, radius):
    """
    Calculate theoretical turn rate for steady turn

    ω = V / R

    Parameters:
        airspeed: Airspeed [m/s]
        radius: Turn radius [m]

    Returns:
        Turn rate [rad/s]
    """
    return airspeed / radius


def theoretical_load_factor(bank_angle):
    """
    Calculate load factor in steady turn

    n = 1 / cos(φ)

    Parameters:
        bank_angle: Bank angle [rad]

    Returns:
        Load factor [dimensionless]
    """
    return 1.0 / np.cos(bank_angle)


def theoretical_lift_in_turn(weight, bank_angle):
    """
    Calculate required lift in steady turn

    L = W / cos(φ)

    Parameters:
        weight: Aircraft weight [N]
        bank_angle: Bank angle [rad]

    Returns:
        Required lift [N]
    """
    return weight / np.cos(bank_angle)


def dynamic_pressure(airspeed, density=AIR_DENSITY_SEA_LEVEL):
    """
    Calculate dynamic pressure

    q = 0.5 * ρ * V^2

    Parameters:
        airspeed: Airspeed [m/s]
        density: Air density [kg/m^3]

    Returns:
        Dynamic pressure [Pa]
    """
    return 0.5 * density * airspeed**2


def lift_from_coefficient(CL, airspeed, wing_area, density=AIR_DENSITY_SEA_LEVEL):
    """
    Calculate lift from lift coefficient

    L = CL * q * S = CL * 0.5 * ρ * V^2 * S

    Parameters:
        CL: Lift coefficient
        airspeed: Airspeed [m/s]
        wing_area: Wing area [m^2]
        density: Air density [kg/m^3]

    Returns:
        Lift force [N]
    """
    q = dynamic_pressure(airspeed, density)
    return CL * q * wing_area


def rotation_matrix_321(phi, theta, psi):
    """
    3-2-1 Euler angle rotation matrix (body to NED)

    Parameters:
        phi: Roll angle [rad]
        theta: Pitch angle [rad]
        psi: Yaw angle [rad]

    Returns:
        3x3 rotation matrix
    """
    # Rotation about z-axis (yaw)
    R_z = np.array([
        [np.cos(psi), -np.sin(psi), 0],
        [np.sin(psi),  np.cos(psi), 0],
        [0,            0,           1]
    ])

    # Rotation about y-axis (pitch)
    R_y = np.array([
        [ np.cos(theta), 0, np.sin(theta)],
        [ 0,             1, 0            ],
        [-np.sin(theta), 0, np.cos(theta)]
    ])

    # Rotation about x-axis (roll)
    R_x = np.array([
        [1, 0,            0           ],
        [0, np.cos(phi), -np.sin(phi)],
        [0, np.sin(phi),  np.cos(phi)]
    ])

    # Combined rotation (R_z * R_y * R_x)
    return R_z @ R_y @ R_x


# ============================================================================
# L1 Guidance Reference Calculations
# ============================================================================

def l1_distance(damping, period, airspeed):
    """
    Calculate L1 distance

    L1 = (1/π) * ζ * T * V_a

    Parameters:
        damping: Damping ratio ζ
        period: Period T [s]
        airspeed: Airspeed V_a [m/s]

    Returns:
        L1 distance [m]
    """
    return (1.0 / np.pi) * damping * period * airspeed


# ============================================================================
# Control Theory Reference Calculations
# ============================================================================

def first_order_time_constant(eigenvalue):
    """
    Calculate time constant from eigenvalue

    τ = -1 / λ  (for λ < 0)

    Parameters:
        eigenvalue: System eigenvalue λ [1/s]

    Returns:
        Time constant [s]
    """
    if eigenvalue >= 0:
        raise ValueError("Eigenvalue must be negative for stability")
    return -1.0 / eigenvalue


def proportional_guidance_eigenvalue(K_p, airspeed):
    """
    Calculate eigenvalue for proportional orbit guidance

    λ = -K_p * V_a

    Parameters:
        K_p: Proportional gain [rad/m]
        airspeed: Airspeed [m/s]

    Returns:
        Eigenvalue [1/s]
    """
    return -K_p * airspeed


def settling_time_first_order(time_constant, tolerance=0.02):
    """
    Calculate settling time for first-order system

    For 2% tolerance: t_s ≈ 4 * τ
    For 5% tolerance: t_s ≈ 3 * τ

    Parameters:
        time_constant: System time constant τ [s]
        tolerance: Settling tolerance (default 0.02 for 2%)

    Returns:
        Settling time [s]
    """
    if tolerance <= 0.02:
        return 4 * time_constant
    elif tolerance <= 0.05:
        return 3 * time_constant
    else:
        return 2 * time_constant


# ============================================================================
# Reference Test Scenarios
# ============================================================================

REFERENCE_SCENARIOS = {
    'trim_level_flight': {
        'description': 'Trimmed level flight',
        'airspeed': 25.0,  # m/s
        'altitude': 100.0,  # m
        'gamma': 0.0,  # rad (flight path angle)
        'turn_radius': np.inf,
        'expected_alpha': np.deg2rad(2.0),  # Approximate for small UAV
        'expected_throttle': 0.5,  # Approximate
    },
    'steady_turn': {
        'description': 'Steady coordinated turn',
        'airspeed': 25.0,  # m/s
        'turn_radius': 100.0,  # m
        'bank_angle': theoretical_bank_angle_steady_turn(25.0, 100.0),
        'turn_rate': theoretical_turn_rate(25.0, 100.0),
        'load_factor': theoretical_load_factor(
            theoretical_bank_angle_steady_turn(25.0, 100.0)
        ),
    },
    'gps_noise': {
        'description': 'GPS measurement noise characteristics',
        'horizontal_std': 2.5,  # m
        'vertical_std': 3.0,  # m
        'drift_magnitude': 1.0,  # m
        'outlier_probability': 0.002,  # 0.2%
        'outlier_magnitude': 15.0,  # m
    },
    'orbit_estimation': {
        'description': 'Orbit parameter estimation',
        'convergence_time_max': 15.0,  # s
        'center_accuracy': 5.0,  # m
        'radius_accuracy': 3.0,  # m
    },
}


# ============================================================================
# Expected Control Performance
# ============================================================================

CONTROL_PERFORMANCE_SPECS = {
    'attitude_control': {
        'settling_time_max': 3.0,  # s
        'overshoot_max': 0.2,  # 20%
        'steady_state_error_max': 0.05,  # 5%
    },
    'altitude_control': {
        'settling_time_max': 10.0,  # s
        'overshoot_max': 0.15,  # 15%
        'steady_state_error_max': 5.0,  # m
    },
    'airspeed_control': {
        'settling_time_max': 5.0,  # s
        'overshoot_max': 0.1,  # 10%
        'steady_state_error_max': 0.5,  # m/s
    },
}


# ============================================================================
# Numerical Integration Test Data
# ============================================================================

def generate_free_fall_reference(t_max, dt, initial_altitude):
    """
    Generate reference trajectory for free fall

    h(t) = h_0 - 0.5 * g * t^2
    v(t) = -g * t

    Parameters:
        t_max: Maximum time [s]
        dt: Time step [s]
        initial_altitude: Initial altitude (positive up) [m]

    Returns:
        Dictionary with 'time', 'altitude', 'velocity' arrays
    """
    time = np.arange(0, t_max, dt)
    altitude = initial_altitude - 0.5 * GRAVITY * time**2
    velocity = -GRAVITY * time

    return {
        'time': time,
        'altitude': altitude,
        'velocity': velocity,
    }
