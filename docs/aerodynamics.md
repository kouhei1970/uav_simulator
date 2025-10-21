# Aerodynamics Module

## Overview

The `aerodynamics.py` module implements the aerodynamic force and moment calculations for fixed-wing UAVs. It models lift, drag, and side forces, as well as roll, pitch, and yaw moments based on aerodynamic coefficients and control surface deflections.

## Class: AerodynamicModel

### Description

The `AerodynamicModel` class computes aerodynamic forces and moments acting on the aircraft based on flight conditions and control inputs.

### Initialization

```python
from src.aerodynamics import AerodynamicModel

# Initialize with default small aircraft parameters
aero = AerodynamicModel()

# Initialize with specific aircraft type
aero = AerodynamicModel(aircraft_type='medium')

# Initialize with custom parameters
custom_params = {
    'S_wing': 0.3,
    'b': 1.6,
    'c': 0.19,
    'C_L_0': 0.23,
    # ... other coefficients
}
aero = AerodynamicModel(params=custom_params)
```

### Available Aircraft Types

- `'micro'`: Micro UAV (1.0m wingspan)
- `'small'`: Small UAV (1.6m wingspan) - default
- `'medium'`: Medium UAV (2.9m wingspan)
- `'large'`: Large UAV (4.0m wingspan)

Stability variants are also available:
- `'small'`: Stable configuration
- `'small_slightly_unstable'`: Slightly unstable configuration
- `'small_unstable'`: Unstable configuration

## Key Methods

### compute_forces_moments(uav, control)

Computes total aerodynamic forces and moments.

**Parameters:**
- `uav`: FixedWingUAV object containing current state
- `control`: Control input array `[delta_e, delta_a, delta_r, delta_t]`

**Returns:**
- `forces_moments`: Array `[Fx, Fy, Fz, L, M, N]`
  - `Fx, Fy, Fz`: Forces in body frame [N]
  - `L, M, N`: Moments (roll, pitch, yaw) [N⋅m]

```python
forces_moments = aero.compute_forces_moments(uav, control)
Fx, Fy, Fz = forces_moments[0:3]  # Forces
L, M, N = forces_moments[3:6]     # Moments
```

### get_alpha_beta(uav)

Calculate angle of attack (α) and sideslip angle (β).

```python
alpha, beta = aero.get_alpha_beta(uav)
```

### get_lift_drag(uav)

Calculate lift and drag coefficients.

```python
C_L, C_D = aero.get_lift_drag(uav)
```

## Aerodynamic Coefficients

### Lift Coefficient (C_L)
```
C_L = C_L_0 + C_L_alpha * α + C_L_q * (c*q)/(2*Va) + C_L_delta_e * δe
```

### Drag Coefficient (C_D)
```
C_D = C_D_0 + C_D_alpha * α + C_D_q * (c*q)/(2*Va) + C_D_delta_e * δe
```

### Side Force Coefficient (C_Y)
```
C_Y = C_Y_0 + C_Y_beta * β + C_Y_p * (b*p)/(2*Va) + C_Y_r * (b*r)/(2*Va)
      + C_Y_delta_a * δa + C_Y_delta_r * δr
```

### Rolling Moment Coefficient (C_l)
```
C_l = C_l_0 + C_l_beta * β + C_l_p * (b*p)/(2*Va) + C_l_r * (b*r)/(2*Va)
      + C_l_delta_a * δa + C_l_delta_r * δr
```

### Pitching Moment Coefficient (C_m)
```
C_m = C_m_0 + C_m_alpha * α + C_m_q * (c*q)/(2*Va) + C_m_delta_e * δe
```

### Yawing Moment Coefficient (C_n)
```
C_n = C_n_0 + C_n_beta * β + C_n_p * (b*p)/(2*Va) + C_n_r * (b*r)/(2*Va)
      + C_n_delta_a * δa + C_n_delta_r * δr
```

## Stability Derivatives

### Longitudinal Stability
- **C_L_alpha**: Lift curve slope (typically > 0)
- **C_m_alpha**: Pitch stability derivative (negative = stable)
- **C_m_q**: Pitch damping (negative = damped)

### Lateral Stability
- **C_l_beta**: Dihedral effect (negative = stable)
- **C_l_p**: Roll damping (negative = damped)

### Directional Stability
- **C_n_beta**: Weathercock stability (positive = stable)
- **C_n_r**: Yaw damping (negative = damped)

## Control Derivatives

### Longitudinal Control
- **C_L_delta_e**: Elevator effectiveness for lift
- **C_m_delta_e**: Elevator effectiveness for pitch moment

### Lateral Control
- **C_l_delta_a**: Aileron effectiveness for roll moment
- **C_Y_delta_a**: Aileron effect on side force

### Directional Control
- **C_n_delta_r**: Rudder effectiveness for yaw moment
- **C_Y_delta_r**: Rudder effectiveness for side force

## Forces and Moments Calculation

### Aerodynamic Forces in Wind Frame
```
Lift = 0.5 * ρ * Va² * S * C_L
Drag = 0.5 * ρ * Va² * S * C_D
Side = 0.5 * ρ * Va² * S * C_Y
```

### Transformation to Body Frame
Forces in wind frame are rotated to body frame using angle of attack (α) and sideslip angle (β).

### Aerodynamic Moments
```
L (roll)  = 0.5 * ρ * Va² * S * b * C_l
M (pitch) = 0.5 * ρ * Va² * S * c * C_m
N (yaw)   = 0.5 * ρ * Va² * S * b * C_n
```

### Propulsion Forces
Thrust is modeled as a function of throttle setting:
```
Thrust = 0.5 * ρ * S_prop * C_prop * [(k_motor * δt)² - Va²]
```

## Example Usage

### Basic Force Calculation

```python
import numpy as np
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel

# Initialize
uav = FixedWingUAV(aircraft_type='small')
aero = AerodynamicModel(aircraft_type='small')

# Set flight condition
uav.set_state([0, 0, -100, 15, 0, 0, 0, 0.1, 0, 0, 0, 0])  # 15 m/s, 0.1 rad pitch

# Control inputs: [elevator, aileron, rudder, throttle]
control = np.array([0.05, 0.0, 0.0, 0.6])

# Calculate forces and moments
forces_moments = aero.compute_forces_moments(uav, control)
print(f"Fx={forces_moments[0]:.2f} N")
print(f"Fy={forces_moments[1]:.2f} N")
print(f"Fz={forces_moments[2]:.2f} N")
print(f"L={forces_moments[3]:.2f} N⋅m")
print(f"M={forces_moments[4]:.2f} N⋅m")
print(f"N={forces_moments[5]:.2f} N⋅m")
```

### Analyzing Flight Condition

```python
# Get aerodynamic angles
alpha, beta = aero.get_alpha_beta(uav)
print(f"Angle of attack: {np.rad2deg(alpha):.2f} deg")
print(f"Sideslip angle: {np.rad2deg(beta):.2f} deg")

# Get lift and drag coefficients
C_L, C_D = aero.get_lift_drag(uav)
print(f"Lift coefficient: {C_L:.4f}")
print(f"Drag coefficient: {C_D:.4f}")

# Calculate lift-to-drag ratio
L_D_ratio = C_L / C_D if C_D > 0 else 0
print(f"L/D ratio: {L_D_ratio:.2f}")
```

### Stability Analysis

```python
from config.aircraft_params import print_aircraft_info

# Print stability information for different variants
print_aircraft_info('small')
print_aircraft_info('small_slightly_unstable')
print_aircraft_info('small_unstable')
```

## Stability Configurations

### Stable Configuration (default 'small')
- Roll: C_l_beta = -0.15 (stable)
- Pitch: C_m_alpha = -0.50 (stable)
- Yaw: C_n_beta = 0.30 (stable)

### Slightly Unstable Configuration
- Roll: C_l_beta = 0.03 (slightly unstable)
- Pitch: C_m_alpha = 0.05 (slightly unstable)
- Yaw: C_n_beta = 0.05 (weakly stable)

### Unstable Configuration
- Roll: C_l_beta = 0.10 (unstable)
- Pitch: C_m_alpha = 0.15 (unstable)
- Yaw: C_n_beta = -0.15 (unstable)

## Integration with Dynamics

```python
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel

uav = FixedWingUAV()
aero = AerodynamicModel()

dt = 0.01
for i in range(1000):
    # Set control
    control = np.array([0.0, 0.0, 0.0, 0.6])

    # Compute aerodynamics
    forces_moments = aero.compute_forces_moments(uav, control)

    # Update dynamics
    uav.update(dt, forces_moments)
```

## Wind Effects

The current implementation assumes calm air (no wind). Wind effects can be added by modifying the airspeed calculation to account for wind velocity.

## See Also

- [Dynamics Module](dynamics.md) - 6-DOF dynamics model
- [Controller Module](controller.md) - Control law implementation
- [Aircraft Generator](aircraft_generator.md) - Custom aircraft creation
- [Usage Guide](usage_guide.md) - Complete simulation examples
