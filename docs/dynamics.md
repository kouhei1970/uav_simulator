# Dynamics Module

## Overview

The `dynamics.py` module implements a 6-DOF (6 Degrees of Freedom) nonlinear dynamics model for fixed-wing UAVs. It simulates the aircraft's motion in 3D space using the NED (North-East-Down) coordinate system.

## Class: FixedWingUAV

### Description

The `FixedWingUAV` class represents the complete dynamics model of a fixed-wing UAV, including position, velocity, attitude, and angular rates.

### State Vector

The 12-dimensional state vector consists of:

```python
state = [pn, pe, pd, u, v, w, phi, theta, psi, p, q, r]
```

- **Position (NED frame)**: `pn` (North), `pe` (East), `pd` (Down)
- **Velocity (body frame)**: `u` (forward), `v` (lateral), `w` (downward)
- **Attitude (Euler angles)**: `phi` (roll), `theta` (pitch), `psi` (yaw)
- **Angular rates (body frame)**: `p` (roll rate), `q` (pitch rate), `r` (yaw rate)

### Initialization

```python
from src.dynamics import FixedWingUAV

# Initialize with default small aircraft (1.6m wingspan, 1.7kg)
uav = FixedWingUAV()

# Initialize with specific aircraft type
uav = FixedWingUAV(aircraft_type='medium')

# Initialize with custom parameters
custom_params = {
    'mass': 2.0,
    'Jx': 0.5,
    'Jy': 0.8,
    'Jz': 1.2,
    # ... other parameters
}
uav = FixedWingUAV(params=custom_params)
```

### Available Aircraft Types

- `'micro'`: 1.0m wingspan, 0.8kg mass
- `'small'`: 1.6m wingspan, 1.7kg mass (default)
- `'medium'`: 2.9m wingspan, 11.0kg mass
- `'large'`: 4.0m wingspan, 25.0kg mass

### Key Methods

#### set_state(state)
Set the complete state vector.

```python
# Set initial state: position (0,0,-100m), velocity 20 m/s forward, level attitude
uav.set_state([0, 0, -100, 20, 0, 0, 0, 0, 0, 0, 0, 0])
```

#### set_control(control)
Set control inputs.

```python
# control = [delta_e, delta_a, delta_r, delta_t]
# delta_e: elevator deflection (rad)
# delta_a: aileron deflection (rad)
# delta_r: rudder deflection (rad)
# delta_t: throttle (0-1)
control = np.array([0.0, 0.0, 0.0, 0.6])
uav.set_control(control)
```

#### update(dt, forces_moments)
Update the state using RK4 integration.

```python
# Calculate forces and moments from aerodynamic model
forces_moments = aero.compute_forces_moments(uav, control)

# Update state with time step
dt = 0.01  # 10ms
uav.update(dt, forces_moments)
```

#### get_state()
Get the complete state vector.

```python
state = uav.get_state()
```

#### get_position()
Get position in NED frame.

```python
position = uav.get_position()  # [pn, pe, pd]
```

#### get_attitude()
Get attitude angles (Euler angles).

```python
phi, theta, psi = uav.get_attitude()  # roll, pitch, yaw (radians)
```

#### get_airspeed()
Calculate and return airspeed magnitude.

```python
Va = uav.get_airspeed()  # airspeed in m/s
```

## Coordinate Systems

### NED Frame (North-East-Down)
- **North**: Positive direction points north
- **East**: Positive direction points east
- **Down**: Positive direction points down (altitude is negative)

### Body Frame
- **x-axis**: Points forward (along fuselage)
- **y-axis**: Points right (starboard wing)
- **z-axis**: Points down

## Dynamics Equations

The model implements standard 6-DOF equations:

### Translational Dynamics
```
ṗn = cos(θ)cos(ψ)u + (sin(φ)sin(θ)cos(ψ) - cos(φ)sin(ψ))v + ...
ṗe = cos(θ)sin(ψ)u + (sin(φ)sin(θ)sin(ψ) + cos(φ)cos(ψ))v + ...
ṗd = sin(θ)u - sin(φ)cos(θ)v - cos(φ)cos(θ)w

u̇ = rv - qw - g·sin(θ) + Fx/m
v̇ = pw - ru + g·cos(θ)sin(φ) + Fy/m
ẇ = qu - pv + g·cos(θ)cos(φ) + Fz/m
```

### Rotational Dynamics
```
φ̇ = p + q·sin(φ)tan(θ) + r·cos(φ)tan(θ)
θ̇ = q·cos(φ) - r·sin(φ)
ψ̇ = q·sin(φ)/cos(θ) + r·cos(φ)/cos(θ)

ṗ = (Jy - Jz)/Jx · qr + L/Jx
q̇ = (Jz - Jx)/Jy · pr + M/Jy
ṙ = (Jx - Jy)/Jz · pq + N/Jz
```

## Integration Method

The module uses **4th-order Runge-Kutta (RK4)** integration for numerical stability and accuracy.

## Example Usage

```python
import numpy as np
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel

# Initialize UAV and aerodynamic model
uav = FixedWingUAV(aircraft_type='small')
aero = AerodynamicModel(aircraft_type='small')

# Set initial state: 100m altitude, 20 m/s airspeed
uav.set_state([0, 0, -100, 20, 0, 0, 0, 0, 0, 0, 0, 0])

# Simulation loop
dt = 0.01  # 10ms time step
for i in range(1000):
    # Set control inputs
    control = np.array([0.0, 0.0, 0.0, 0.6])  # level flight, 60% throttle
    uav.set_control(control)

    # Compute aerodynamic forces
    forces_moments = aero.compute_forces_moments(uav, control)

    # Update dynamics
    uav.update(dt, forces_moments)

    # Get current state
    position = uav.get_position()
    altitude = -position[2]  # Convert down to altitude
    airspeed = uav.get_airspeed()

    print(f"Time: {i*dt:.2f}s, Altitude: {altitude:.1f}m, Airspeed: {airspeed:.1f}m/s")
```

## Parameters

Key physical parameters stored in `uav.params`:

- `mass`: Aircraft mass [kg]
- `Jx`, `Jy`, `Jz`: Moments of inertia [kg⋅m²]
- `Jxz`: Product of inertia [kg⋅m²]
- `S_wing`: Wing area [m²]
- `b`: Wingspan [m]
- `c`: Mean aerodynamic chord [m]
- `rho`: Air density [kg/m³]
- `gravity`: Gravitational acceleration [m/s²]

## See Also

- [Aerodynamics Module](aerodynamics.md) - Aerodynamic force and moment calculations
- [Controller Module](controller.md) - Control law implementation
- [Usage Guide](usage_guide.md) - Complete simulation examples
