# Controller Module

## Overview

The `controller.py` module implements control laws for fixed-wing UAVs, including attitude control, altitude control, and airspeed control. The controllers use PID (Proportional-Integral-Derivative) algorithms with cascade control architecture.

## Controller Classes

### AttitudeController

Controls the aircraft's attitude (roll, pitch, yaw) by generating control surface commands.

#### Initialization

```python
from src.controller import AttitudeController

# Initialize with default gains
attitude_controller = AttitudeController()
```

#### Key Method: compute_control(uav, phi_c, theta_c, dt)

Computes control surface deflections to track desired roll and pitch angles.

**Parameters:**
- `uav`: FixedWingUAV object containing current state
- `phi_c`: Commanded roll angle [rad]
- `theta_c`: Commanded pitch angle [rad]
- `dt`: Time step [s]

**Returns:**
- `delta_a`: Aileron deflection [rad]
- `delta_e`: Elevator deflection [rad]
- `delta_r`: Rudder deflection [rad]

```python
phi_c = 0.1  # 0.1 rad roll command
theta_c = 0.05  # 0.05 rad pitch command
delta_a, delta_e, delta_r = attitude_controller.compute_control(uav, phi_c, theta_c, dt)
```

#### Control Architecture

**Roll Control (Aileron):**
```
e_phi = phi_c - phi
delta_a = kp_phi * e_phi + kd_phi * (e_phi - e_phi_prev) / dt
```

**Pitch Control (Elevator):**
```
e_theta = theta_c - theta
delta_e = kp_theta * e_theta + kd_theta * (e_theta - e_theta_prev) / dt
```

**Yaw Damper (Rudder):**
```
delta_r = -kd_psi * r  # Yaw rate damping
```

#### Tunable Parameters

```python
# Roll control gains
attitude_controller.kp_phi = 0.8    # Proportional gain
attitude_controller.kd_phi = 0.1    # Derivative gain

# Pitch control gains
attitude_controller.kp_theta = 1.0  # Proportional gain
attitude_controller.kd_theta = 0.2  # Derivative gain

# Yaw damper gain
attitude_controller.kd_psi = 0.3    # Yaw rate damping
```

### AltitudeController

Controls the aircraft's altitude by generating pitch angle commands.

#### Initialization

```python
from src.controller import AltitudeController

# Initialize with default gains
altitude_controller = AltitudeController()
```

#### Key Method: compute_pitch_command(uav, h_c, dt)

Computes desired pitch angle to track commanded altitude.

**Parameters:**
- `uav`: FixedWingUAV object containing current state
- `h_c`: Commanded altitude in NED frame (negative for above ground) [m]
- `dt`: Time step [s]

**Returns:**
- `theta_c`: Commanded pitch angle [rad]

```python
h_c = -150.0  # Command 150m altitude (negative in NED)
theta_c = altitude_controller.compute_pitch_command(uav, h_c, dt)
```

#### Control Law

```
h = -pd  # Current altitude (convert from NED down)
e_h = h_c - h  # Altitude error
theta_c = kp_h * e_h + ki_h * integral(e_h) + kd_h * de_h/dt
```

Pitch command is saturated to prevent extreme maneuvers:
```
theta_c ∈ [-20°, 20°]
```

#### Tunable Parameters

```python
altitude_controller.kp_h = 0.01   # Proportional gain
altitude_controller.ki_h = 0.001  # Integral gain
altitude_controller.kd_h = 0.02   # Derivative gain
altitude_controller.theta_max = np.deg2rad(20)  # Max pitch angle
```

### AirspeedController

Controls the aircraft's airspeed by generating throttle commands.

#### Initialization

```python
from src.controller import AirspeedController

# Initialize with default gains
airspeed_controller = AirspeedController()
```

#### Key Method: compute_throttle_command(uav, Va_c, dt)

Computes throttle setting to track commanded airspeed.

**Parameters:**
- `uav`: FixedWingUAV object containing current state
- `Va_c`: Commanded airspeed [m/s]
- `dt`: Time step [s]

**Returns:**
- `delta_t`: Throttle setting [0-1]

```python
Va_c = 25.0  # Command 25 m/s airspeed
delta_t = airspeed_controller.compute_throttle_command(uav, Va_c, dt)
```

#### Control Law

```
Va = ||[u, v, w]||  # Current airspeed
e_Va = Va_c - Va  # Airspeed error
delta_t = kp_Va * e_Va + ki_Va * integral(e_Va)
```

Throttle is saturated to valid range:
```
delta_t ∈ [0, 1]
```

#### Tunable Parameters

```python
airspeed_controller.kp_Va = 0.1   # Proportional gain
airspeed_controller.ki_Va = 0.01  # Integral gain
```

### TECSController (Total Energy Control System)

Advanced controller that coordinates altitude and airspeed control by managing total energy and energy distribution.

#### Initialization

```python
from src.controller import TECSController

# Initialize with default parameters
tecs = TECSController()
```

#### Key Method: compute_commands(uav, h_c, Va_c, dt)

Computes both pitch and throttle commands simultaneously.

**Parameters:**
- `uav`: FixedWingUAV object containing current state
- `h_c`: Commanded altitude [m] (NED frame)
- `Va_c`: Commanded airspeed [m/s]
- `dt`: Time step [s]

**Returns:**
- `theta_c`: Commanded pitch angle [rad]
- `delta_t`: Throttle setting [0-1]

```python
h_c = -150.0  # 150m altitude
Va_c = 25.0   # 25 m/s airspeed
theta_c, delta_t = tecs.compute_commands(uav, h_c, Va_c, dt)
```

#### Energy Formulation

**Total Energy:**
```
E_total = m*g*h + 0.5*m*Va²
```

**Total Energy Rate:**
```
Ė_total = m*g*ḣ + m*Va*V̇a
```

**Energy Distribution (Balance):**
```
E_balance = m*g*h - 0.5*m*Va²
```

TECS uses throttle to control total energy and pitch to control energy distribution, providing coordinated altitude and airspeed control.

## Control Architecture

### Cascade Control Structure

```
Altitude Controller → Pitch Command → Attitude Controller → Elevator
Airspeed Controller → Throttle Command
```

### Typical Control Loop

```python
from src.controller import AttitudeController, AltitudeController, AirspeedController

# Initialize controllers
attitude_ctrl = AttitudeController()
altitude_ctrl = AltitudeController()
airspeed_ctrl = AirspeedController()

# Control loop
dt = 0.01
h_c = -150.0   # Target altitude
Va_c = 25.0    # Target airspeed
phi_c = 0.0    # Level wings

while True:
    # Outer loops: altitude and airspeed
    theta_c = altitude_ctrl.compute_pitch_command(uav, h_c, dt)
    delta_t = airspeed_ctrl.compute_throttle_command(uav, Va_c, dt)

    # Inner loop: attitude
    delta_a, delta_e, delta_r = attitude_ctrl.compute_control(uav, phi_c, theta_c, dt)

    # Apply control
    control = np.array([delta_e, delta_a, delta_r, delta_t])
    uav.set_control(control)
```

## Complete Example

### Basic Level Flight

```python
import numpy as np
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel
from src.controller import AttitudeController, AltitudeController, AirspeedController

# Initialize UAV
uav = FixedWingUAV()
aero = AerodynamicModel()

# Initialize controllers
attitude_controller = AttitudeController()
altitude_controller = AltitudeController()
airspeed_controller = AirspeedController()

# Initial state: 100m altitude, 25 m/s airspeed
uav.set_state([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])

# Target values
h_c = -150.0    # Climb to 150m
Va_c = 25.0     # Maintain 25 m/s
phi_c = 0.0     # Keep wings level

# Simulation parameters
dt = 0.01
T_sim = 60.0

time = 0.0
while time < T_sim:
    # Compute control commands
    theta_c = altitude_controller.compute_pitch_command(uav, h_c, dt)
    delta_t = airspeed_controller.compute_throttle_command(uav, Va_c, dt)
    delta_a, delta_e, delta_r = attitude_controller.compute_control(uav, phi_c, theta_c, dt)

    # Set control inputs
    control = np.array([delta_e, delta_a, delta_r, delta_t])
    uav.set_control(control)

    # Compute forces and update dynamics
    forces_moments = aero.compute_forces_moments(uav, control)
    uav.update(dt, forces_moments)

    time += dt

# Print final state
position = uav.get_position()
print(f"Final altitude: {-position[2]:.1f} m")
print(f"Final airspeed: {uav.get_airspeed():.1f} m/s")
```

### Turning Flight

```python
# Add coordinated turn capability
phi_c = np.deg2rad(15)  # Bank 15 degrees for turn

# Control loop with turn
for i in range(int(T_sim / dt)):
    theta_c = altitude_controller.compute_pitch_command(uav, h_c, dt)
    delta_t = airspeed_controller.compute_throttle_command(uav, Va_c, dt)
    delta_a, delta_e, delta_r = attitude_controller.compute_control(uav, phi_c, theta_c, dt)

    control = np.array([delta_e, delta_a, delta_r, delta_t])
    uav.set_control(control)

    forces_moments = aero.compute_forces_moments(uav, control)
    uav.update(dt, forces_moments)
```

## Tuning Guidelines

### Roll Controller
- Increase `kp_phi` for faster roll response
- Increase `kd_phi` to reduce roll oscillations
- Typical range: kp_phi ∈ [0.5, 1.5]

### Pitch Controller
- Increase `kp_theta` for faster pitch response
- Increase `kd_theta` to reduce pitch oscillations
- Typical range: kp_theta ∈ [0.5, 2.0]

### Altitude Controller
- Increase `kp_h` for faster altitude response
- Add `ki_h` to eliminate steady-state error
- Increase `kd_h` to reduce overshoot
- Typical range: kp_h ∈ [0.005, 0.02]

### Airspeed Controller
- Increase `kp_Va` for faster speed response
- Add `ki_Va` to eliminate steady-state error
- Typical range: kp_Va ∈ [0.05, 0.2]

## Advanced Features

### Anti-Windup

All controllers implement integrator anti-windup to prevent integral term saturation:

```python
# Example: Altitude controller anti-windup
if abs(self.integral_h) > self.max_integral:
    self.integral_h = np.sign(self.integral_h) * self.max_integral
```

### Saturation Limits

Control outputs are limited to realistic values:
- Elevator: ±25 degrees
- Aileron: ±25 degrees
- Rudder: ±25 degrees
- Throttle: 0 to 1
- Pitch command: ±20 degrees

## See Also

- [Dynamics Module](dynamics.md) - Aircraft dynamics model
- [Guidance Module](guidance.md) - High-level path following
- [Usage Guide](usage_guide.md) - Complete simulation examples
