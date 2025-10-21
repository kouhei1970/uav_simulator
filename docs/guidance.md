# Guidance Module

## Overview

The `guidance.py` module implements high-level guidance laws for path following and navigation. It provides waypoint navigation, straight-line following, orbit patterns, and coordinated turn guidance for fixed-wing UAVs.

## Guidance Classes

### WaypointGuidance

Manages sequential waypoint navigation with automatic waypoint switching.

#### Initialization

```python
from src.guidance import WaypointGuidance
import numpy as np

# Define waypoints (North, East, Down) in NED frame
waypoints = np.array([
    [0, 0, -100],      # Start at origin, 100m altitude
    [500, 0, -100],    # Fly north 500m
    [500, 500, -150],  # Turn east, climb to 150m
    [0, 0, -100]       # Return home
])

# Initialize guidance with capture radius
guidance = WaypointGuidance(waypoints, R_min=50.0)
```

#### Key Methods

##### update(position)

Updates the current waypoint based on position and returns the target waypoint.

**Parameters:**
- `position`: Current position [pn, pe, pd] in NED frame

**Returns:**
- `waypoint`: Current target waypoint [pn, pe, pd]
- `completed`: Boolean indicating if all waypoints reached

```python
position = uav.get_position()
waypoint, completed = guidance.update(position)

if completed:
    print("All waypoints reached!")
```

##### compute_heading_command(position)

Computes the desired heading angle to fly toward the current waypoint.

**Parameters:**
- `position`: Current position [pn, pe, pd] in NED frame

**Returns:**
- `psi_c`: Commanded heading angle [rad]

```python
psi_c = guidance.compute_heading_command(position)
```

#### Waypoint Switching Logic

A waypoint is considered reached when the aircraft enters a sphere of radius `R_min` centered on the waypoint:

```
distance_to_waypoint = ||position - waypoint||
if distance_to_waypoint < R_min:
    switch to next waypoint
```

#### Properties

```python
# Get current waypoint index
current_idx = guidance.current_waypoint_index

# Check if mission completed
is_complete = guidance.completed

# Get total number of waypoints
num_waypoints = len(guidance.waypoints)
```

### StraightLineGuidance

Follows a straight line path between two points using vector field guidance.

#### Initialization

```python
from src.guidance import StraightLineGuidance
import numpy as np

# Define line by start and end points
r_start = np.array([0, 0, -100])    # Line start
r_end = np.array([1000, 500, -150])  # Line end

guidance = StraightLineGuidance(r_start, r_end, chi_inf=np.deg2rad(50))
```

#### Parameters

- `r_start`: Line start point [pn, pe, pd]
- `r_end`: Line end point [pn, pe, pd]
- `chi_inf`: Maximum approach angle [rad] (default: 50°)

#### Key Method: compute_heading_command(position, k=0.01)

Computes heading to follow the line using vector field guidance.

**Parameters:**
- `position`: Current position [pn, pe, pd]
- `k`: Guidance gain (default: 0.01)

**Returns:**
- `psi_c`: Commanded heading angle [rad]

```python
position = uav.get_position()
psi_c = guidance.compute_heading_command(position, k=0.01)
```

#### Vector Field Guidance

The guidance law uses a vector field that smoothly transitions from approach to line-following:

```
chi_q = atan2(line_direction)  # Line direction angle
chi_d = cross_track_error dependent angle
psi_c = chi_q - chi_inf * (2/π) * atan(k * cross_track_error)
```

### OrbitGuidance

Maintains circular orbit around a specified center point.

#### Initialization

```python
from src.guidance import OrbitGuidance
import numpy as np

# Define orbit parameters
center = np.array([500, 500, -150])  # Orbit center (North, East, Down)
radius = 200.0                        # Orbit radius [m]
direction = 'CW'                      # 'CW' (clockwise) or 'CCW' (counter-clockwise)

guidance = OrbitGuidance(center, radius, direction)
```

#### Key Methods

##### compute_heading_command(position, k_orbit=1.0)

Computes heading to maintain circular orbit.

**Parameters:**
- `position`: Current position [pn, pe, pd]
- `k_orbit`: Orbit guidance gain (default: 1.0)

**Returns:**
- `psi_c`: Commanded heading angle [rad]

```python
position = uav.get_position()
psi_c = guidance.compute_heading_command(position, k_orbit=2.0)
```

##### compute_radius_error(position)

Calculates the deviation from desired orbit radius.

**Parameters:**
- `position`: Current position [pn, pe, pd]

**Returns:**
- `radius_error`: Distance from desired radius [m]

```python
radius_error = guidance.compute_radius_error(position)
print(f"Orbit radius error: {radius_error:.1f} m")
```

#### Orbit Control Law

```
d = distance from orbit center
e_r = d - R_orbit  # Radial error

For CW orbit:  psi_c = atan2(e_y, e_x) + π/2 - atan(k_orbit * e_r)
For CCW orbit: psi_c = atan2(e_y, e_x) - π/2 + atan(k_orbit * e_r)
```

### CoordinatedTurnGuidance

Generates roll angle commands for coordinated turns while tracking a heading.

#### Initialization

```python
from src.guidance import CoordinatedTurnGuidance

# Initialize with nominal airspeed
guidance = CoordinatedTurnGuidance(V_a=25.0)
```

#### Key Method: compute_roll_command(psi, psi_c, k_psi=0.5)

Computes the required roll angle for coordinated turn to desired heading.

**Parameters:**
- `psi`: Current heading [rad]
- `psi_c`: Commanded heading [rad]
- `k_psi`: Heading gain (default: 0.5)

**Returns:**
- `phi_c`: Commanded roll angle [rad]

```python
phi, theta, psi = uav.get_attitude()
psi_c = guidance_law.compute_heading_command(position)
phi_c = turn_guidance.compute_roll_command(psi, psi_c, k_psi=0.5)
```

#### Coordinated Turn Dynamics

```
phi_ff = atan2(V_a² * psi_dot, g)  # Feedforward term
phi_fb = k_psi * (psi_c - psi)      # Feedback term
phi_c = phi_ff + phi_fb             # Total roll command
```

Roll command is limited to prevent excessive bank angles (default: ±45°).

### PathManager

High-level manager that coordinates different guidance modes.

#### Initialization

```python
from src.guidance import PathManager
import numpy as np

waypoints = np.array([
    [0, 0, -100],
    [500, 0, -100],
    [500, 500, -150]
])

path_manager = PathManager(waypoints)
```

#### Key Method: update(position)

Updates the current guidance mode and returns guidance commands.

**Parameters:**
- `position`: Current position [pn, pe, pd]

**Returns:**
- `psi_c`: Commanded heading [rad]
- `h_c`: Commanded altitude [m]
- `completed`: Boolean indicating mission completion

```python
position = uav.get_position()
psi_c, h_c, completed = path_manager.update(position)
```

#### Guidance Modes

PathManager can switch between:
- Waypoint-to-waypoint navigation
- Straight line following
- Orbit holding

## Complete Examples

### Waypoint Navigation

```python
import numpy as np
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel
from src.controller import AttitudeController, AltitudeController, AirspeedController
from src.guidance import WaypointGuidance, CoordinatedTurnGuidance

# Initialize
uav = FixedWingUAV()
aero = AerodynamicModel()
attitude_ctrl = AttitudeController()
altitude_ctrl = AltitudeController()
airspeed_ctrl = AirspeedController()

# Define mission
waypoints = np.array([
    [0, 0, -100],
    [500, 0, -100],
    [500, 500, -150],
    [0, 500, -150],
    [0, 0, -100]
])

waypoint_guidance = WaypointGuidance(waypoints, R_min=50.0)
turn_guidance = CoordinatedTurnGuidance(V_a=25.0)

# Initial state
uav.set_state([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])

# Simulation parameters
dt = 0.01
Va_c = 25.0

# Mission execution
time = 0.0
while not waypoint_guidance.completed:
    # Get current state
    position = uav.get_position()
    phi, theta, psi = uav.get_attitude()

    # Guidance layer
    waypoint, completed = waypoint_guidance.update(position)
    psi_c = waypoint_guidance.compute_heading_command(position)
    h_c = waypoint[2]

    # Convert heading to roll command
    phi_c = turn_guidance.compute_roll_command(psi, psi_c, k_psi=0.5)

    # Control layer
    theta_c = altitude_ctrl.compute_pitch_command(uav, h_c, dt)
    delta_t = airspeed_ctrl.compute_throttle_command(uav, Va_c, dt)
    delta_a, delta_e, delta_r = attitude_ctrl.compute_control(uav, phi_c, theta_c, dt)

    # Apply control
    control = np.array([delta_e, delta_a, delta_r, delta_t])
    uav.set_control(control)

    # Update dynamics
    forces_moments = aero.compute_forces_moments(uav, control)
    uav.update(dt, forces_moments)

    time += dt

print("Mission complete!")
```

### Orbit Pattern

```python
from src.guidance import OrbitGuidance

# Define orbit
orbit_center = np.array([500, 500, -150])
orbit_radius = 200.0
orbit_direction = 'CW'

orbit_guidance = OrbitGuidance(orbit_center, orbit_radius, orbit_direction)
turn_guidance = CoordinatedTurnGuidance(V_a=25.0)

# Orbit execution
time = 0.0
T_orbit = 120.0  # Orbit for 2 minutes

while time < T_orbit:
    position = uav.get_position()
    phi, theta, psi = uav.get_attitude()

    # Orbit guidance
    psi_c = orbit_guidance.compute_heading_command(position, k_orbit=2.0)
    h_c = orbit_center[2]

    # Coordinated turn
    phi_c = turn_guidance.compute_roll_command(psi, psi_c, k_psi=0.8)

    # Control (altitude and attitude)
    theta_c = altitude_ctrl.compute_pitch_command(uav, h_c, dt)
    delta_t = airspeed_ctrl.compute_throttle_command(uav, Va_c, dt)
    delta_a, delta_e, delta_r = attitude_ctrl.compute_control(uav, phi_c, theta_c, dt)

    # Apply and update
    control = np.array([delta_e, delta_a, delta_r, delta_t])
    uav.set_control(control)
    forces_moments = aero.compute_forces_moments(uav, control)
    uav.update(dt, forces_moments)

    # Monitor orbit quality
    if int(time * 10) % 100 == 0:  # Every 10 seconds
        radius_error = orbit_guidance.compute_radius_error(position)
        print(f"Time: {time:.1f}s, Radius error: {radius_error:.1f}m")

    time += dt
```

### Straight Line Following

```python
from src.guidance import StraightLineGuidance

# Define line path
r_start = np.array([0, 0, -100])
r_end = np.array([1000, 500, -150])

line_guidance = StraightLineGuidance(r_start, r_end, chi_inf=np.deg2rad(50))
turn_guidance = CoordinatedTurnGuidance(V_a=25.0)

# Follow line
time = 0.0
T_sim = 80.0

while time < T_sim:
    position = uav.get_position()
    phi, theta, psi = uav.get_attitude()

    # Line following guidance
    psi_c = line_guidance.compute_heading_command(position, k=0.01)
    h_c = r_end[2]  # Target altitude at end of line

    # Coordinated turn
    phi_c = turn_guidance.compute_roll_command(psi, psi_c, k_psi=0.6)

    # Control and update
    theta_c = altitude_ctrl.compute_pitch_command(uav, h_c, dt)
    delta_t = airspeed_ctrl.compute_throttle_command(uav, Va_c, dt)
    delta_a, delta_e, delta_r = attitude_ctrl.compute_control(uav, phi_c, theta_c, dt)

    control = np.array([delta_e, delta_a, delta_r, delta_t])
    uav.set_control(control)
    forces_moments = aero.compute_forces_moments(uav, control)
    uav.update(dt, forces_moments)

    time += dt
```

## Tuning Guidelines

### Waypoint Guidance
- **R_min**: Larger values → earlier waypoint switching (less accurate)
- Typical range: 30-100m depending on aircraft size

### Coordinated Turn
- **k_psi**: Higher gain → faster heading response
- Typical range: 0.3-1.0
- Too high → oscillatory heading tracking

### Orbit Guidance
- **k_orbit**: Higher gain → tighter orbit tracking
- Typical range: 1.0-3.0
- Monitor radius_error for performance

### Straight Line
- **k**: Higher gain → faster convergence to line
- **chi_inf**: Larger angle → more direct approach
- Typical chi_inf: 40-60 degrees

## Integration with Control

The guidance layer sits above the control layer:

```
Guidance Layer:  position → heading command, altitude command
    ↓
Turn Coordination: heading command → roll command
    ↓
Control Layer:   roll/pitch commands → control surface deflections
    ↓
Dynamics:        control surfaces → aircraft motion
```

## See Also

- [Controller Module](controller.md) - Low-level control implementation
- [Dynamics Module](dynamics.md) - Aircraft dynamics
- [Usage Guide](usage_guide.md) - Complete simulation examples
