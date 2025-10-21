# Visualization Module

## Overview

The `visualization.py` module provides comprehensive visualization tools for analyzing UAV simulation results. It offers 2D/3D trajectory plots, time series analysis, and data export capabilities.

## Class: SimulationVisualizer

### Description

The `SimulationVisualizer` class records simulation data and generates plots for analyzing flight performance.

### Initialization

```python
from src.visualization import SimulationVisualizer

# Create visualizer instance
viz = SimulationVisualizer()
```

## Data Recording

### add_data(time, state, control)

Records a snapshot of simulation data.

**Parameters:**
- `time`: Current simulation time [s]
- `state`: 12-element state vector [pn, pe, pd, u, v, w, phi, theta, psi, p, q, r]
- `control`: 4-element control vector [delta_e, delta_a, delta_r, delta_t]

```python
# Record data at each time step
time = 0.0
dt = 0.01

while time < sim_time:
    # ... simulation updates ...

    # Record every 0.1 seconds
    if step % 10 == 0:
        viz.add_data(time, uav.get_state(), control)

    time += dt
    step += 1
```

## Visualization Methods

### plot_3d_trajectory(waypoints=None)

Displays the 3D flight trajectory.

**Parameters:**
- `waypoints`: Optional array of waypoints to display [N×3 array]

**Features:**
- Trajectory colored by time progression
- Start position marked with green circle
- End position marked with red square
- Optional waypoint markers

```python
# Plot trajectory only
viz.plot_3d_trajectory()

# Plot trajectory with waypoints
waypoints = np.array([
    [0, 0, -100],
    [500, 0, -100],
    [500, 500, -150]
])
viz.plot_3d_trajectory(waypoints=waypoints)
```

**Plot Elements:**
- X-axis: North position [m]
- Y-axis: East position [m]
- Z-axis: Altitude [m] (positive up, converted from NED down)
- Color: Time progression (blue → yellow)

### plot_2d_trajectory(waypoints=None, orbit_center=None, orbit_radius=None)

Displays the 2D horizontal trajectory (top-down view).

**Parameters:**
- `waypoints`: Optional waypoint array [N×3]
- `orbit_center`: Optional orbit center position [3-element array]
- `orbit_radius`: Optional orbit radius [m]

**Features:**
- Top-down view of flight path
- Waypoint markers and labels
- Orbit circle visualization
- Start/end markers

```python
# Simple 2D trajectory
viz.plot_2d_trajectory()

# With waypoints
viz.plot_2d_trajectory(waypoints=waypoints)

# With orbit pattern
orbit_center = np.array([500, 500, -150])
orbit_radius = 200.0
viz.plot_2d_trajectory(orbit_center=orbit_center, orbit_radius=orbit_radius)
```

**Plot Elements:**
- X-axis: North position [m]
- Y-axis: East position [m]
- Green dot: Start position
- Red square: End position
- Blue circles: Waypoints

### plot_states()

Displays time series of all state variables.

**Subplots:**
1. **Position** (North, East, Altitude)
2. **Velocity** (u, v, w in body frame)
3. **Attitude** (Roll, Pitch, Yaw in degrees)
4. **Angular Rates** (p, q, r in deg/s)

```python
viz.plot_states()
```

### plot_controls()

Displays time series of control inputs.

**Subplots:**
1. **Elevator deflection** [deg]
2. **Aileron deflection** [deg]
3. **Rudder deflection** [deg]
4. **Throttle** [0-1]

```python
viz.plot_controls()
```

### plot_airdata()

Displays aerodynamic flight data.

**Subplots:**
1. **Airspeed** [m/s]
2. **Angle of Attack** [deg]
3. **Sideslip Angle** [deg]
4. **Flight Path Angle** [deg]

```python
viz.plot_airdata()
```

## Data Export

### save_data(filename)

Saves simulation data to a compressed NumPy archive.

**Parameters:**
- `filename`: Output filename (`.npz` extension)

**Saved Arrays:**
- `time`: Time history
- `states`: State history [N×12]
- `controls`: Control history [N×4]

```python
# Save data
viz.save_data('flight_data.npz')

# Load data later
data = np.load('flight_data.npz')
times = data['time']
states = data['states']
controls = data['controls']
```

## Complete Example

### Basic Visualization

```python
import numpy as np
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel
from src.controller import AttitudeController, AltitudeController, AirspeedController
from src.visualization import SimulationVisualizer

# Initialize
uav = FixedWingUAV()
aero = AerodynamicModel()
attitude_ctrl = AttitudeController()
altitude_ctrl = AltitudeController()
airspeed_ctrl = AirspeedController()
viz = SimulationVisualizer()

# Initial state
uav.set_state([0, 0, -100, 25, 0, 0, 0, 0, 0, 0, 0, 0])

# Simulation parameters
dt = 0.01
T_sim = 60.0
h_c = -150.0
Va_c = 25.0

# Simulation loop
time = 0.0
step = 0
while time < T_sim:
    # Control
    phi_c = 0.0
    theta_c = altitude_ctrl.compute_pitch_command(uav, h_c, dt)
    delta_t = airspeed_ctrl.compute_throttle_command(uav, Va_c, dt)
    delta_a, delta_e, delta_r = attitude_ctrl.compute_control(uav, phi_c, theta_c, dt)

    # Update
    control = np.array([delta_e, delta_a, delta_r, delta_t])
    uav.set_control(control)
    forces_moments = aero.compute_forces_moments(uav, control)
    uav.update(dt, forces_moments)

    # Record data every 0.1 seconds
    if step % 10 == 0:
        viz.add_data(time, uav.get_state(), control)

    time += dt
    step += 1

# Visualize results
print("Generating plots...")
viz.plot_3d_trajectory()
viz.plot_states()
viz.plot_controls()
viz.plot_airdata()

# Save data
viz.save_data('simulation_results.npz')
```

### Advanced Visualization with Waypoints

```python
from src.guidance import WaypointGuidance, CoordinatedTurnGuidance

# Define mission
waypoints = np.array([
    [0, 0, -100],
    [500, 0, -100],
    [500, 500, -150],
    [0, 500, -150],
    [0, 0, -100]
])

# Initialize guidance
waypoint_guidance = WaypointGuidance(waypoints, R_min=50.0)
turn_guidance = CoordinatedTurnGuidance(V_a=25.0)

# ... simulation loop ...

# Visualize with waypoints
viz.plot_3d_trajectory(waypoints=waypoints)
viz.plot_2d_trajectory(waypoints=waypoints)
viz.plot_states()
viz.plot_controls()
viz.plot_airdata()
```

### Orbit Visualization

```python
from src.guidance import OrbitGuidance

# Define orbit
orbit_center = np.array([500, 500, -150])
orbit_radius = 200.0
orbit_direction = 'CW'

orbit_guidance = OrbitGuidance(orbit_center, orbit_radius, orbit_direction)

# ... simulation loop ...

# Visualize orbit pattern
viz.plot_2d_trajectory(orbit_center=orbit_center, orbit_radius=orbit_radius)
viz.plot_3d_trajectory()
```

## Customization

### Accessing Raw Data

```python
# Access recorded data
times = np.array(viz.time_history)
states = np.array(viz.state_history)
controls = np.array(viz.control_history)

# Extract specific variables
north_pos = states[:, 0]
east_pos = states[:, 1]
altitude = -states[:, 2]  # Convert NED down to altitude
airspeed = np.sqrt(states[:, 3]**2 + states[:, 4]**2 + states[:, 5]**2)

# Custom plotting
import matplotlib.pyplot as plt

plt.figure()
plt.plot(times, altitude)
plt.xlabel('Time [s]')
plt.ylabel('Altitude [m]')
plt.title('Custom Altitude Plot')
plt.grid(True)
plt.show()
```

### Multiple Flight Comparison

```python
# Simulate multiple configurations
viz_stable = SimulationVisualizer()
viz_unstable = SimulationVisualizer()

# ... run simulations with different aircraft ...

# Compare trajectories
import matplotlib.pyplot as plt

fig = plt.figure(figsize=(12, 6))
ax = fig.add_subplot(111, projection='3d')

# Plot both trajectories
states1 = np.array(viz_stable.state_history)
states2 = np.array(viz_unstable.state_history)

ax.plot(states1[:, 0], states1[:, 1], -states1[:, 2], 'b-', label='Stable', linewidth=2)
ax.plot(states2[:, 0], states2[:, 1], -states2[:, 2], 'r-', label='Unstable', linewidth=2)

ax.set_xlabel('North [m]')
ax.set_ylabel('East [m]')
ax.set_zlabel('Altitude [m]')
ax.legend()
ax.set_title('Stability Comparison')
plt.show()
```

### Export for External Analysis

```python
# Save to CSV for Excel/MATLAB
times = np.array(viz.time_history)
states = np.array(viz.state_history)
controls = np.array(viz.control_history)

# Combine data
data = np.column_stack([times, states, controls])

# Save to CSV
header = 'time,pn,pe,pd,u,v,w,phi,theta,psi,p,q,r,delta_e,delta_a,delta_r,delta_t'
np.savetxt('flight_data.csv', data, delimiter=',', header=header, comments='')
```

## Plot Customization

All plotting methods return matplotlib figure and axes objects for further customization:

```python
# Customize 3D trajectory plot
fig, ax = viz.plot_3d_trajectory()
ax.view_init(elev=30, azim=45)  # Change viewing angle
fig.savefig('trajectory_3d.png', dpi=300)  # Save high-res image
```

## Performance Tips

### Memory Efficiency

For long simulations, record data less frequently:

```python
# Record every 1 second instead of every 0.1 seconds
if step % 100 == 0:  # With dt=0.01, this is every 1 second
    viz.add_data(time, uav.get_state(), control)
```

### Large Datasets

For very large datasets, consider saving data incrementally:

```python
# Save data in chunks
if time % 60.0 < dt:  # Every 60 seconds
    viz.save_data(f'flight_data_{int(time)}.npz')
    viz = SimulationVisualizer()  # Create new visualizer
```

## See Also

- [Dynamics Module](dynamics.md) - State vector definition
- [Controller Module](controller.md) - Control inputs
- [Guidance Module](guidance.md) - Waypoint and path following
- [Usage Guide](usage_guide.md) - Complete simulation examples
