# Fixed-Wing UAV Simulator - Usage Guide

## Table of Contents

1. [Getting Started](#getting-started)
2. [Quick Start Example](#quick-start-example)
3. [Basic Simulations](#basic-simulations)
4. [Advanced Features](#advanced-features)
5. [Creating Custom Aircraft](#creating-custom-aircraft)
6. [Troubleshooting](#troubleshooting)

## Getting Started

### Installation

```bash
# Clone the repository
git clone https://github.com/your-username/uav_simulator.git
cd uav_simulator

# Install dependencies
pip install -r requirements.txt
```

### Project Structure

```
uav_simulator/
├── src/                    # Core simulation modules
│   ├── dynamics.py         # 6-DOF dynamics model
│   ├── aerodynamics.py     # Aerodynamic forces and moments
│   ├── controller.py       # Control laws (attitude, altitude, airspeed)
│   ├── guidance.py         # Path following and navigation
│   ├── visualization.py    # Plotting and data visualization
│   └── aircraft_generator.py  # Custom aircraft creation
├── config/                 # Configuration files
│   └── aircraft_params.py  # Predefined aircraft parameters
├── examples/               # Example simulations
│   ├── basic_flight.py     # Simple altitude/airspeed control
│   ├── waypoint_nav.py     # Waypoint navigation
│   ├── orbit_flight.py     # Circular orbit pattern
│   ├── small_uav_demo.py   # Small UAV demonstration
│   └── compare_aircraft.py # Compare different aircraft types
├── docs/                   # Documentation
└── README.md              # Project overview
```

## Quick Start Example

### Run a Basic Simulation

```bash
# Run the basic flight example
python examples/basic_flight.py
```

This will:
1. Initialize a small UAV (1.6m wingspan, 1.7kg)
2. Command it to climb from 100m to 150m altitude
3. Maintain 25 m/s airspeed
4. Display 3D trajectory and time series plots

### Understanding the Output

You'll see plots showing:
- **3D Trajectory**: Flight path in 3D space
- **States**: Position, velocity, attitude, angular rates over time
- **Controls**: Elevator, aileron, rudder, throttle commands
- **Air Data**: Airspeed, angle of attack, sideslip angle

## Basic Simulations

### 1. Level Flight

```python
import numpy as np
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel
from src.controller import AttitudeController, AltitudeController, AirspeedController
from src.visualization import SimulationVisualizer

# Initialize aircraft and controllers
uav = FixedWingUAV()  # Default small UAV
aero = AerodynamicModel()
attitude_ctrl = AttitudeController()
altitude_ctrl = AltitudeController()
airspeed_ctrl = AirspeedController()
viz = SimulationVisualizer()

# Set initial state: [pn, pe, pd, u, v, w, phi, theta, psi, p, q, r]
# Starting at 100m altitude, 20 m/s airspeed, level attitude
uav.set_state([0, 0, -100, 20, 0, 0, 0, 0, 0, 0, 0, 0])

# Simulation parameters
dt = 0.01  # 10ms time step
T_sim = 60.0  # 60 seconds

# Target values
h_c = -100.0  # Maintain 100m altitude (negative in NED)
Va_c = 20.0   # Maintain 20 m/s airspeed
phi_c = 0.0   # Keep wings level

# Main simulation loop
time = 0.0
step = 0
while time < T_sim:
    # Compute control commands
    theta_c = altitude_ctrl.compute_pitch_command(uav, h_c, dt)
    delta_t = airspeed_ctrl.compute_throttle_command(uav, Va_c, dt)
    delta_a, delta_e, delta_r = attitude_ctrl.compute_control(uav, phi_c, theta_c, dt)

    # Set control inputs
    control = np.array([delta_e, delta_a, delta_r, delta_t])
    uav.set_control(control)

    # Calculate forces and update dynamics
    forces_moments = aero.compute_forces_moments(uav, control)
    uav.update(dt, forces_moments)

    # Record data every 0.1 seconds
    if step % 10 == 0:
        viz.add_data(time, uav.get_state(), control)

    time += dt
    step += 1

# Visualize results
viz.plot_3d_trajectory()
viz.plot_states()
viz.plot_controls()
```

### 2. Waypoint Navigation

```python
from src.guidance import WaypointGuidance, CoordinatedTurnGuidance

# Define waypoints (North, East, Down) in meters
waypoints = np.array([
    [0, 0, -100],       # Start
    [500, 0, -100],     # Fly north
    [500, 500, -150],   # Turn east and climb
    [0, 500, -150],     # Turn west
    [0, 0, -100]        # Return home
])

# Initialize guidance
waypoint_guidance = WaypointGuidance(waypoints, R_min=50.0)
turn_guidance = CoordinatedTurnGuidance(V_a=20.0)

# Initialize aircraft and controllers
uav = FixedWingUAV()
aero = AerodynamicModel()
attitude_ctrl = AttitudeController()
altitude_ctrl = AltitudeController()
airspeed_ctrl = AirspeedController()
viz = SimulationVisualizer()

# Initial state at first waypoint
uav.set_state([0, 0, -100, 20, 0, 0, 0, 0, 0, 0, 0, 0])

# Simulation parameters
dt = 0.01
Va_c = 20.0

# Mission execution loop
time = 0.0
step = 0
while not waypoint_guidance.completed and time < 300.0:
    # Get current state
    position = uav.get_position()
    phi, theta, psi = uav.get_attitude()

    # Guidance: where to go
    waypoint, completed = waypoint_guidance.update(position)
    psi_c = waypoint_guidance.compute_heading_command(position)
    h_c = waypoint[2]

    # Convert heading command to roll command
    phi_c = turn_guidance.compute_roll_command(psi, psi_c, k_psi=0.5)

    # Control: how to get there
    theta_c = altitude_ctrl.compute_pitch_command(uav, h_c, dt)
    delta_t = airspeed_ctrl.compute_throttle_command(uav, Va_c, dt)
    delta_a, delta_e, delta_r = attitude_ctrl.compute_control(uav, phi_c, theta_c, dt)

    # Apply control and update
    control = np.array([delta_e, delta_a, delta_r, delta_t])
    uav.set_control(control)
    forces_moments = aero.compute_forces_moments(uav, control)
    uav.update(dt, forces_moments)

    # Record data
    if step % 10 == 0:
        viz.add_data(time, uav.get_state(), control)

    # Progress update
    if step % 1000 == 0:
        wp_idx = waypoint_guidance.current_waypoint_index
        print(f"Time: {time:.1f}s, Waypoint: {wp_idx}, Alt: {-position[2]:.1f}m")

    time += dt
    step += 1

# Visualize with waypoints
viz.plot_3d_trajectory(waypoints=waypoints)
viz.plot_2d_trajectory(waypoints=waypoints)
viz.plot_states()
```

### 3. Circular Orbit

```python
from src.guidance import OrbitGuidance

# Define orbit parameters
orbit_center = np.array([500, 500, -150])  # Center (N, E, D)
orbit_radius = 200.0  # Radius in meters
orbit_direction = 'CW'  # Clockwise

# Initialize orbit guidance
orbit_guidance = OrbitGuidance(orbit_center, orbit_radius, orbit_direction)
turn_guidance = CoordinatedTurnGuidance(V_a=25.0)

# Initialize aircraft
uav = FixedWingUAV()
aero = AerodynamicModel()
# ... (controllers as before)

# Start outside orbit circle
initial_pos = orbit_center + np.array([orbit_radius + 100, 0, 0])
uav.set_state([initial_pos[0], initial_pos[1], initial_pos[2],
               25, 0, 0, 0, 0, 0, 0, 0, 0])

# Fly orbit pattern
dt = 0.01
T_orbit = 120.0  # 2 minutes
time = 0.0
step = 0

while time < T_orbit:
    position = uav.get_position()
    phi, theta, psi = uav.get_attitude()

    # Orbit guidance
    psi_c = orbit_guidance.compute_heading_command(position, k_orbit=2.0)
    h_c = orbit_center[2]
    phi_c = turn_guidance.compute_roll_command(psi, psi_c, k_psi=0.8)

    # Control and update (as before)
    theta_c = altitude_ctrl.compute_pitch_command(uav, h_c, dt)
    delta_t = airspeed_ctrl.compute_throttle_command(uav, 25.0, dt)
    delta_a, delta_e, delta_r = attitude_ctrl.compute_control(uav, phi_c, theta_c, dt)

    control = np.array([delta_e, delta_a, delta_r, delta_t])
    uav.set_control(control)
    forces_moments = aero.compute_forces_moments(uav, control)
    uav.update(dt, forces_moments)

    if step % 10 == 0:
        viz.add_data(time, uav.get_state(), control)

    time += dt
    step += 1

# Visualize orbit
viz.plot_2d_trajectory(orbit_center=orbit_center, orbit_radius=orbit_radius)
viz.plot_3d_trajectory()
```

## Advanced Features

### Using Different Aircraft Types

```python
# Micro UAV (1.0m, 0.8kg)
uav = FixedWingUAV(aircraft_type='micro')
aero = AerodynamicModel(aircraft_type='micro')

# Small UAV (1.6m, 1.7kg) - default
uav = FixedWingUAV(aircraft_type='small')
aero = AerodynamicModel(aircraft_type='small')

# Medium UAV (2.9m, 11kg)
uav = FixedWingUAV(aircraft_type='medium')
aero = AerodynamicModel(aircraft_type='medium')

# Large UAV (4.0m, 25kg)
uav = FixedWingUAV(aircraft_type='large')
aero = AerodynamicModel(aircraft_type='large')
```

### Stability Variants

```python
# Stable configuration (easy to fly)
uav = FixedWingUAV(aircraft_type='small')
aero = AerodynamicModel(aircraft_type='small')

# Slightly unstable (challenging)
uav = FixedWingUAV(aircraft_type='small_slightly_unstable')
aero = AerodynamicModel(aircraft_type='small_slightly_unstable')

# Unstable (requires aggressive control)
uav = FixedWingUAV(aircraft_type='small_unstable')
aero = AerodynamicModel(aircraft_type='small_unstable')
```

### Tuning Controllers

```python
# Tune attitude controller for more aggressive response
attitude_ctrl.kp_phi = 1.2    # Increase roll response
attitude_ctrl.kd_phi = 0.15   # Increase roll damping

attitude_ctrl.kp_theta = 1.5  # Increase pitch response
attitude_ctrl.kd_theta = 0.25 # Increase pitch damping

# Tune altitude controller
altitude_ctrl.kp_h = 0.015    # Faster altitude response
altitude_ctrl.ki_h = 0.002    # Eliminate steady-state error
altitude_ctrl.kd_h = 0.03     # Reduce overshoot

# Tune airspeed controller
airspeed_ctrl.kp_Va = 0.15    # Faster speed response
airspeed_ctrl.ki_Va = 0.02    # Eliminate speed error
```

## Creating Custom Aircraft

### Using Aircraft Generator

```python
from src.aircraft_generator import AircraftGenerator

# Create generator
generator = AircraftGenerator()

# Design custom aircraft
params = generator.generate_aircraft(
    wingspan=2.0,           # 2 meter wingspan
    mass=2.5,               # 2.5 kg mass
    roll_stability='stable',
    pitch_stability='stable',
    yaw_stability='stable',
    aspect_ratio=10.0,      # High aspect ratio for efficiency
    name='my_glider'
)

# Use custom aircraft
uav = FixedWingUAV(params=params)
aero = AerodynamicModel(params=params)

# Print aircraft info
print(f"Wing area: {params['S_wing']:.3f} m²")
print(f"Wing loading: {params['mass']*9.81/params['S_wing']:.1f} N/m²")
```

### Stability Experimentation

```python
# Create three variants of same aircraft
variants = {
    'stable': generator.generate_aircraft(
        1.6, 1.7, 'stable', 'stable', 'stable', name='stable'
    ),
    'neutral': generator.generate_aircraft(
        1.6, 1.7, 'neutral', 'neutral', 'neutral', name='neutral'
    ),
    'unstable': generator.generate_aircraft(
        1.6, 1.7, 'unstable', 'unstable', 'unstable', name='unstable'
    )
}

# Simulate each and compare
for name, params in variants.items():
    print(f"\nSimulating {name} configuration...")
    uav = FixedWingUAV(params=params)
    aero = AerodynamicModel(params=params)
    # ... run simulation ...
```

## Data Analysis and Export

### Saving Simulation Data

```python
# Save to NumPy archive
viz.save_data('flight_data.npz')

# Load later
data = np.load('flight_data.npz')
times = data['time']
states = data['states']
controls = data['controls']
```

### Export to CSV

```python
# Get data
times = np.array(viz.time_history)
states = np.array(viz.state_history)
controls = np.array(viz.control_history)

# Combine and save
data = np.column_stack([times, states, controls])
header = 'time,pn,pe,pd,u,v,w,phi,theta,psi,p,q,r,delta_e,delta_a,delta_r,delta_t'
np.savetxt('flight_data.csv', data, delimiter=',', header=header, comments='')
```

### Custom Analysis

```python
# Extract specific data
times = np.array(viz.time_history)
states = np.array(viz.state_history)

# Calculate derived quantities
altitude = -states[:, 2]  # Convert NED down to altitude
airspeed = np.sqrt(states[:, 3]**2 + states[:, 4]**2 + states[:, 5]**2)
roll = np.rad2deg(states[:, 6])
pitch = np.rad2deg(states[:, 7])
yaw = np.rad2deg(states[:, 8])

# Custom plotting
import matplotlib.pyplot as plt

fig, axes = plt.subplots(2, 1, figsize=(10, 8))

axes[0].plot(times, altitude)
axes[0].set_ylabel('Altitude [m]')
axes[0].grid(True)

axes[1].plot(times, airspeed)
axes[1].set_ylabel('Airspeed [m/s]')
axes[1].set_xlabel('Time [s]')
axes[1].grid(True)

plt.tight_layout()
plt.show()
```

## Troubleshooting

### Aircraft Diverges Immediately

**Problem**: Aircraft goes unstable right after start

**Solutions**:
1. Check initial velocity is sufficient (> 15 m/s)
2. Use stable aircraft configuration
3. Verify initial attitude is level (phi, theta near 0)
4. Check control gains aren't too aggressive

```python
# Good initial state
uav.set_state([0, 0, -100, 20, 0, 0, 0, 0, 0, 0, 0, 0])

# Bad initial state (too slow)
uav.set_state([0, 0, -100, 5, 0, 0, 0, 0, 0, 0, 0, 0])
```

### Controller Oscillations

**Problem**: Aircraft oscillates around target

**Solutions**:
1. Reduce proportional gains
2. Increase derivative gains
3. Add integral term for steady-state error
4. Reduce time step if oscillations are high-frequency

```python
# Reduce oscillations
attitude_ctrl.kp_phi = 0.5   # Reduce from 0.8
attitude_ctrl.kd_phi = 0.2   # Increase from 0.1
```

### Waypoint Not Reached

**Problem**: Aircraft circles waypoint but doesn't switch

**Solutions**:
1. Increase capture radius `R_min`
2. Check altitude is achievable
3. Verify airspeed is appropriate for turns

```python
# Increase capture radius
waypoint_guidance = WaypointGuidance(waypoints, R_min=80.0)  # Increase from 50
```

### Unrealistic Flight

**Problem**: Aircraft performance seems wrong

**Solutions**:
1. Verify aircraft parameters are reasonable
2. Check mass and wingspan relationship
3. Ensure airspeed is within valid range (15-40 m/s)
4. Verify control surface limits

```python
# Check parameters
print(f"Wing loading: {uav.params['mass']*9.81/uav.params['S_wing']:.1f} N/m²")
print(f"Aspect ratio: {uav.params['b']**2/uav.params['S_wing']:.1f}")
```

## Additional Resources

- [Dynamics Module Documentation](dynamics.md)
- [Aerodynamics Module Documentation](aerodynamics.md)
- [Controller Module Documentation](controller.md)
- [Guidance Module Documentation](guidance.md)
- [Visualization Module Documentation](visualization.md)
- [Aircraft Generator Documentation](aircraft_generator.md)
- [Requirements Document](requirements.md)

## Example Scripts

All example scripts are in the `examples/` directory:

- `basic_flight.py` - Simple altitude and airspeed control
- `waypoint_nav.py` - Sequential waypoint following
- `orbit_flight.py` - Circular orbit pattern
- `small_uav_demo.py` - Small UAV demonstration
- `compare_aircraft.py` - Compare different aircraft types

Run any example:
```bash
python examples/basic_flight.py
python examples/waypoint_nav.py
python examples/orbit_flight.py
```

## Next Steps

1. **Start with examples**: Run the provided examples to understand the simulator
2. **Modify parameters**: Experiment with different aircraft and control gains
3. **Create missions**: Design your own waypoint patterns
4. **Custom aircraft**: Use the aircraft generator to create unique designs
5. **Advanced control**: Implement new guidance laws or control algorithms

Happy simulating!
