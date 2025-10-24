# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a 6-DOF (degree of freedom) fixed-wing UAV simulator for validating control and guidance algorithms. The simulator uses the North-East-Down (NED) coordinate system and implements complete nonlinear equations of motion with aerodynamic coefficients based on stability and control derivatives.

**Primary Language**: Japanese (日本語) - Most documentation and comments are in Japanese.

## Essential Commands

### Running Simulations

```bash
# Basic flight demonstration
python examples/basic_flight.py

# Waypoint navigation
python examples/waypoint_nav.py

# Orbit/circular flight
python examples/orbit_flight.py

# Cascade control (current focus - 3-axis attitude control)
python examples/cascade_control.py

# Compare multiple aircraft models
python examples/compare_aircraft.py
```

### Testing

```bash
# Run all tests
python -m pytest

# Run specific test file
python -m pytest tests/test_dynamics.py

# Run basic smoke tests
python test_sim.py
python test_aircraft_models.py
python test_stability.py

# Run with verbose output
python -m pytest -v
```

### Dependencies

```bash
pip install numpy scipy matplotlib
```

## Architecture Overview

### Core Components

1. **dynamics.py** (`src/dynamics.py`)
   - Implements 6-DOF nonlinear dynamics using RK4 integration
   - State vector: `[x, y, z, u, v, w, phi, theta, psi, p, q, r]`
     - Position (NED): x, y, z
     - Velocity (body frame): u, v, w
     - Attitude (Euler angles): phi (roll), theta (pitch), psi (yaw)
     - Angular rates (body frame): p, q, r
   - Control inputs: `[delta_e, delta_a, delta_r, delta_t]` (elevator, aileron, rudder, throttle)
   - Default time step: 0.01s (100 Hz)

2. **aerodynamics.py** (`src/aerodynamics.py`)
   - Calculates aerodynamic forces and moments
   - Based on stability derivatives (C_L_alpha, C_m_alpha, etc.) and control derivatives (C_L_delta_e, etc.)
   - Implements propeller thrust model
   - Returns forces in body frame

3. **controller.py** (`src/controller.py`)
   - **PIDController**: Basic PID with integrator and differentiator
   - **AttitudeController**: Single-loop attitude control
   - **CascadeAttitudeController**: Two-loop cascade control structure
     - Outer loop: Angle control (angle error → target rate)
     - Inner loop: Rate control (rate error → control surface)
   - **AltitudeController**: Altitude control via pitch angle command
   - **SpeedController**: Airspeed control via throttle

4. **guidance.py** (`src/guidance.py`)
   - **WaypointGuidance**: Sequential waypoint following
   - **L1Guidance**: Vector field path following
   - **OrbitGuidance**: Circular orbit around a point
   - **ProportionalOrbitGuidance**: Advanced orbit with proportional navigation

5. **aircraft_generator.py** (`src/aircraft_generator.py`)
   - Generates custom aircraft with specified wingspan, mass, and stability characteristics
   - Stability levels per axis (roll/pitch/yaw): stable, neutral, slightly_unstable, unstable
   - Automatically calculates inertia and aerodynamic coefficients

6. **visualization.py** (`src/visualization.py`)
   - **SimulationVisualizer**: Plots 3D trajectory, state variables, control inputs, and air data
   - Generates comprehensive multi-panel plots

### Configuration

**config/aircraft_params.py** defines preset aircraft models:
- `micro`: 0.8m wingspan, 0.5kg (indoor)
- `small`: 1.6m wingspan, 1.7kg (default, target aircraft)
- `medium`: 2.9m wingspan, 11.0kg (Rascal-class)
- `large`: 4.0m wingspan, 20.0kg (long endurance)

Each model has stability variants (for small aircraft):
- `small`: Stable (default)
- `small_slightly_unstable`: Moderately challenging
- `small_unstable`: Highly challenging

## Critical Implementation Details

### Coordinate Systems

- **NED (North-East-Down)**: Inertial frame
  - X: North, Y: East, Z: Down (altitude is negative)
- **Body Frame**: Aircraft-fixed frame
  - X: Forward (nose), Y: Right (right wing), Z: Down

### Trim Conditions

For stable level flight at 15 m/s (typical for small UAV):
- Angle of attack (alpha): ~1.9 degrees
- Pitch angle (theta): ~5.0 degrees
- Elevator trim: Calculated from `-(C_m_0 + C_m_alpha * alpha) / C_m_delta_e`
- Throttle trim: ~0.305 (empirically determined)

**Important**: When initializing simulations, set proper trim conditions to avoid initial transients. See `examples/cascade_control.py` for reference implementation.

### Controller Tuning Philosophy

The codebase uses **cascade control** for attitude:
1. Outer loop controls angles (slower, provides setpoint for inner loop)
2. Inner loop controls rates (faster, directly commands control surfaces)

This structure:
- Separates concerns between angle tracking and rate damping
- Provides better disturbance rejection
- Allows independent tuning of each loop

### Simulation Loop Pattern

Standard pattern used across examples:

```python
# 1. Initialize
uav = FixedWingUAV(aircraft_type='small')
aero = AerodynamicModel(aircraft_type='small')
controller = CascadeAttitudeController()

# 2. Set trim condition
uav.state[3] = u_trim  # Forward velocity
uav.state[5] = w_trim  # Downward velocity
uav.state[7] = theta_trim  # Pitch angle
uav.control[0] = elevator_trim
uav.control[3] = throttle_trim

# 3. Simulation loop
dt = 0.01
for i in range(num_steps):
    # Get state
    state = uav.state

    # Calculate control (from controller)
    delta_e, delta_a, delta_r = controller.update(...)

    # Update control vector
    uav.control = [delta_e, delta_a, delta_r, throttle]

    # Calculate forces/moments
    forces, moments = aero.calculate_forces_and_moments(state, uav.control)

    # Integrate dynamics
    state = uav.runge_kutta_step(state, uav.control, forces, moments, dt)
    uav.state = state
```

## Common Development Tasks

### Adding a New Controller

1. Implement controller class in `src/controller.py`
2. Follow the pattern of existing controllers (PIDController, CascadeAttitudeController)
3. Add an example in `examples/` demonstrating the controller
4. Add unit tests in `tests/test_controller.py`

### Creating a Custom Aircraft

Use `aircraft_generator.py`:

```python
from src.aircraft_generator import create_aircraft

aircraft_params, aero_params = create_aircraft(
    wingspan=2.0,
    mass=3.0,
    roll_stability='stable',
    pitch_stability='slightly_unstable',
    yaw_stability='stable'
)

uav = FixedWingUAV(params=aircraft_params)
aero = AerodynamicModel(params=aero_params)
```

### Testing a Control Algorithm

1. Start with simple test in `examples/` directory
2. Use `SimulationVisualizer` to plot results
3. Check for:
   - Settling time (typically < 3s for attitude)
   - Overshoot (< 10% desired)
   - Steady-state error
   - Control saturation

## Known Issues and Gotchas

### Sign Conventions

- **Altitude**: Negative in NED (z = -100 means 100m altitude)
- **Elevator**: Negative deflection typically produces nose-down moment
- **Pitch control**: There may be sign inconsistencies in pitch control - check if pitch response is inverted

Recent commit mentions: "ピッチの舵角の正負が逆か？" (Is the sign of pitch control surface reversed?)

### Trim Finding

- Trim conditions are empirically determined (see `examples/trim_search.py`)
- Small changes in throttle (~0.01) can significantly affect altitude rate
- Always verify trim in level flight before testing controllers

### Control Limits

Standard limits used:
- Elevator/Aileron/Rudder: ±0.5 rad (~±28.6 degrees)
- Throttle: 0.0 to 1.0
- Pitch angle command: ±45 degrees (in some controllers)

## Testing Strategy

- **Unit tests** (`tests/`): Test individual components in isolation
- **Integration tests** (`tests/test_integration.py`): Test component interactions
- **Example scripts** (`examples/`): Serve as both demos and integration tests
- **Basic smoke tests**: `test_sim.py`, `test_aircraft_models.py`, `test_stability.py`

Run tests before committing control law changes to ensure dynamics remain stable.

## Reference Materials

Key documentation in `docs/`:
- `controller.md`: Control system details
- `dynamics.md`: Equations of motion
- `aerodynamics.md`: Aerodynamic model
- `guidance.md`: Guidance algorithms
- `l1_guidance_theory.md/ja`: L1 guidance theory (English/Japanese)
- `orbit_guidance_theory.md/ja`: Orbit guidance theory (English/Japanese)

The simulator is based on:
- Beard, R. W., & McLain, T. W. (2012). Small Unmanned Aircraft: Theory and Practice
- Stevens, B. L., & Lewis, F. L. (2003). Aircraft Control and Simulation
