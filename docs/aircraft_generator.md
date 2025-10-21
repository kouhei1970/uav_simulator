# Aircraft Generator Module

## Overview

The `aircraft_generator.py` module provides tools for creating custom aircraft configurations based on physical dimensions and desired stability characteristics. It automatically calculates aerodynamic and inertia parameters from wingspan and mass, with independent control over roll, pitch, and yaw stability.

## Class: AircraftGenerator

### Description

The `AircraftGenerator` class generates complete aircraft parameter sets for use with the UAV simulator, enabling rapid prototyping of different aircraft designs.

### Initialization

```python
from src.aircraft_generator import AircraftGenerator

# Create generator instance
generator = AircraftGenerator()
```

## Key Methods

### generate_aircraft()

Generates a complete aircraft parameter dictionary.

**Parameters:**
- `wingspan`: Aircraft wingspan [m]
- `mass`: Aircraft mass [kg]
- `roll_stability`: Roll stability level - `'stable'`, `'neutral'`, `'slightly_unstable'`, or `'unstable'`
- `pitch_stability`: Pitch stability level - `'stable'`, `'neutral'`, `'slightly_unstable'`, or `'unstable'`
- `yaw_stability`: Yaw stability level - `'stable'`, `'neutral'`, `'slightly_unstable'`, or `'unstable'`
- `aspect_ratio`: Wing aspect ratio (default: 9.0)
- `name`: Aircraft name (default: "custom")

**Returns:**
- Dictionary containing complete aircraft parameters compatible with `FixedWingUAV` and `AerodynamicModel`

```python
# Generate stable small aircraft
params = generator.generate_aircraft(
    wingspan=1.6,
    mass=1.7,
    roll_stability='stable',
    pitch_stability='stable',
    yaw_stability='stable',
    aspect_ratio=9.0,
    name='my_uav'
)
```

## Stability Levels

### Roll Stability (C_l_beta, C_l_p)

**Stable:**
- C_l_beta = -0.15 (strong dihedral effect)
- C_l_p = -0.5 (strong roll damping)
- Behavior: Self-levels, resists bank angles

**Neutral:**
- C_l_beta = 0.0 (no dihedral effect)
- C_l_p = -0.3 (moderate damping)
- Behavior: Maintains roll angle, no self-leveling

**Slightly Unstable:**
- C_l_beta = 0.03 (weak reverse dihedral)
- C_l_p = -0.2 (weak damping)
- Behavior: Slowly diverges from level flight

**Unstable:**
- C_l_beta = 0.10 (strong reverse dihedral)
- C_l_p = -0.1 (very weak damping)
- Behavior: Rapidly diverges, requires active control

### Pitch Stability (C_m_alpha, C_m_q)

**Stable:**
- C_m_alpha = -0.50 (strong pitch stability)
- C_m_q = -3.5 (strong pitch damping)
- Behavior: Returns to trim angle of attack

**Neutral:**
- C_m_alpha = 0.0 (neutrally stable)
- C_m_q = -2.0 (moderate damping)
- Behavior: Maintains current pitch attitude

**Slightly Unstable:**
- C_m_alpha = 0.05 (weakly unstable)
- C_m_q = -1.5 (weak damping)
- Behavior: Slowly diverges in pitch

**Unstable:**
- C_m_alpha = 0.15 (strongly unstable)
- C_m_q = -1.0 (minimal damping)
- Behavior: Rapidly diverges in pitch

### Yaw Stability (C_n_beta, C_n_r)

**Stable:**
- C_n_beta = 0.30 (strong weathercock stability)
- C_n_r = -0.3 (strong yaw damping)
- Behavior: Aligns with relative wind

**Neutral:**
- C_n_beta = 0.0 (no weathercock effect)
- C_n_r = -0.15 (moderate damping)
- Behavior: Maintains current heading

**Slightly Unstable:**
- C_n_beta = 0.05 (weak stability)
- C_n_r = -0.1 (weak damping)
- Behavior: Slow heading divergence

**Unstable:**
- C_n_beta = -0.15 (adverse yaw)
- C_n_r = -0.05 (minimal damping)
- Behavior: Rapid heading divergence

## Parameter Calculations

### Geometric Parameters

**Wing Area:**
```
S_wing = wingspan² / aspect_ratio
```

**Mean Aerodynamic Chord:**
```
c = wingspan / aspect_ratio
```

### Inertia Parameters

**Roll Inertia (Jx):**
```
Jx = 0.15 × mass × wingspan²
```

**Pitch Inertia (Jy):**
```
Jy = 0.25 × mass × wingspan²
```

**Yaw Inertia (Jz):**
```
Jz = 0.35 × mass × wingspan²
```

**Product of Inertia (Jxz):**
```
Jxz = 0.03 × mass × wingspan²
```

### Aerodynamic Coefficients

Base coefficients are scaled according to aircraft size and Reynolds number effects:

```
Re = ρ × V × c / μ
```

Control effectiveness is adjusted based on wing area and dynamic pressure.

## Complete Examples

### Basic Aircraft Generation

```python
from src.aircraft_generator import AircraftGenerator
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel

# Create generator
generator = AircraftGenerator()

# Generate small stable UAV
params = generator.generate_aircraft(
    wingspan=1.6,
    mass=1.7,
    roll_stability='stable',
    pitch_stability='stable',
    yaw_stability='stable',
    name='small_stable_uav'
)

# Use with simulator
uav = FixedWingUAV(params=params)
aero = AerodynamicModel(params=params)

# Initialize and fly
uav.set_state([0, 0, -100, 20, 0, 0, 0, 0, 0, 0, 0, 0])
```

### Unstable Research Aircraft

```python
# Generate unstable aircraft for control research
params = generator.generate_aircraft(
    wingspan=2.0,
    mass=3.0,
    roll_stability='unstable',
    pitch_stability='slightly_unstable',
    yaw_stability='neutral',
    aspect_ratio=8.0,
    name='research_uav'
)

# This aircraft will require aggressive control to maintain stable flight
uav = FixedWingUAV(params=params)
aero = AerodynamicModel(params=params)
```

### High Aspect Ratio Glider

```python
# Generate efficient glider
params = generator.generate_aircraft(
    wingspan=3.0,
    mass=2.5,
    roll_stability='stable',
    pitch_stability='stable',
    yaw_stability='stable',
    aspect_ratio=15.0,  # High aspect ratio for efficiency
    name='glider'
)

uav = FixedWingUAV(params=params)
aero = AerodynamicModel(params=params)
```

### Agile Aerobatic Aircraft

```python
# Generate aerobatic aircraft
params = generator.generate_aircraft(
    wingspan=1.2,
    mass=1.0,
    roll_stability='neutral',
    pitch_stability='neutral',
    yaw_stability='neutral',
    aspect_ratio=6.0,  # Lower aspect ratio for maneuverability
    name='aerobatic'
)

uav = FixedWingUAV(params=params)
aero = AerodynamicModel(params=params)
```

## Saving Custom Aircraft

### Save to Configuration File

```python
import json

# Generate aircraft
params = generator.generate_aircraft(
    wingspan=1.6,
    mass=1.7,
    roll_stability='stable',
    pitch_stability='stable',
    yaw_stability='stable',
    name='my_custom_uav'
)

# Save to JSON file
with open('my_custom_uav.json', 'w') as f:
    # Convert numpy arrays to lists for JSON serialization
    params_serializable = {k: (v.tolist() if hasattr(v, 'tolist') else v)
                          for k, v in params.items()}
    json.dump(params_serializable, f, indent=2)

# Load later
with open('my_custom_uav.json', 'r') as f:
    loaded_params = json.load(f)
    uav = FixedWingUAV(params=loaded_params)
```

### Add to aircraft_params.py

```python
# Add to config/aircraft_params.py
from src.aircraft_generator import AircraftGenerator

generator = AircraftGenerator()
my_aircraft = generator.generate_aircraft(
    wingspan=1.8,
    mass=2.0,
    roll_stability='stable',
    pitch_stability='stable',
    yaw_stability='stable',
    name='my_aircraft'
)

# In AIRCRAFT_TYPES dictionary:
AIRCRAFT_TYPES = {
    # ... existing types ...
    'my_aircraft': my_aircraft,
}
```

## Comparative Analysis

### Generate Multiple Variants

```python
from src.aircraft_generator import AircraftGenerator
from src.dynamics import FixedWingUAV
from src.aerodynamics import AerodynamicModel
from src.visualization import SimulationVisualizer

generator = AircraftGenerator()

# Generate stability variants
variants = {
    'stable': generator.generate_aircraft(1.6, 1.7, 'stable', 'stable', 'stable'),
    'neutral': generator.generate_aircraft(1.6, 1.7, 'neutral', 'neutral', 'neutral'),
    'unstable': generator.generate_aircraft(1.6, 1.7, 'unstable', 'unstable', 'unstable')
}

# Simulate each variant
results = {}
for name, params in variants.items():
    uav = FixedWingUAV(params=params)
    aero = AerodynamicModel(params=params)
    viz = SimulationVisualizer()

    # ... run simulation ...

    results[name] = viz

# Compare results
import matplotlib.pyplot as plt

fig = plt.figure(figsize=(15, 5))
for i, (name, viz) in enumerate(results.items()):
    ax = fig.add_subplot(1, 3, i+1, projection='3d')
    states = np.array(viz.state_history)
    ax.plot(states[:, 0], states[:, 1], -states[:, 2])
    ax.set_title(f'{name.capitalize()} Configuration')
    ax.set_xlabel('North [m]')
    ax.set_ylabel('East [m]')
    ax.set_zlabel('Altitude [m]')
plt.tight_layout()
plt.show()
```

## Design Guidelines

### Choosing Wingspan and Mass

**Scale Relationship:**
- Micro (0.8-1.2m): 0.5-1.5 kg
- Small (1.5-2.0m): 1.5-3.0 kg
- Medium (2.5-3.5m): 5.0-15.0 kg
- Large (3.5-5.0m): 15.0-30.0 kg

**Wing Loading:**
```
W/S = mass × g / S_wing
```
- Low (< 30 N/m²): Good for slow flight, training
- Medium (30-60 N/m²): Balanced performance
- High (> 60 N/m²): Fast flight, challenging handling

### Choosing Aspect Ratio

- **High AR (12-20)**: Gliders, long endurance
  - Better lift/drag ratio
  - Slower roll response
  - More efficient

- **Medium AR (8-12)**: General aviation, trainers
  - Balanced performance
  - Moderate agility

- **Low AR (5-8)**: Aerobatic, racing
  - Higher drag
  - Better roll rate
  - More maneuverable

### Choosing Stability

**For Training/Autonomous Flight:**
- Roll: Stable
- Pitch: Stable
- Yaw: Stable
- Easy to fly, forgiving

**For Manual Control:**
- Roll: Neutral or Slightly Unstable
- Pitch: Stable
- Yaw: Neutral
- Responsive, fun to fly

**For Control Research:**
- Roll: Adjustable
- Pitch: Adjustable
- Yaw: Adjustable
- Tests control algorithms

## Validation

### Check Generated Parameters

```python
params = generator.generate_aircraft(1.6, 1.7, 'stable', 'stable', 'stable')

# Verify key parameters
print(f"Wing area: {params['S_wing']:.3f} m²")
print(f"Chord: {params['c']:.3f} m")
print(f"Aspect ratio: {params['b']**2 / params['S_wing']:.2f}")
print(f"Wing loading: {params['mass'] * 9.81 / params['S_wing']:.1f} N/m²")

# Check stability derivatives
print(f"\nStability:")
print(f"  Roll (C_l_beta): {params['C_l_beta']:.3f}")
print(f"  Pitch (C_m_alpha): {params['C_m_alpha']:.3f}")
print(f"  Yaw (C_n_beta): {params['C_n_beta']:.3f}")
```

### Print Aircraft Info

```python
from config.aircraft_params import print_aircraft_info

# If saved to aircraft_params
print_aircraft_info('my_aircraft')
```

## See Also

- [Dynamics Module](dynamics.md) - Using generated parameters
- [Aerodynamics Module](aerodynamics.md) - Aerodynamic coefficients
- [Usage Guide](usage_guide.md) - Complete examples
- [Requirements](requirements.md) - Design specifications
