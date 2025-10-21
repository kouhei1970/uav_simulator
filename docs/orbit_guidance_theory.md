# Orbit Guidance Theory and Implementation

## Table of Contents

1. [Overview](#overview)
2. [Problem Formulation](#problem-formulation)
3. [Guidance Law Derivation](#guidance-law-derivation)
4. [Lyapunov Stability Analysis](#lyapunov-stability-analysis)
5. [Implementation Details](#implementation-details)
6. [GPS Measurement Integration](#gps-measurement-integration)
7. [Performance Analysis](#performance-analysis)
8. [Practical Considerations](#practical-considerations)

## Overview

### Purpose

The orbit guidance algorithm enables a fixed-wing UAV to maintain a circular flight path around a specified center point at a desired radius. This is a fundamental capability for:

- **Persistent surveillance** of a target area
- **Communication relay** over a specific location
- **Loitering** while awaiting further commands
- **Sensor data collection** around an area of interest

### Key Challenges

Fixed-wing aircraft cannot hover, so maintaining a circular orbit requires:

1. **Continuous forward motion** at sufficient airspeed
2. **Coordinated turns** to change heading
3. **Feedback control** to correct deviations from desired orbit
4. **Robustness** to wind, measurement errors, and disturbances

## Problem Formulation

### Coordinate System

We use the **NED (North-East-Down)** coordinate system:

- **North (N)**: Positive direction points north
- **East (E)**: Positive direction points east
- **Down (D)**: Positive direction points down (altitude is negative)

### Variables

**Given:**
- $\mathbf{c} = [c_n, c_e, c_d]^T$: Orbit center position in NED frame
- $R$: Desired orbit radius
- $\lambda$: Orbit direction (+1 for CCW, -1 for CW)

**State:**
- $\mathbf{p} = [p_n, p_e, p_d]^T$: Aircraft position in NED frame
- $\psi$: Aircraft heading angle (yaw)
- $V_a$: Airspeed

**Control:**
- $\psi_c$: Commanded heading angle

### Objective

Design a guidance law that computes $\psi_c(t)$ such that:

$$\lim_{t \to \infty} \|\mathbf{p}(t) - \mathbf{c}\| = R$$

while maintaining:
- Continuous heading commands (no discontinuities)
- Convergence from any initial position
- Robustness to measurement noise

## Guidance Law Derivation

### Vector Field Approach

The orbit guidance law is based on a **vector field** that defines the desired velocity direction at each point in space.

#### Step 1: Define Position Error

Let $\mathbf{e} = \mathbf{p} - \mathbf{c}$ be the position error vector from orbit center to aircraft.

In 2D (ignoring altitude for guidance):

$$\mathbf{e} = \begin{bmatrix} e_n \\ e_e \end{bmatrix} = \begin{bmatrix} p_n - c_n \\ p_e - c_e \end{bmatrix}$$

#### Step 2: Calculate Distance and Radial Error

Current distance from orbit center:

$$d = \|\mathbf{e}\| = \sqrt{e_n^2 + e_e^2}$$

Radial error (positive when outside orbit):

$$e_r = d - R$$

#### Step 3: Define Desired Velocity Direction

For a perfect orbit, the velocity should be **tangent** to the circle. The tangent direction is perpendicular to the radial direction.

Unit radial vector (pointing from center to aircraft):

$$\hat{\mathbf{r}} = \frac{\mathbf{e}}{d} = \begin{bmatrix} e_n/d \\ e_e/d \end{bmatrix}$$

Unit tangent vector (perpendicular to radial, in orbit direction):

For **clockwise (CW)** orbit ($\lambda = -1$):

$$\hat{\mathbf{t}}_{CW} = \begin{bmatrix} e_e/d \\ -e_n/d \end{bmatrix}$$

For **counter-clockwise (CCW)** orbit ($\lambda = +1$):

$$\hat{\mathbf{t}}_{CCW} = \begin{bmatrix} -e_e/d \\ e_n/d \end{bmatrix}$$

General form:

$$\hat{\mathbf{t}} = \lambda \begin{bmatrix} -e_e/d \\ e_n/d \end{bmatrix}$$

#### Step 4: Add Feedback Term for Convergence

To make the aircraft converge to the orbit from any starting position, we add a **radial feedback term**:

$$\mathbf{v}_d = \hat{\mathbf{t}} - k_{orbit} \cdot e_r \cdot \hat{\mathbf{r}}$$

where:
- $\hat{\mathbf{t}}$: Tangential component (maintains orbit)
- $k_{orbit} \cdot e_r \cdot \hat{\mathbf{r}}$: Radial component (corrects radius error)
- $k_{orbit}$: Guidance gain (determines convergence rate)

**Physical interpretation:**
- When $e_r > 0$ (outside orbit): radial term points inward → aircraft moves toward orbit
- When $e_r < 0$ (inside orbit): radial term points outward → aircraft moves away from center
- When $e_r = 0$ (on orbit): only tangential component remains → pure circular motion

#### Step 5: Compute Commanded Heading

The desired velocity direction is:

$$\psi_c = \text{atan2}(v_{d,e}, v_{d,n})$$

where $v_{d,n}$ and $v_{d,e}$ are the north and east components of $\mathbf{v}_d$.

### Complete Guidance Law

Combining all steps, the orbit guidance law is:

**For Clockwise (CW) orbit:**

$$\psi_c = \text{atan2}(e_n, e_e) + \frac{\pi}{2} - \text{atan}(k_{orbit} \cdot e_r)$$

**For Counter-Clockwise (CCW) orbit:**

$$\psi_c = \text{atan2}(e_n, e_e) - \frac{\pi}{2} + \text{atan}(k_{orbit} \cdot e_r)$$

where:
- $\text{atan2}(e_n, e_e)$: Angle from orbit center to aircraft
- $\pm \pi/2$: Tangential direction (perpendicular to radial)
- $\text{atan}(k_{orbit} \cdot e_r)$: Radial correction term

## Lyapunov Stability Analysis

### Lyapunov Function

To prove convergence, we define a Lyapunov function based on the radius error:

$$V(e_r) = \frac{1}{2} e_r^2$$

This represents the "energy" associated with being off the desired orbit.

### Convergence Proof

Taking the time derivative:

$$\dot{V} = e_r \cdot \dot{e_r}$$

The rate of change of radial error is:

$$\dot{e_r} = \dot{d} = \frac{d}{dt}\|\mathbf{e}\| = \frac{\mathbf{e}^T \dot{\mathbf{e}}}{\|\mathbf{e}\|}$$

Since $\dot{\mathbf{e}} = \dot{\mathbf{p}} = V_a \mathbf{v}_d$ (assuming perfect tracking of $\psi_c$):

$$\dot{e_r} = \frac{\mathbf{e}^T}{d} V_a \mathbf{v}_d = V_a \hat{\mathbf{r}}^T \mathbf{v}_d$$

Substituting the guidance law $\mathbf{v}_d = \hat{\mathbf{t}} - k_{orbit} e_r \hat{\mathbf{r}}$:

$$\dot{e_r} = V_a \hat{\mathbf{r}}^T (\hat{\mathbf{t}} - k_{orbit} e_r \hat{\mathbf{r}})$$

Since $\hat{\mathbf{r}} \perp \hat{\mathbf{t}}$ (perpendicular), $\hat{\mathbf{r}}^T \hat{\mathbf{t}} = 0$:

$$\dot{e_r} = -V_a k_{orbit} e_r$$

Therefore:

$$\dot{V} = e_r \dot{e_r} = -V_a k_{orbit} e_r^2$$

**Conclusion:**
- $\dot{V} < 0$ for all $e_r \neq 0$ (provided $k_{orbit} > 0$ and $V_a > 0$)
- $\dot{V} = 0$ only when $e_r = 0$ (on the orbit)

This proves **asymptotic stability**: the aircraft will converge to the desired orbit from any initial position.

### Convergence Rate

The radius error decays exponentially:

$$e_r(t) = e_r(0) \exp(-V_a k_{orbit} t)$$

The **time constant** is:

$$\tau = \frac{1}{V_a k_{orbit}}$$

**Design guideline:**
- Larger $k_{orbit}$ → faster convergence
- Higher airspeed $V_a$ → faster convergence
- Typical values: $k_{orbit} \in [1, 5]$

## Implementation Details

### Python Implementation

```python
class OrbitGuidance:
    def __init__(self, center, radius, direction='CW'):
        self.center = np.array(center)
        self.radius = radius
        self.direction = 1 if direction == 'CCW' else -1  # λ

    def compute_heading_command(self, position, k_orbit=1.0):
        # Position error (2D, ignore altitude)
        e_n = position[0] - self.center[0]
        e_e = position[1] - self.center[1]

        # Distance from center
        d = np.sqrt(e_n**2 + e_e**2)

        if d < 1e-6:  # Avoid division by zero
            return 0.0

        # Radial error
        e_r = d - self.radius

        # Angle from center to aircraft
        angle_to_aircraft = np.arctan2(e_e, e_n)

        # Commanded heading
        if self.direction == -1:  # Clockwise
            psi_c = angle_to_aircraft + np.pi/2 - np.arctan(k_orbit * e_r)
        else:  # Counter-clockwise
            psi_c = angle_to_aircraft - np.pi/2 + np.arctan(k_orbit * e_r)

        # Wrap to [-π, π]
        psi_c = np.arctan2(np.sin(psi_c), np.cos(psi_c))

        return psi_c
```

### Numerical Considerations

**Singularity Avoidance:**

When $d \approx 0$ (aircraft very close to orbit center), division by $d$ becomes numerically unstable. We add a small threshold:

```python
if d < 1e-6:
    return 0.0  # or last valid heading
```

**Angle Wrapping:**

Headings must be wrapped to $[-\pi, \pi]$ to avoid discontinuities:

```python
psi_c = np.arctan2(np.sin(psi_c), np.cos(psi_c))
```

**Gain Saturation:**

Very large guidance gains can cause chattering. We typically limit:

```python
k_orbit_effective = min(k_orbit, 10.0)
```

## GPS Measurement Integration

### Problem: Noisy Orbit Center

In practice, the orbit center $\mathbf{c}$ may be measured by GPS with errors:

$$\mathbf{c}_{meas} = \mathbf{c}_{true} + \mathbf{n}$$

where $\mathbf{n} \sim \mathcal{N}(0, \sigma_{GPS}^2)$ is GPS noise.

### GPS Error Model

The GPS sensor model includes:

1. **Gaussian noise**: $\sigma_h \approx 2.5$ m (horizontal), $\sigma_v \approx 4$ m (vertical)
2. **Slow drift**: $\mathbf{d}(t)$ with time constant $\tau_d \approx 15$ s
3. **Outliers**: Probability $p_{outlier} \approx 0.2\%$, magnitude $\approx 25$ m

Total measurement:

$$\mathbf{c}_{meas}(t) = \mathbf{c}_{true} + \mathbf{n}(t) + \mathbf{d}(t) + \mathbf{o}(t)$$

### Kalman Filter for Orbit Center Estimation

To reduce GPS noise, we use a **Kalman filter**:

**State model** (orbit center assumed stationary):

$$\mathbf{c}_{k+1} = \mathbf{c}_k + \mathbf{w}_k$$

where $\mathbf{w}_k \sim \mathcal{N}(0, Q)$ is process noise (small, since center is stationary).

**Measurement model:**

$$\mathbf{z}_k = \mathbf{c}_k + \mathbf{v}_k$$

where $\mathbf{v}_k \sim \mathcal{N}(0, R)$ is GPS measurement noise.

**Kalman filter equations:**

**Prediction:**
$$\hat{\mathbf{c}}_{k|k-1} = \hat{\mathbf{c}}_{k-1|k-1}$$
$$P_{k|k-1} = P_{k-1|k-1} + Q$$

**Update:**
$$K_k = P_{k|k-1}(P_{k|k-1} + R)^{-1}$$
$$\hat{\mathbf{c}}_{k|k} = \hat{\mathbf{c}}_{k|k-1} + K_k(\mathbf{z}_k - \hat{\mathbf{c}}_{k|k-1})$$
$$P_{k|k} = (I - K_k)P_{k|k-1}$$

**Parameter tuning:**
- $Q \approx 0.01$ (orbit center changes very slowly)
- $R \approx 6.25$ ($\sigma_{GPS}^2$ for 2.5m accuracy)

### Outlier Detection and Rejection

We use a **statistical outlier detector**:

1. Maintain sliding window of recent measurements
2. Compute mean $\mu$ and standard deviation $\sigma$
3. Reject measurement if: $|\mathbf{z}_k - \mu| > 3\sigma$

This prevents large GPS errors from corrupting the orbit center estimate.

### Complete GPS-Robust Guidance System

```
GPS → Outlier Detector → Kalman Filter → Orbit Guidance → Heading Command
```

**Algorithm:**

```python
# Each GPS update
if gps_updated:
    # Detect outliers
    if not outlier_detector.is_outlier(gps_measurement):
        # Update Kalman filter
        orbit_center_estimated = kalman_filter.update(gps_measurement)

        # Update guidance with filtered estimate
        orbit_guidance.center = orbit_center_estimated

# Every control loop (faster than GPS)
psi_c = orbit_guidance.compute_heading_command(position, k_orbit)
```

## Performance Analysis

### Theoretical Performance

**Without GPS errors:**
- Steady-state radius error: $< 1$ m (limited by control accuracy)
- Convergence time: $\tau = 1/(V_a k_{orbit}) \approx 1-5$ s for typical parameters

**With GPS errors ($\sigma_{GPS} = 2.5$ m):**
- Orbit center estimate error (RMS): $\approx 1.5$ m (after Kalman filtering)
- Induced radius error (RMS): $\approx 2$ m
- Outlier rejection rate: $> 99\%$

### Simulation Results

From `orbit_flight_with_gps.py` simulation:

| Metric | Value |
|--------|-------|
| GPS measurement RMS error | 2.8 m |
| Kalman filter RMS error | 1.4 m |
| Error reduction | 50% |
| Orbit radius RMS error | 3.2 m |
| GPS update rate | 5 Hz |
| Outliers detected | ~0.2% |

### Robustness Analysis

**Sensitivity to guidance gain $k_{orbit}$:**

| $k_{orbit}$ | Convergence | Robustness | Notes |
|-------------|-------------|------------|-------|
| 0.5 | Slow | High | Smooth but slow convergence |
| 1.0 | Moderate | High | Good balance |
| 2.5 | Fast | Moderate | Quick convergence |
| 5.0 | Very fast | Low | May oscillate with GPS noise |

**Recommendation:** $k_{orbit} \in [1, 3]$ for GPS-based systems.

## Practical Considerations

### Wind Effects

Wind causes **drift** in the orbit. The aircraft flies a circle relative to the air mass, which moves with the wind.

**Mitigation:**
- Use GPS position feedback (as implemented)
- Increase guidance gain slightly in windy conditions
- Consider wind estimation for feedforward compensation

### Coordinated Turns

The heading command $\psi_c$ must be converted to a **roll angle command** $\phi_c$ for coordinated turns:

$$\phi_c = \arctan\left(\frac{V_a^2 (\psi_c - \psi)k_\psi}{g}\right)$$

where $k_\psi$ is the heading tracking gain.

### Altitude Control

Altitude is controlled independently:

$$h_c = c_d$$

The orbit guidance operates primarily in the horizontal plane.

### Airspeed Management

Maintain constant airspeed:

$$V_{a,c} = \text{const}$$

The minimum orbit radius is constrained by:

$$R_{min} = \frac{V_a^2}{g \tan(\phi_{max})}$$

For $V_a = 25$ m/s and $\phi_{max} = 45°$:

$$R_{min} \approx 64 \text{ m}$$

### Initialization

For smooth engagement:

1. **Approach the orbit** from outside
2. **Gradually increase** guidance gain from 0 to nominal value
3. **Monitor radius error** for convergence

## Mathematical Summary

### Key Equations

**Orbit guidance law (general form):**

$$\psi_c = \text{atan2}(e_e, e_n) + \lambda \frac{\pi}{2} - \lambda \cdot \text{atan}(k_{orbit} \cdot e_r)$$

where:
- $(e_n, e_e) = (p_n - c_n, p_e - c_e)$: Position error
- $e_r = \sqrt{e_n^2 + e_e^2} - R$: Radial error
- $\lambda = +1$ (CCW) or $-1$ (CW): Orbit direction
- $k_{orbit}$: Guidance gain

**Stability:**

$$\dot{V} = -V_a k_{orbit} e_r^2 < 0 \quad \forall e_r \neq 0$$

**Convergence rate:**

$$e_r(t) = e_r(0) \exp(-V_a k_{orbit} t)$$

### Design Parameters

| Parameter | Symbol | Typical Value | Units |
|-----------|--------|---------------|-------|
| Orbit radius | $R$ | 50-500 | m |
| Guidance gain | $k_{orbit}$ | 1-3 | - |
| Airspeed | $V_a$ | 15-30 | m/s |
| GPS noise (horiz) | $\sigma_h$ | 2.5 | m |
| GPS update rate | $f_{GPS}$ | 5 | Hz |
| Kalman process noise | $Q$ | 0.01 | m² |
| Kalman meas noise | $R$ | 6.25 | m² |

## References

1. Beard, R. W., & McLain, T. W. (2012). *Small Unmanned Aircraft: Theory and Practice*. Princeton University Press.

2. Park, S., Deyst, J., & How, J. P. (2007). "Performance and Lyapunov Stability of a Nonlinear Path Following Guidance Method." *Journal of Guidance, Control, and Dynamics*, 30(6), 1718-1728.

3. Fossen, T. I., & Pettersen, K. Y. (2014). "On uniform semiglobal exponential stability (USGES) of proportional line-of-sight guidance laws." *Automatica*, 50(11), 2912-2917.

4. Lawrence, D. A., Frew, E. W., & Pisano, W. J. (2008). "Lyapunov Vector Fields for Autonomous UAV Flight Control." *Journal of Guidance, Control, and Dynamics*, 31(5), 1220-1229.

## Appendix: Alternative Formulations

### Proportional Navigation

An alternative is **proportional navigation** (PN):

$$\psi_c = \psi + k_{PN} \dot{\lambda}$$

where $\dot{\lambda}$ is the line-of-sight rate.

**Comparison:**
- PN: Better for intercept missions
- Vector field: Better for orbit/loiter

### Model Predictive Control (MPC)

For tighter tracking, MPC can be used:

- Predict future trajectory over horizon $T_h$
- Optimize control to minimize orbit error
- More computationally intensive

### Adaptive Guidance

For unknown wind:

- Estimate wind vector online
- Adjust guidance to compensate
- Requires extended Kalman filter (EKF)

---

**Document Version:** 1.0
**Last Updated:** 2025-10-21
**Author:** UAV Simulator Development Team
