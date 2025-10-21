# L1 Adaptive Guidance Theory

## Overview

The L1 adaptive guidance law is a nonlinear trajectory tracking algorithm specifically designed for fixed-wing unmanned aerial vehicles (UAVs). This document provides a comprehensive theoretical foundation and implementation details for L1 guidance.

## 1. Introduction

### 1.1 Background

The L1 guidance law was developed by Park, Deyst, and How (2004) in their paper "A New Nonlinear Guidance Logic for Trajectory Tracking". It addresses the challenge of accurate path following for aircraft with:

- Significant lateral position errors
- Course angle errors
- Wind disturbances
- Varying airspeeds

### 1.2 Key Advantages

1. **Adaptive**: Automatically adjusts to different airspeeds
2. **Robust**: Handles large initial errors effectively
3. **Simple**: Elegant geometric interpretation
4. **Proven**: Widely used in commercial autopilots (ArduPilot, PX4)

### 1.3 Applications

- Straight line path following
- Circular orbit tracking
- Waypoint navigation
- Survey missions
- Loiter patterns

## 2. Mathematical Foundation

### 2.1 Coordinate Systems

**North-East-Down (NED) Frame**: Inertial reference frame
- $p_n$: North position [m]
- $p_e$: East position [m]
- $p_d$: Down position [m] (negative altitude)

**Course Angle**: Ground track direction
$$\chi = \arctan2(v_e, v_n)$$

where $v_n$, $v_e$ are ground velocity components.

### 2.2 L1 Distance Parameter

The L1 distance is a look-ahead distance that determines how far ahead the guidance law "looks" on the desired path:

$$L_1 = \frac{1}{\pi} \zeta T V_a$$

where:
- $\zeta$: Damping ratio (typically 0.5-1.0)
- $T$: Period parameter [s] (typically 10-25 s)
- $V_a$: Airspeed [m/s]

**Physical Interpretation**:
- Larger $L_1$ → Smoother, more gentle corrections
- Smaller $L_1$ → Tighter tracking, more aggressive maneuvers

**Typical Values**:
- Small UAV (15 m/s): $L_1 \approx 34$ m ($\zeta=0.707$, $T=15$ s)
- Medium UAV (25 m/s): $L_1 \approx 56$ m ($\zeta=0.707$, $T=15$ s)

## 3. Straight Line Path Following

### 3.1 Problem Formulation

Given:
- Line defined by start point $\mathbf{r}_s$ and end point $\mathbf{r}_e$
- Current position $\mathbf{p}$
- Current course angle $\chi$
- Airspeed $V_a$

Find: Lateral acceleration command $a_c$ to track the line

### 3.2 Geometric Setup

**Path Direction**:
$$\mathbf{q} = \frac{\mathbf{r}_e - \mathbf{r}_s}{|\mathbf{r}_e - \mathbf{r}_s|}$$

**Path Course Angle**:
$$\chi_q = \arctan2(q_e, q_n)$$

**Position Error Vector**:
$$\mathbf{r} = \mathbf{p} - \mathbf{r}_s$$

**Crosstrack Error** (perpendicular distance from path):
$$e_{py} = \mathbf{r} \times \mathbf{q} = r_n q_e - r_e q_n$$

(In 2D, this is the z-component of the cross product)

Positive $e_{py}$ means the aircraft is to the left of the path.

### 3.3 L1 Guidance Law for Lines

The lateral acceleration command is:

$$a_c = \frac{2 V_a^2}{L_1} \sin\left(\arctan2(-e_{py}, L_1)\right)$$

**Derivation**:

The L1 law is based on creating a vector field that drives the aircraft toward a point on the path that is $L_1$ distance ahead along the approach angle.

The acceleration required to track a circular arc of radius $R$ at velocity $V_a$ is:
$$a = \frac{V_a^2}{R}$$

For L1 guidance, the effective radius is related to the crosstrack error and look-ahead distance:
$$\sin(\eta) = \frac{-e_{py}}{L_1}$$

where $\eta$ is the approach angle.

Therefore:
$$a_c = \frac{2V_a^2}{L_1} \sin(\eta)$$

### 3.4 Course Angle Error

The course angle error can be computed as:
$$\epsilon_\chi = \chi_q - \chi$$

wrapped to $[-\pi, \pi]$.

### 3.5 Convergence Properties

The L1 law guarantees:
1. **Asymptotic stability**: $e_{py} \rightarrow 0$ as $t \rightarrow \infty$
2. **No overshoot**: For well-tuned parameters
3. **Exponential convergence**: Error decreases exponentially

## 4. Circular Orbit Following

### 4.1 Problem Formulation

Given:
- Orbit center $\mathbf{c} = [c_n, c_e, c_d]^T$
- Orbit radius $R$
- Orbit direction: CW (clockwise) or CCW (counter-clockwise)
- Current position $\mathbf{p}$
- Current course angle $\chi$
- Airspeed $V_a$

Find: Lateral acceleration command $a_c$ to track the orbit

### 4.2 Geometric Setup

**Distance Vector from Center**:
$$\mathbf{d} = [p_n - c_n, p_e - c_e]^T$$

**Distance from Center**:
$$d = |\mathbf{d}| = \sqrt{(p_n - c_n)^2 + (p_e - c_e)^2}$$

**Angle from Center**:
$$\alpha = \arctan2(p_e - c_e, p_n - c_n)$$

**Radius Error**:
$$e_r = d - R$$

Positive $e_r$ means outside the orbit, negative means inside.

### 4.3 Direction Parameter

Define:
$$\lambda = \begin{cases}
+1 & \text{for CW (clockwise)} \\
-1 & \text{for CCW (counter-clockwise)}
\end{cases}$$

### 4.4 Desired Course Angle

The desired course angle is tangent to the orbit:
$$\chi_q = \alpha + \lambda \frac{\pi}{2}$$

For CW orbits, this is 90° ahead of the radial direction.
For CCW orbits, this is 90° behind the radial direction.

### 4.5 L1 Guidance Law for Orbits

The lateral acceleration command consists of two components:

$$a_c = a_{lateral} + a_{centripetal}$$

**Lateral Correction** (drives toward orbit):
$$a_{lateral} = \frac{2V_a^2}{L_1} \sin\left(\arctan2(\lambda e_r, L_1)\right)$$

**Centripetal Acceleration** (maintains circular motion):
$$a_{centripetal} = \lambda \frac{V_a^2}{R}$$

**Total Command**:
$$a_c = \frac{2V_a^2}{L_1} \sin\left(\arctan2(\lambda e_r, L_1)\right) + \lambda \frac{V_a^2}{R}$$

### 4.6 Physical Interpretation

- The centripetal term maintains the circular motion
- The lateral term corrects for radius errors
- When on the orbit ($e_r = 0$), only centripetal acceleration remains
- The sign of $\lambda$ determines the turn direction

## 5. Roll Angle Command Generation

### 5.1 Coordinated Turn Relationship

In a coordinated turn (no sideslip), the lateral acceleration is:

$$a = g \tan(\phi)$$

where:
- $g = 9.81$ m/s²: Gravitational acceleration
- $\phi$: Bank angle (roll angle) [rad]

### 5.2 Roll Command

From the lateral acceleration command, compute the bank angle:

$$\phi_c = \arctan\left(\frac{a_c}{g}\right)$$

### 5.3 Bank Angle Limits

For safety and comfort, limit the bank angle:

$$\phi_c = \text{saturate}(\phi_c, -\phi_{max}, \phi_{max})$$

Typical limits:
- Small UAV: $\phi_{max} = 45°$ (0.785 rad)
- Aerobatic: $\phi_{max} = 60°$ (1.047 rad)
- Commercial: $\phi_{max} = 30°$ (0.524 rad)

## 6. Parameter Tuning Guidelines

### 6.1 L1 Period ($T$)

**Effect**:
- Larger $T$ → Larger $L_1$ → Gentler turns → Better for slow aircraft
- Smaller $T$ → Smaller $L_1$ → Tighter tracking → Better for agile aircraft

**Recommended Values**:
- Slow/stable aircraft: $T = 20-25$ s
- Medium aircraft: $T = 15-20$ s
- Fast/agile aircraft: $T = 10-15$ s

### 6.2 L1 Damping ($\zeta$)

**Effect**:
- $\zeta = 0.707$ (critically damped): Optimal for most cases
- $\zeta < 0.707$ (underdamped): Faster response, possible oscillation
- $\zeta > 0.707$ (overdamped): Slower response, no oscillation

**Recommended Values**:
- Standard: $\zeta = 0.707$
- Windy conditions: $\zeta = 0.8-1.0$
- Calm conditions: $\zeta = 0.6-0.7$

### 6.3 Fixed L1 Distance

Alternatively, specify $L_1$ directly:

**Rule of Thumb**:
$$L_1 = (2-5) \times V_a$$

For example, at 15 m/s:
- Conservative: $L_1 = 30$ m
- Standard: $L_1 = 45$ m
- Aggressive: $L_1 = 75$ m

## 7. Stability Analysis

### 7.1 Lyapunov Analysis for Line Following

Consider the Lyapunov candidate function:
$$V = \frac{1}{2} e_{py}^2$$

Taking the time derivative and showing it is negative definite proves stability.

**Result**: The L1 guidance law is globally asymptotically stable for straight line following.

### 7.2 Orbit Following Stability

For circular orbits, similar analysis shows:
- Inner loops (radius control) are stable
- Outer loops (tangent tracking) are stable
- Combined system is stable for $L_1 > 0$

## 8. Comparison with Other Methods

### 8.1 vs. Pure Pursuit

**L1 Advantages**:
- Adaptive to airspeed changes
- Better handling of large errors
- Smoother transitions

**Pure Pursuit**:
- Simpler implementation
- Fixed look-ahead distance

### 8.2 vs. Vector Field Guidance

**L1 Advantages**:
- More aggressive for large errors
- Proven field performance

**Vector Field**:
- Smoother far from path
- Better theoretical guarantees

### 8.3 vs. PID Control

**L1 Advantages**:
- Nonlinear (handles large errors)
- No integral windup
- Adaptive behavior

**PID**:
- Works for small errors
- Tuning experience available

## 9. Implementation Considerations

### 9.1 Course Angle Computation

Use GPS velocity when available:
$$\chi = \arctan2(v_e, v_n)$$

In simulation, use body velocity projected to NED:
$$\chi = \arctan2(v, u)$$

where $u$, $v$ are body-frame velocities transformed to NED.

### 9.2 Wind Compensation

L1 guidance naturally handles wind:
- Uses course angle $\chi$ (ground track), not heading $\psi$
- Airspeed $V_a$ in the $L_1$ calculation
- Wind triangle automatically resolved

### 9.3 Transition Between Paths

When switching paths:
1. Update path parameters instantly
2. $L_1$ distance adapts automatically
3. No special logic needed

### 9.4 Numerical Considerations

**Angle Wrapping**: Always wrap angles to $[-\pi, \pi]$:
```python
def wrap_angle(angle):
    while angle > π:
        angle -= 2π
    while angle < -π:
        angle += 2π
    return angle
```

**Singularity at Center**: For orbits, check $d > d_{min}$ (e.g., 0.1 m) to avoid division by zero.

## 10. Example Calculations

### 10.1 Straight Line Example

Given:
- Line from $(0, 0)$ to $(1000, 500)$ m
- Aircraft at $(200, 100)$ m
- Course angle $\chi = 30°$
- Airspeed $V_a = 15$ m/s
- $L_1 = 35$ m

**Step 1**: Path direction
$$\mathbf{q} = \frac{1}{\sqrt{1000^2 + 500^2}} [1000, 500]^T = [0.894, 0.447]^T$$

**Step 2**: Crosstrack error
$$e_{py} = 200 \times 0.447 - 100 \times 0.894 = 89.4 - 89.4 = 0$$

Aircraft is on the path!

**Step 3**: Lateral acceleration
$$a_c = \frac{2 \times 15^2}{35} \sin(\arctan2(0, 35)) = 0 \text{ m/s}^2$$

No correction needed.

### 10.2 Orbit Example

Given:
- Orbit center at $(400, 400)$ m
- Orbit radius $R = 60$ m
- Direction: CW ($\lambda = 1$)
- Aircraft at $(450, 400)$ m
- Course angle $\chi = 90°$ (East)
- Airspeed $V_a = 15$ m/s
- $L_1 = 35$ m

**Step 1**: Distance from center
$$d = \sqrt{(450-400)^2 + (400-400)^2} = 50 \text{ m}$$

**Step 2**: Radius error
$$e_r = 50 - 60 = -10 \text{ m}$$

Aircraft is inside the orbit.

**Step 3**: Desired course angle
$$\alpha = \arctan2(0, 50) = 0°$$
$$\chi_q = 0° + 90° = 90°$$

**Step 4**: Lateral acceleration
$$a_{lateral} = \frac{2 \times 15^2}{35} \sin(\arctan2(-10, 35)) = 12.86 \times \sin(-15.9°) = -3.5 \text{ m/s}^2$$
$$a_{centripetal} = \frac{15^2}{60} = 3.75 \text{ m/s}^2$$
$$a_c = -3.5 + 3.75 = 0.25 \text{ m/s}^2$$

**Step 5**: Roll command
$$\phi_c = \arctan(0.25 / 9.81) = 1.5°$$

Small correction to move outward.

## 11. Performance Metrics

### 11.1 Steady-State Error

For straight lines:
- Typical: < 2 m
- Best case: < 0.5 m

For orbits:
- Typical: < 3 m radius error
- Best case: < 1 m radius error

### 11.2 Convergence Time

From large initial errors (50 m):
- 50% reduction: ~10-15 seconds
- 90% reduction: ~30-40 seconds

### 11.3 Sensitivity to Wind

L1 guidance handles winds up to 50% of airspeed:
- 15 m/s airspeed: Handles 7.5 m/s wind
- Crosstrack error increases slightly
- No instability

## 12. Advanced Topics

### 12.1 Adaptive L1

For varying conditions, adapt $L_1$ online:
$$L_1(t) = f(V_a(t), \text{wind}(t), \text{error}(t))$$

### 12.2 Path Transitions

Smooth transition between line segments:
- Fillet paths at corners
- Use orbit segments for turns
- Dubins paths for optimal transitions

### 12.3 3D Extensions

Extend to 3D paths:
- Add altitude control loop
- Separate lateral and vertical guidance
- Maintain constant airspeed

## 13. References

1. Park, S., Deyst, J., and How, J. P. (2004). "A New Nonlinear Guidance Logic for Trajectory Tracking". *AIAA Guidance, Navigation, and Control Conference and Exhibit*.

2. Beard, R. W., and McLain, T. W. (2012). *Small Unmanned Aircraft: Theory and Practice*. Princeton University Press.

3. ArduPilot Development Team. "L1 Controller Documentation". https://ardupilot.org

4. Sujit, P. B., Saripalli, S., and Sousa, J. B. (2014). "Unmanned Aerial Vehicle Path Following: A Survey and Analysis of Algorithms for Fixed-Wing Unmanned Aerial Vehicles". *IEEE Control Systems Magazine*, 34(1), 42-59.

## 14. Summary

The L1 adaptive guidance law provides:
- **Robust** path following for fixed-wing UAVs
- **Simple** geometric interpretation
- **Adaptive** to varying flight conditions
- **Proven** performance in real-world applications

Key equations:
- L1 distance: $L_1 = \frac{1}{\pi} \zeta T V_a$
- Line tracking: $a_c = \frac{2 V_a^2}{L_1} \sin(\arctan2(-e_{py}, L_1))$
- Orbit tracking: $a_c = \frac{2V_a^2}{L_1} \sin(\arctan2(\lambda e_r, L_1)) + \lambda \frac{V_a^2}{R}$
- Roll command: $\phi_c = \arctan(a_c / g)$

With proper tuning ($\zeta \approx 0.707$, $T = 15$ s), L1 guidance delivers excellent tracking performance for a wide range of fixed-wing UAV applications.
