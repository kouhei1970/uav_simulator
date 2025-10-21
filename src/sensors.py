"""
Sensor Models for UAV Simulation

This module implements realistic sensor models including GPS with noise,
drift, and outliers to simulate real-world measurement conditions.
"""

import numpy as np


class GPSSensor:
    """
    GPS sensor model with realistic measurement errors.

    Simulates:
    - Gaussian noise (typical GPS accuracy)
    - Slow drift (atmospheric effects, satellite geometry changes)
    - Occasional outliers (multipath, satellite dropouts)
    """

    def __init__(self,
                 noise_std_horizontal=2.0,
                 noise_std_vertical=3.0,
                 drift_magnitude=1.0,
                 drift_time_constant=10.0,
                 outlier_probability=0.001,
                 outlier_magnitude=20.0,
                 update_rate=5.0):
        """
        Initialize GPS sensor model.

        Parameters:
            noise_std_horizontal: Standard deviation of horizontal position noise [m]
            noise_std_vertical: Standard deviation of vertical position noise [m]
            drift_magnitude: Magnitude of slow drift [m]
            drift_time_constant: Time constant for drift changes [s]
            outlier_probability: Probability of outlier per measurement
            outlier_magnitude: Typical magnitude of outliers [m]
            update_rate: GPS update rate [Hz]
        """
        self.noise_std_horizontal = noise_std_horizontal
        self.noise_std_vertical = noise_std_vertical
        self.drift_magnitude = drift_magnitude
        self.drift_time_constant = drift_time_constant
        self.outlier_probability = outlier_probability
        self.outlier_magnitude = outlier_magnitude
        self.update_rate = update_rate
        self.dt_update = 1.0 / update_rate

        # Internal state
        self.drift = np.array([0.0, 0.0, 0.0])  # Current drift in NED
        self.last_update_time = 0.0
        self.last_measurement = None

    def measure(self, true_position, time):
        """
        Measure position with GPS errors.

        Parameters:
            true_position: True position [pn, pe, pd] in NED frame [m]
            time: Current simulation time [s]

        Returns:
            measured_position: GPS measurement [pn, pe, pd] with errors [m]
            is_updated: Boolean indicating if new GPS measurement is available
        """
        # Check if it's time for a new GPS update
        time_since_update = time - self.last_update_time

        if time_since_update < self.dt_update:
            # Return last measurement (GPS hasn't updated yet)
            if self.last_measurement is not None:
                return self.last_measurement.copy(), False
            else:
                return true_position.copy(), False

        # Time for new measurement
        self.last_update_time = time

        # Update drift (slow random walk)
        alpha = 1.0 - np.exp(-time_since_update / self.drift_time_constant)
        self.drift = (1 - alpha) * self.drift + \
                     alpha * self.drift_magnitude * np.random.randn(3)

        # Add Gaussian noise
        noise = np.array([
            np.random.normal(0, self.noise_std_horizontal),
            np.random.normal(0, self.noise_std_horizontal),
            np.random.normal(0, self.noise_std_vertical)
        ])

        # Occasional outliers
        if np.random.random() < self.outlier_probability:
            outlier = self.outlier_magnitude * np.random.randn(3)
            print(f"GPS outlier detected at time {time:.2f}s: {np.linalg.norm(outlier):.1f}m")
        else:
            outlier = np.zeros(3)

        # Combine all error sources
        measured_position = true_position + noise + self.drift + outlier

        self.last_measurement = measured_position

        return measured_position, True

    def reset(self):
        """Reset sensor state."""
        self.drift = np.array([0.0, 0.0, 0.0])
        self.last_update_time = 0.0
        self.last_measurement = None


class MovingAverageFilter:
    """
    Simple moving average filter for smoothing noisy measurements.
    """

    def __init__(self, window_size=5):
        """
        Initialize moving average filter.

        Parameters:
            window_size: Number of samples to average
        """
        self.window_size = window_size
        self.buffer = []

    def update(self, measurement):
        """
        Add new measurement and return filtered value.

        Parameters:
            measurement: New measurement (can be scalar or array)

        Returns:
            filtered_value: Moving average of recent measurements
        """
        self.buffer.append(np.array(measurement))

        # Keep only recent measurements
        if len(self.buffer) > self.window_size:
            self.buffer.pop(0)

        # Compute average
        return np.mean(self.buffer, axis=0)

    def reset(self):
        """Reset filter state."""
        self.buffer = []


class SimpleKalmanFilter:
    """
    Simple 1D Kalman filter for smoothing GPS measurements.
    """

    def __init__(self, process_variance=0.1, measurement_variance=4.0):
        """
        Initialize Kalman filter.

        Parameters:
            process_variance: Process noise variance (how much state changes)
            measurement_variance: Measurement noise variance (GPS accuracy)
        """
        self.q = process_variance  # Process noise
        self.r = measurement_variance  # Measurement noise

        self.x = 0.0  # Estimated state
        self.p = 1.0  # Estimation error covariance

    def update(self, measurement):
        """
        Update filter with new measurement.

        Parameters:
            measurement: New measurement value

        Returns:
            x: Filtered estimate
        """
        # Prediction
        # x = x (assume constant position)
        self.p = self.p + self.q

        # Update
        k = self.p / (self.p + self.r)  # Kalman gain
        self.x = self.x + k * (measurement - self.x)
        self.p = (1 - k) * self.p

        return self.x

    def reset(self, initial_value=0.0):
        """Reset filter state."""
        self.x = initial_value
        self.p = 1.0


class OrbitCenterEstimator:
    """
    Estimates orbit center from noisy GPS measurements.

    Uses separate Kalman filters for North, East, and Down coordinates.
    """

    def __init__(self, process_variance=0.01, measurement_variance=4.0):
        """
        Initialize orbit center estimator.

        Parameters:
            process_variance: How much we expect the orbit center to change
            measurement_variance: GPS measurement noise variance
        """
        # Three Kalman filters for N, E, D coordinates
        self.filters = [
            SimpleKalmanFilter(process_variance, measurement_variance),
            SimpleKalmanFilter(process_variance, measurement_variance),
            SimpleKalmanFilter(process_variance, measurement_variance)
        ]

        self.initialized = False

    def update(self, measured_center):
        """
        Update orbit center estimate.

        Parameters:
            measured_center: Measured orbit center [pn, pe, pd] with GPS errors

        Returns:
            estimated_center: Filtered orbit center estimate
        """
        if not self.initialized:
            # Initialize filters with first measurement
            for i, filt in enumerate(self.filters):
                filt.reset(measured_center[i])
            self.initialized = True

        # Update each coordinate independently
        estimated_center = np.array([
            self.filters[0].update(measured_center[0]),
            self.filters[1].update(measured_center[1]),
            self.filters[2].update(measured_center[2])
        ])

        return estimated_center

    def reset(self):
        """Reset estimator state."""
        self.initialized = False


class OutlierDetector:
    """
    Detects and rejects outlier measurements.

    Uses statistical methods to identify measurements that are
    inconsistent with recent history.
    """

    def __init__(self, threshold_sigma=3.0, window_size=10):
        """
        Initialize outlier detector.

        Parameters:
            threshold_sigma: Number of standard deviations for outlier threshold
            window_size: Number of recent measurements to use for statistics
        """
        self.threshold_sigma = threshold_sigma
        self.window_size = window_size
        self.buffer = []

    def is_outlier(self, measurement):
        """
        Check if measurement is an outlier.

        Parameters:
            measurement: New measurement (scalar or array)

        Returns:
            is_outlier: True if measurement is detected as outlier
        """
        if len(self.buffer) < 3:
            # Not enough data to detect outliers
            self.buffer.append(np.array(measurement))
            return False

        # Compute statistics from recent measurements
        recent = np.array(self.buffer)
        mean = np.mean(recent, axis=0)
        std = np.std(recent, axis=0) + 1e-6  # Avoid division by zero

        # Check if measurement is too far from mean
        deviation = np.abs(measurement - mean)
        is_outlier = np.any(deviation > self.threshold_sigma * std)

        if not is_outlier:
            # Add to buffer only if not outlier
            self.buffer.append(np.array(measurement))
            if len(self.buffer) > self.window_size:
                self.buffer.pop(0)

        return is_outlier

    def reset(self):
        """Reset detector state."""
        self.buffer = []
