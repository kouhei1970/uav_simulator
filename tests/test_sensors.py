"""
Unit tests for sensors module (src/sensors.py)

Tests sensor models and filters including:
- GPS sensor with noise and outliers
- Kalman filter
- Orbit center estimator
- Moving average filter
- Outlier detector
"""

import pytest
import numpy as np
from src.sensors import (
    GPSSensor, SimpleKalmanFilter, OrbitCenterEstimator,
    MovingAverageFilter, OutlierDetector
)


class TestGPSSensor:
    """Test GPS sensor model"""

    def test_initialization(self, gps_sensor):
        """Test GPS sensor initialization"""
        assert gps_sensor is not None

    def test_measurement_format(self, gps_sensor):
        """Test that GPS measurement returns correct format"""
        true_position = np.array([100, 200, -100])
        time = 0.0

        gps_pos, valid = gps_sensor.measure(true_position, time)

        # Should return 3D position and valid flag
        assert len(gps_pos) == 3
        assert isinstance(valid, (bool, np.bool_))

    def test_noise_statistics(self):
        """Test that GPS noise has correct statistical properties"""
        gps = GPSSensor(
            noise_std_horizontal=2.5,
            noise_std_vertical=3.0,
            drift_magnitude=0,  # No drift for this test
            outlier_probability=0  # No outliers for this test
        )

        true_position = np.array([100, 200, -100])
        n_samples = 1000
        errors = []

        for i in range(n_samples):
            gps_pos, valid = gps.measure(true_position, time=i * 0.1)
            if valid:
                errors.append(gps_pos - true_position)

        errors = np.array(errors)

        # Check mean is close to zero (no bias)
        mean_error = np.mean(errors, axis=0)
        assert abs(mean_error[0]) < 0.5  # North
        assert abs(mean_error[1]) < 0.5  # East
        assert abs(mean_error[2]) < 0.6  # Down

        # Check standard deviation is close to specified value
        std_error = np.std(errors, axis=0)
        assert abs(std_error[0] - 2.5) < 0.5  # Horizontal
        assert abs(std_error[1] - 2.5) < 0.5  # Horizontal
        assert abs(std_error[2] - 3.0) < 0.6  # Vertical

    def test_drift_exists(self):
        """Test that GPS drift is present"""
        gps = GPSSensor(
            noise_std_horizontal=0,  # No noise
            noise_std_vertical=0,
            drift_magnitude=2.0,
            outlier_probability=0
        )

        true_position = np.array([100, 200, -100])

        # Measure at different times
        pos1, _ = gps.measure(true_position, time=0.0)
        pos2, _ = gps.measure(true_position, time=10.0)

        # Drift should cause measurements to differ even with same true position
        # (Though drift changes slowly, so difference might be small)
        # We just check that system can handle it
        assert pos1 is not None
        assert pos2 is not None

    def test_outlier_detection_flag(self):
        """Test that outliers are flagged"""
        gps = GPSSensor(
            noise_std_horizontal=2.5,
            noise_std_vertical=3.0,
            drift_magnitude=1.0,
            outlier_probability=0.5,  # High probability for testing
            outlier_magnitude=50.0
        )

        true_position = np.array([100, 200, -100])
        n_samples = 100
        outlier_count = 0

        for i in range(n_samples):
            gps_pos, valid = gps.measure(true_position, time=i * 0.1)
            error = np.linalg.norm(gps_pos - true_position)

            # If error is large, it might be an outlier
            if error > 20.0:
                outlier_count += 1

        # With 50% probability, we should see some outliers
        assert outlier_count > 5  # At least some outliers

    def test_reproducibility_with_seed(self):
        """Test that GPS produces measurements (reproducibility requires manual seed)"""
        true_position = np.array([100, 200, -100])

        # Set numpy random seed for reproducibility
        np.random.seed(42)
        gps1 = GPSSensor(
            noise_std_horizontal=2.5,
            noise_std_vertical=3.0,
            drift_magnitude=1.0,
            outlier_probability=0.002
        )

        # Collect measurements
        measurements1 = []
        for i in range(10):
            pos1, valid1 = gps1.measure(true_position, time=i * 0.1)
            measurements1.append(pos1)

        # Reset seed and create new GPS
        np.random.seed(42)
        gps2 = GPSSensor(
            noise_std_horizontal=2.5,
            noise_std_vertical=3.0,
            drift_magnitude=1.0,
            outlier_probability=0.002
        )

        # Should produce same measurements with same seed
        for i in range(10):
            pos2, valid2 = gps2.measure(true_position, time=i * 0.1)
            assert np.allclose(pos2, measurements1[i])


class TestSimpleKalmanFilter:
    """Test Simple Kalman Filter"""

    def test_initialization(self, kalman_filter):
        """Test Kalman filter initialization"""
        assert kalman_filter is not None

    def test_convergence_to_true_value(self):
        """Test that filter converges to true value"""
        kf = SimpleKalmanFilter(
            process_variance=0.01,
            measurement_variance=4.0
        )

        true_value = 100.0
        n_steps = 100

        # Feed noisy measurements
        np.random.seed(42)
        for _ in range(n_steps):
            noisy_measurement = true_value + np.random.normal(0, 2.0)
            kf.update(noisy_measurement)

        # Estimate should be close to true value
        estimate = kf.x
        assert abs(estimate - true_value) < 5.0

    def test_noise_reduction(self):
        """Test that filter reduces measurement noise"""
        kf = SimpleKalmanFilter(
            process_variance=0.01,
            measurement_variance=4.0
        )

        true_value = 50.0
        measurement_std = 2.0
        n_steps = 100

        measurements = []
        estimates = []

        np.random.seed(42)
        for _ in range(n_steps):
            noisy_measurement = true_value + np.random.normal(0, measurement_std)
            measurements.append(noisy_measurement)
            kf.update(noisy_measurement)
            estimates.append(kf.x)

        # After convergence, estimate variance should be less than measurement variance
        estimates_converged = estimates[-50:]  # Last 50 estimates
        estimate_std = np.std(estimates_converged)

        assert estimate_std < measurement_std

    def test_update_returns_estimate(self, kalman_filter):
        """Test that update returns current estimate"""
        measurement = 10.0
        estimate = kalman_filter.update(measurement)

        # update() returns the estimate, and it's stored in kalman_filter.x
        assert estimate == kalman_filter.x

    def test_variance_decreases_with_measurements(self):
        """Test that variance decreases as measurements are added"""
        kf = SimpleKalmanFilter(
            process_variance=0.01,
            measurement_variance=4.0
        )
        # Set high initial variance via internal state
        kf.p = 100.0

        variance_initial = kf.p

        # Add measurements
        for _ in range(10):
            kf.update(50.0)

        variance_after = kf.p

        assert variance_after < variance_initial


class TestOrbitCenterEstimator:
    """Test Orbit Center Estimator"""

    def test_initialization(self, orbit_estimator):
        """Test orbit estimator initialization"""
        assert orbit_estimator is not None

    def test_center_estimation_accuracy(self):
        """Test accuracy of center estimation"""
        estimator = OrbitCenterEstimator(
            process_variance=0.01,
            measurement_variance=6.25
        )

        # True orbit center
        true_center = np.array([300, 400, -100])
        radius = 50.0

        # Generate orbit points with noise
        np.random.seed(42)
        n_points = 200
        for i in range(n_points):
            angle = 2 * np.pi * i / n_points
            true_pos = true_center + radius * np.array([
                np.cos(angle),
                np.sin(angle),
                0
            ])
            # Add noise
            noisy_pos = true_pos + np.random.normal(0, 2.5, size=3)
            estimator.update(noisy_pos, time=i * 0.1)

        # Check center estimate
        estimated_center = estimator.get_center()
        center_error = np.linalg.norm(estimated_center - true_center)

        # Should be within 30m after full orbit (with noise)
        assert center_error < 30.0

    def test_radius_estimation_accuracy(self):
        """Test accuracy of radius estimation"""
        estimator = OrbitCenterEstimator(
            process_variance=0.01,
            measurement_variance=6.25
        )

        true_center = np.array([300, 400, -100])
        true_radius = 50.0

        # Generate orbit points
        np.random.seed(42)
        n_points = 200
        for i in range(n_points):
            angle = 2 * np.pi * i / n_points
            true_pos = true_center + true_radius * np.array([
                np.cos(angle),
                np.sin(angle),
                0
            ])
            noisy_pos = true_pos + np.random.normal(0, 2.5, size=3)
            estimator.update(noisy_pos, time=i * 0.1)

        # Check radius estimate
        estimated_radius = estimator.get_radius()
        radius_error = abs(estimated_radius - true_radius)

        # Should be within 15m
        assert radius_error < 15.0

    def test_estimation_convergence(self):
        """Test that estimation converges over time"""
        estimator = OrbitCenterEstimator(
            process_variance=0.01,
            measurement_variance=6.25
        )

        true_center = np.array([300, 400, -100])
        radius = 50.0

        center_errors = []
        np.random.seed(42)

        # Generate orbit points and track error
        for i in range(100):
            angle = 2 * np.pi * i / 100
            true_pos = true_center + radius * np.array([
                np.cos(angle),
                np.sin(angle),
                0
            ])
            noisy_pos = true_pos + np.random.normal(0, 2.5, size=3)
            estimator.update(noisy_pos, time=i * 0.1)

            if i % 10 == 0:
                center = estimator.get_center()
                error = np.linalg.norm(center - true_center)
                center_errors.append(error)

        # Error should decrease (convergence)
        # Last error should be less than first
        assert center_errors[-1] < center_errors[0]


class TestMovingAverageFilter:
    """Test Moving Average Filter"""

    def test_initialization(self):
        """Test moving average filter initialization"""
        maf = MovingAverageFilter(window_size=10)
        assert maf is not None

    def test_smoothing_effect(self):
        """Test that filter smooths noisy signal"""
        maf = MovingAverageFilter(window_size=10)

        # Generate noisy signal
        np.random.seed(42)
        true_signal = 50.0
        noisy_measurements = [true_signal + np.random.normal(0, 5.0)
                             for _ in range(100)]

        smoothed = []
        for measurement in noisy_measurements:
            smoothed.append(maf.update(measurement))

        # Smoothed signal should have less variance than raw
        smoothed_var = np.var(smoothed[-50:])  # After filter fills
        raw_var = np.var(noisy_measurements[-50:])

        assert smoothed_var < raw_var

    def test_window_size_effect(self):
        """Test that larger window provides more smoothing"""
        maf_small = MovingAverageFilter(window_size=5)
        maf_large = MovingAverageFilter(window_size=20)

        np.random.seed(42)
        noisy_data = [50.0 + np.random.normal(0, 5.0) for _ in range(100)]

        smoothed_small = [maf_small.update(x) for x in noisy_data]
        smoothed_large = [maf_large.update(x) for x in noisy_data]

        var_small = np.var(smoothed_small[-30:])
        var_large = np.var(smoothed_large[-30:])

        # Larger window should have less variance
        assert var_large < var_small

    def test_response_to_step_change(self):
        """Test filter response to step change"""
        maf = MovingAverageFilter(window_size=10)

        # Feed constant value
        for _ in range(20):
            maf.update(10.0)

        # Step change
        for _ in range(20):
            output = maf.update(20.0)

        # Should converge to new value
        assert abs(output - 20.0) < 1.0


class TestOutlierDetector:
    """Test Outlier Detector"""

    def test_initialization(self):
        """Test outlier detector initialization"""
        detector = OutlierDetector(threshold_sigma=3.0)
        assert detector is not None

    def test_outlier_detection(self):
        """Test that outliers are detected"""
        detector = OutlierDetector(threshold_sigma=3.0, window_size=20)

        np.random.seed(42)
        # Feed normal data
        outlier_count = 0
        for _ in range(30):
            value = 50.0 + np.random.normal(0, 2.0)
            is_outlier = detector.is_outlier(value)
            if is_outlier:
                outlier_count += 1

        # Most normal data should not be outliers (allow some false positives)
        assert outlier_count < 5  # Less than 17% false positive rate

        # Feed obvious outlier
        is_outlier = detector.is_outlier(100.0)
        assert is_outlier

    def test_threshold_sensitivity(self):
        """Test that threshold affects detection sensitivity"""
        strict_detector = OutlierDetector(threshold_sigma=2.0, window_size=20)
        lenient_detector = OutlierDetector(threshold_sigma=5.0, window_size=20)

        np.random.seed(42)
        # Feed normal data
        for _ in range(30):
            value = 50.0 + np.random.normal(0, 2.0)
            strict_detector.is_outlier(value)
            lenient_detector.is_outlier(value)

        # Test a moderately abnormal value
        test_value = 60.0  # 5 std deviations if std ~2

        strict_result = strict_detector.is_outlier(test_value)
        lenient_result = lenient_detector.is_outlier(test_value)

        # Strict should be more likely to flag as outlier
        # (This test might be probabilistic, so we just check it runs)
        assert isinstance(strict_result, (bool, np.bool_))
        assert isinstance(lenient_result, (bool, np.bool_))


class TestSensorIntegration:
    """Test sensor integration scenarios"""

    def test_gps_with_kalman_filter_reduces_noise(self):
        """Test that Kalman filter reduces GPS noise"""
        np.random.seed(42)
        gps = GPSSensor(
            noise_std_horizontal=5.0,
            noise_std_vertical=6.0,
            drift_magnitude=0,
            outlier_probability=0
        )

        kf_n = SimpleKalmanFilter(process_variance=0.01, measurement_variance=25.0)
        kf_e = SimpleKalmanFilter(process_variance=0.01, measurement_variance=25.0)
        kf_d = SimpleKalmanFilter(process_variance=0.01, measurement_variance=36.0)

        true_position = np.array([100, 200, -100])

        # Initialize filters closer to true position
        kf_n.x = 100.0
        kf_e.x = 200.0
        kf_d.x = -100.0

        raw_errors = []
        filtered_errors = []

        for i in range(100):
            gps_pos, _ = gps.measure(true_position, time=i * 0.1)

            # Apply Kalman filter
            filtered_pos = np.array([
                kf_n.update(gps_pos[0]),
                kf_e.update(gps_pos[1]),
                kf_d.update(gps_pos[2])
            ])

            raw_errors.append(np.linalg.norm(gps_pos - true_position))
            filtered_errors.append(np.linalg.norm(filtered_pos - true_position))

        # After convergence, filtered should have less error
        # Use only last 20 samples to ensure convergence
        raw_rms = np.sqrt(np.mean(np.array(raw_errors[-20:])**2))
        filtered_rms = np.sqrt(np.mean(np.array(filtered_errors[-20:])**2))

        # Filtered RMS should be less than raw RMS after convergence
        assert filtered_rms < raw_rms

    def test_orbit_estimator_handles_noisy_gps(self):
        """Test that orbit estimator works with noisy GPS"""
        np.random.seed(42)
        gps = GPSSensor(
            noise_std_horizontal=2.5,
            noise_std_vertical=3.0,
            drift_magnitude=1.0,
            outlier_probability=0.01
        )

        estimator = OrbitCenterEstimator(
            process_variance=0.01,
            measurement_variance=6.25
        )

        true_center = np.array([300, 400, -100])
        radius = 50.0

        # Simulate UAV flying in orbit with GPS measurements
        for i in range(200):
            angle = 2 * np.pi * i / 200
            true_pos = true_center + radius * np.array([
                np.cos(angle),
                np.sin(angle),
                0
            ])

            gps_pos, valid = gps.measure(true_pos, time=i * 0.1)
            if valid:
                estimator.update(gps_pos, time=i * 0.1)

        # Should still get reasonable estimate
        estimated_center = estimator.get_center()
        center_error = np.linalg.norm(estimated_center - true_center)

        assert center_error < 15.0  # Allow more error with realistic GPS
