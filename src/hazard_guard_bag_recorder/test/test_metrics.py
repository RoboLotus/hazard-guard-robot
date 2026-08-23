from hazard_guard_bag_recorder.metrics import DriveMetrics


def test_drive_metrics_uses_odom_samples_for_distance_and_stops():
    metrics = DriveMetrics()
    metrics.observe_odometry(0, 0, 0, 0, 10.0)
    metrics.observe_odometry(3, 4, 1, 0, 12.0)
    metrics.observe_odometry(3, 4, 0, 0, 15.0)
    output = metrics.as_dict()
    assert output["distance_m"] == 5.0
    assert output["max_speed_mps"] == 1.0
    assert output["stopped_seconds"] == 3.0


def test_drive_metrics_counts_stationary_interval_using_current_velocity():
    metrics = DriveMetrics()
    metrics.observe_odometry(0, 0, 1, 0, 1.0)
    metrics.observe_odometry(1, 0, 0, 0, 3.0)
    metrics.observe_odometry(1, 0, 0, 0, 6.0)
    assert metrics.as_dict()["stopped_seconds"] == 5.0
