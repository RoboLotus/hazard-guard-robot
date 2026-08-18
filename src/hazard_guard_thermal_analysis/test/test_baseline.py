import json

import pytest

from hazard_guard_thermal_analysis.baseline import load_baselines


def baseline_document(*, sample_count=10, include_quantization=True):
    values = {
        "temperature_c": 35.0,
        "sigma_normal_c": 1.2,
        "sigma_repeat_c": 0.2,
        "sigma_residual_c": 1.0,
        "sample_count": sample_count,
        "state": "provisional",
    }
    if include_quantization:
        values["sensor_quantization_c"] = 0.1
    return {
        "schema_version": 1,
        "equipment": {
            "motor": {"equipment": values, "voxels": {}}
        },
    }


def test_baseline_file_requires_sensor_quantization_and_ten_samples(tmp_path):
    missing_quantization = tmp_path / "missing.json"
    missing_quantization.write_text(
        json.dumps(baseline_document(include_quantization=False)),
        encoding="utf-8",
    )
    with pytest.raises(KeyError, match="sensor_quantization_c"):
        load_baselines(missing_quantization)

    too_short = tmp_path / "short.json"
    too_short.write_text(
        json.dumps(baseline_document(sample_count=9)), encoding="utf-8"
    )
    with pytest.raises(ValueError, match="at least 10"):
        load_baselines(too_short)


def test_valid_provisional_baseline_loads(tmp_path):
    path = tmp_path / "baseline.json"
    path.write_text(json.dumps(baseline_document()), encoding="utf-8")
    baseline = load_baselines(path)["motor"].equipment
    assert baseline.sample_count == 10
    assert baseline.sensor_quantization_c == 0.1
    assert baseline.state == "provisional"
