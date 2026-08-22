import json

import pytest

from hazard_guard_thermal_analysis.baseline import load_baselines


def baseline_document(*, sample_count=8, schema_version=2):
    return {
        "schema_version": schema_version,
        "equipment": {
            "motor": {
                "equipment": {
                    "temperature_c": 35.0,
                    "sample_count": sample_count,
                    "environment_delta_c": 15.0,
                    "state": "provisional",
                },
                "voxels": {},
            }
        },
    }


def test_simple_baseline_requires_eight_patrols(tmp_path):
    path = tmp_path / "short.json"
    path.write_text(
        json.dumps(baseline_document(sample_count=7)), encoding="utf-8"
    )
    with pytest.raises(ValueError, match="at least 8"):
        load_baselines(path)


def test_valid_simple_baseline_loads_without_sigma(tmp_path):
    path = tmp_path / "baseline.json"
    path.write_text(json.dumps(baseline_document()), encoding="utf-8")
    baseline = load_baselines(path)["motor"].equipment
    assert baseline.temperature_c == 35.0
    assert baseline.sample_count == 8
    assert baseline.environment_delta_c == 15.0
    assert baseline.sigma_normal_c == 0.0
    assert baseline.state == "provisional"


def test_schema1_detailed_baseline_remains_compatible(tmp_path):
    document = baseline_document(schema_version=1)
    document["equipment"]["motor"]["equipment"].update(
        {
            "sigma_normal_c": 1.2,
            "sigma_repeat_c": 0.2,
            "sigma_residual_c": 1.0,
            "sensor_quantization_c": 0.1,
        }
    )
    path = tmp_path / "legacy.json"
    path.write_text(json.dumps(document), encoding="utf-8")
    baseline = load_baselines(path)["motor"].equipment
    assert baseline.sigma_normal_c == 1.2
    assert baseline.sensor_quantization_c == 0.1
