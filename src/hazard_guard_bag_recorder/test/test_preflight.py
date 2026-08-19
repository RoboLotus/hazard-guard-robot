from pathlib import Path

from hazard_guard_bag_recorder.preflight import PreflightError, run_preflight
from hazard_guard_bag_recorder.profiles import load_profile_document, resolve_profile


PACKAGE = Path(__file__).resolve().parent.parent


def profile(name="navigation-core"):
    return resolve_profile(name, load_profile_document(PACKAGE / "config" / "profiles.json"))


def test_preflight_resolves_available_aliases_and_reports_missing_optional(tmp_path):
    report = run_preflight(
        profile(),
        {"/tf", "/tf_static", "/odom", "/scan"},
        tmp_path,
        minimum_free_bytes=0,
    )
    assert report.can_start
    assert report.selected_topics["scan"] == "/scan"
    assert "map" in report.missing_optional


def test_preflight_blocks_missing_required_topics(tmp_path):
    report = run_preflight(profile(), {"/tf", "/odom"}, tmp_path, minimum_free_bytes=0)
    assert not report.can_start
    assert set(report.missing_required) == {"tf_static", "scan"}


def test_preflight_requires_explicit_opt_in_for_unconfirmed_thermal_topics(tmp_path):
    try:
        run_preflight(profile("thermal-calibration"), set(), tmp_path, minimum_free_bytes=0)
    except PreflightError as exc:
        assert "experimental" in str(exc)
    else:
        raise AssertionError("experimental profile must be blocked by default")
