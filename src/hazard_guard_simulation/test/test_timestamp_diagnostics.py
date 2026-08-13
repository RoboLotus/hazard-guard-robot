from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
import sys

import pytest


SCRIPT = Path(__file__).parents[1] / "scripts" / "timestamp_diagnostics.py"
SPEC = spec_from_file_location("timestamp_diagnostics", SCRIPT)
MODULE = module_from_spec(SPEC)
assert SPEC.loader is not None
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def test_topic_stats_reports_signed_skew_p95_and_tf_ratio():
    stats = MODULE.TopicStats(window_size=3)
    stats.add(stamp_ns=12_000_000_000, now_ns=10_000_000_000,
              odom_stamp_ns=10_000_000_000, has_tf=False)
    stats.add(stamp_ns=10_500_000_000, now_ns=11_000_000_000,
              odom_stamp_ns=10_000_000_000, has_tf=True)
    stats.add(stamp_ns=12_000_000_000, now_ns=12_000_000_000,
              odom_stamp_ns=12_000_000_000, has_tf=True)

    summary = stats.summary()

    assert summary["age_sec"]["mean"] == pytest.approx(-0.5)
    assert summary["age_sec"]["p95_abs"] == 2.0
    assert summary["offset_to_latest_odom_sec"]["mean"] == pytest.approx(5 / 6)
    assert summary["tf_available_ratio"] == pytest.approx(2 / 3)


def test_window_discards_old_samples_but_keeps_total_count():
    stats = MODULE.TopicStats(window_size=2)
    for stamp in (1, 2, 3):
        stats.add(
            stamp_ns=stamp * 1_000_000_000,
            now_ns=4_000_000_000,
            odom_stamp_ns=None,
            has_tf=True,
        )

    summary = stats.summary()

    assert summary["received"] == 3
    assert summary["window_samples"] == 2
    assert summary["age_sec"]["mean"] == pytest.approx(1.5)
    assert summary["tf_available_ratio"] == 1.0


def test_reports_include_all_topics_and_explain_negative_age():
    payload = {
        "generated_at_ns": 123,
        "topics": {
            "rgb": {
                "received": 1,
                "window_samples": 1,
                "age_sec": {"mean": -2.0, "p95_abs": 2.0},
                "offset_to_latest_odom_sec": {
                    "mean": 2.0,
                    "p95_abs": 2.0,
                },
                "tf_available_ratio": 0.5,
            }
        },
    }

    csv_report = MODULE.render_csv(payload)
    markdown = MODULE.render_markdown(payload)

    assert "rgb,1,1,-2.0,2.0,2.0,2.0,0.5" in csv_report
    assert "| rgb | 1 | -2.0000 | 2.0000 | 2.0000 | 0.5000 |" in markdown
    assert "negative age" in markdown


def test_tf_ratio_uses_same_rolling_window_as_timestamp_stats():
    stats = MODULE.TopicStats(window_size=2)
    for has_tf in (False, False, True):
        stats.add(
            stamp_ns=1,
            now_ns=2,
            odom_stamp_ns=None,
            has_tf=has_tf,
        )

    assert stats.summary()["tf_available_ratio"] == 0.5
