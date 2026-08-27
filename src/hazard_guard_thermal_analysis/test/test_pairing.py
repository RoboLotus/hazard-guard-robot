from hazard_guard_thermal_analysis.pairing import (
    newest_unconsumed_receipt,
    receipt_age_seconds,
)


def test_receipt_pairing_uses_newest_depth_frame() -> None:
    assert newest_unconsumed_receipt(
        [(10.0, "old"), (10.3, "new"), (10.2, "middle")],
        after_receipt=-1.0,
    ) == (10.3, "new")


def test_receipt_pairing_does_not_reuse_published_depth_frame() -> None:
    samples = [(10.0, "old"), (10.3, "published")]

    assert newest_unconsumed_receipt(
        samples,
        after_receipt=10.3,
    ) is None
    assert newest_unconsumed_receipt(
        [*samples, (10.4, "next")],
        after_receipt=10.3,
    ) == (10.4, "next")


def test_stale_unconsumed_depth_keeps_its_original_receipt_age() -> None:
    selected = newest_unconsumed_receipt(
        [(10.0, "stale-depth")],
        after_receipt=9.0,
    )

    assert selected == (10.0, "stale-depth")
    age = receipt_age_seconds(receipt=selected[0], now=10.35)
    assert 0.34 < age < 0.36
    assert age > 0.2
    # The fusion callback advances the watermark when it drops this stale
    # receipt, so later thermal frames cannot pair the same depth again.
    assert newest_unconsumed_receipt(
        [(10.0, "stale-depth")],
        after_receipt=selected[0],
    ) is None
