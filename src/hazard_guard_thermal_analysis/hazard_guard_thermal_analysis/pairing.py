"""Small, ROS-independent helpers for physical camera frame pairing."""

from __future__ import annotations

from collections.abc import Iterable
from typing import TypeVar


FrameT = TypeVar("FrameT")


def newest_unconsumed_receipt(
    samples: Iterable[tuple[float, FrameT]],
    *,
    after_receipt: float,
) -> tuple[float, FrameT] | None:
    """Return the newest sample that has not backed a published cloud yet."""

    eligible = (
        sample for sample in samples if sample[0] > after_receipt
    )
    return max(eligible, key=lambda sample: sample[0], default=None)


def receipt_age_seconds(*, receipt: float, now: float) -> float:
    """Return source receipt age without replacing it with a message stamp."""

    return max(0.0, float(now) - float(receipt))
