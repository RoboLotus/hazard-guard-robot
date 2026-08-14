"""The detector must react to people only, and only to confident ones.

A COCO model labels chairs, bags and monitors in the same result as people.
If a stray class or a low-confidence guess reached the Bool, navigation
would brake for furniture.
"""
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from types import SimpleNamespace

SCRIPT = Path(__file__).parents[1] / "scripts" / "person_detector.py"
SPEC = spec_from_file_location("person_detector", SCRIPT)
MODULE = module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def fake_box(class_id, score, xyxy=(10.0, 20.0, 30.0, 40.0)):
    return SimpleNamespace(cls=class_id, conf=score, xyxy=[xyxy])


def fake_result(*boxes):
    return SimpleNamespace(boxes=list(boxes))


def test_only_person_class_counts():
    result = fake_result(
        fake_box(MODULE.PERSON_CLASS_ID, 0.9),
        fake_box(56, 0.95),  # chair
        fake_box(62, 0.99),  # tv
    )
    assert len(MODULE.person_boxes(result, 0.4)) == 1


def test_low_confidence_people_are_dropped():
    result = fake_result(
        fake_box(MODULE.PERSON_CLASS_ID, 0.35),
        fake_box(MODULE.PERSON_CLASS_ID, 0.41),
    )
    boxes = MODULE.person_boxes(result, 0.4)
    assert [round(box[4], 2) for box in boxes] == [0.41]


def test_box_corners_are_passed_through():
    result = fake_result(
        fake_box(MODULE.PERSON_CLASS_ID, 0.8, (1.0, 2.0, 3.0, 4.0))
    )
    assert MODULE.person_boxes(result, 0.4)[0][:4] == (1.0, 2.0, 3.0, 4.0)


if __name__ == "__main__":
    test_only_person_class_counts()
    test_low_confidence_people_are_dropped()
    test_box_corners_are_passed_through()
    print("ok")
