from types import SimpleNamespace

from hazard_guard_person_detection.detection import detections_from_ultralytics


class TensorLike:
    def __init__(self, value):
        self.value = value

    def detach(self):
        return self

    def cpu(self):
        return self

    def tolist(self):
        return self.value


def test_filters_non_person_and_low_confidence_boxes():
    result = SimpleNamespace(
        boxes=SimpleNamespace(
            cls=TensorLike([0, 2, 0]),
            conf=TensorLike([0.91, 0.99, 0.20]),
            xyxy=TensorLike([[10, 20, 50, 100], [0, 0, 20, 20], [5, 5, 30, 30]]),
        )
    )

    detections = detections_from_ultralytics(result, minimum_confidence=0.4)

    assert len(detections) == 1
    assert detections[0].confidence == 0.91
    assert detections[0].center_x == 30.0
    assert detections[0].center_y == 60.0
    assert detections[0].size_x == 40.0
    assert detections[0].size_y == 80.0


def test_skips_malformed_and_inverted_boxes():
    result = SimpleNamespace(
        boxes=SimpleNamespace(
            cls=[0, 0, "bad"],
            conf=[0.8, 0.9, 0.9],
            xyxy=[[20, 20, 10, 30], [1, 2, 3], [0, 0, 5, 5]],
        )
    )

    assert detections_from_ultralytics(result) == []


def test_result_without_boxes_is_empty():
    assert detections_from_ultralytics(SimpleNamespace()) == []


def test_keeps_single_person_box_outer_dimension():
    result = SimpleNamespace(
        boxes=SimpleNamespace(
            cls=TensorLike([0]),
            conf=TensorLike([0.87]),
            xyxy=TensorLike([[11, 22, 55, 110]]),
        )
    )

    detections = detections_from_ultralytics(result, minimum_confidence=0.4)

    assert len(detections) == 1
    assert detections[0].center_x == 33.0


def test_skips_non_finite_confidence_and_coordinates():
    result = SimpleNamespace(
        boxes=SimpleNamespace(
            cls=[0, 0],
            conf=[float("nan"), 0.8],
            xyxy=[[0, 0, 10, 10], [0, 0, float("inf"), 10]],
        )
    )

    assert detections_from_ultralytics(result) == []
