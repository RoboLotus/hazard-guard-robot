from types import SimpleNamespace

from hazard_guard_person_detection.backend import UltralyticsBackend


class FakeModel:
    def __init__(self):
        self.arguments = None

    def predict(self, **kwargs):
        self.arguments = kwargs
        return [SimpleNamespace(boxes=None)]


def test_inference_uses_person_filter_and_config_without_loading_ultralytics():
    backend = UltralyticsBackend(
        model_path="models/yolo11n.pt",
        confidence=0.55,
        image_size=512,
        device="cuda:0",
    )
    fake_model = FakeModel()
    backend._model = fake_model
    image = object()

    result, inference_ms = backend.infer(image)

    assert result.boxes is None
    assert inference_ms >= 0.0
    assert backend.name == "ultralytics:yolo11n.pt"
    assert fake_model.arguments == {
        "source": image,
        "classes": [0],
        "conf": 0.55,
        "imgsz": 512,
        "verbose": False,
        "device": "cuda:0",
    }
