"""Lazy Ultralytics inference backend."""

from pathlib import Path
from time import perf_counter
from typing import Any, Optional, Tuple


class UltralyticsBackend:
    """Load YOLO only on the first inference call.

    Keeping the import and model construction lazy allows ROS package discovery,
    launch inspection, and pure unit tests to work before Jetson inference
    dependencies have been installed.
    """

    def __init__(
        self,
        *,
        model_path: str = "yolo11n.pt",
        confidence: float = 0.4,
        image_size: int = 640,
        device: str = "",
        person_class_id: int = 0,
    ) -> None:
        self.model_path = model_path
        self.confidence = confidence
        self.image_size = image_size
        self.device = device.strip()
        self.person_class_id = person_class_id
        self._model: Optional[Any] = None

    @property
    def name(self) -> str:
        return f"ultralytics:{Path(self.model_path).name}"

    def _ensure_model(self) -> Any:
        if self._model is None:
            try:
                from ultralytics import YOLO
            except ImportError as exc:
                raise RuntimeError(
                    "Ultralytics is not installed. Install the JetPack-compatible "
                    "inference environment before enabling person detection."
                ) from exc
            self._model = YOLO(self.model_path)
        return self._model

    def infer(self, image: Any) -> Tuple[Any, float]:
        model = self._ensure_model()
        arguments = {
            "source": image,
            "classes": [self.person_class_id],
            "conf": self.confidence,
            "imgsz": self.image_size,
            "verbose": False,
        }
        if self.device:
            arguments["device"] = self.device
        started = perf_counter()
        results = model.predict(**arguments)
        elapsed_ms = (perf_counter() - started) * 1000.0
        if not results:
            raise RuntimeError("Ultralytics returned no result object")
        return results[0], elapsed_ms
