from pathlib import Path


LAUNCH = Path(__file__).parents[1] / "launch" / "person_detection.launch.py"


def test_runtime_tuning_arguments_reach_the_detector_node():
    source = LAUNCH.read_text(encoding="utf-8")

    for name in ("confidence", "image_size", "inference_rate_hz"):
        assert f'DeclareLaunchArgument("{name}"' in source
        assert f'LaunchConfiguration("{name}")' in source

    assert 'LaunchConfiguration("confidence"), value_type=float' in source
    assert 'LaunchConfiguration("image_size"), value_type=int' in source
    assert 'LaunchConfiguration("inference_rate_hz"), value_type=float' in source
