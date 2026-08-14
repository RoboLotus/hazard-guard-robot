import struct

import pytest
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from hazard_guard_thermal_analysis.cloud import (
    create_thermal_cloud,
    decode_scalar_image,
    iter_thermal_cloud,
)
from hazard_guard_thermal_analysis.projection import ThermalPoint


def test_centikelvin_image_decodes_to_celsius() -> None:
    image = Image()
    image.width = 2
    image.height = 1
    image.encoding = "mono16"
    image.step = 4
    image.data = struct.pack("<HH", 29815, 33315)

    values = decode_scalar_image(image, scale=0.01, offset=-273.15)

    assert values == pytest.approx([25.0, 60.0])


def test_thermal_point_cloud_contract_round_trip() -> None:
    header = Header()
    header.frame_id = "thermal_camera_optical_frame"
    source = [ThermalPoint(1.0, 2.0, 3.0, 64.5, 0.8)]

    cloud = create_thermal_cloud(header, source)
    restored = list(iter_thermal_cloud(cloud))

    assert cloud.point_step == 20
    assert [field.name for field in cloud.fields] == [
        "x",
        "y",
        "z",
        "temperature_c",
        "confidence",
    ]
    assert restored[0].x == pytest.approx(1.0)
    assert restored[0].temperature_c == pytest.approx(64.5)
