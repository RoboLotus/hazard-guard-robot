import struct

import pytest
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import Header

from hazard_guard_thermal_analysis.cloud import (
    create_thermal_cloud,
    decode_scalar_array,
    decode_scalar_image,
    iter_thermal_cloud,
    temperature_rgb,
)
from hazard_guard_thermal_analysis.projection import ThermalPoint


def test_centikelvin_image_decodes_to_celsius() -> None:
    image = Image()
    image.width = 2
    image.height = 1
    image.encoding = "mono16"
    image.step = 4
    image.data = struct.pack("<HH", 29815, 33315)
    assert decode_scalar_image(image, scale=0.01, offset=-273.15) == pytest.approx([25.0, 60.0])


def test_thermal_point_cloud_contract_round_trip_includes_radiometric_pixel() -> None:
    header = Header()
    header.frame_id = "thermal_camera_optical_frame"
    source = [ThermalPoint(1.0, 2.0, 3.0, 64.5, 0.8, 12.0, 34.0)]
    cloud = create_thermal_cloud(header, source)
    restored = list(iter_thermal_cloud(cloud))
    assert cloud.point_step == 32
    assert [field.name for field in cloud.fields] == [
        "x", "y", "z", "temperature_c", "confidence", "pixel_u", "pixel_v", "rgb"
    ]
    assert restored[0].x == pytest.approx(1.0)
    assert restored[0].temperature_c == pytest.approx(64.5)
    assert (restored[0].pixel_u, restored[0].pixel_v) == pytest.approx((12.0, 34.0))
    assert struct.unpack_from("<I", cloud.data, 28)[0] == temperature_rgb(64.5, 10.0, 60.0)


def test_numpy_scalar_decoder_honours_padded_rows() -> None:
    image = Image()
    image.width = 2
    image.height = 2
    image.encoding = "16UC1"
    image.step = 6
    image.data = struct.pack("<HHH", 1000, 2000, 9999) + struct.pack(
        "<HHH", 3000, 4000, 9999
    )
    decoded = decode_scalar_array(image, scale=0.001)
    assert decoded.reshape(-1).tolist() == pytest.approx([1.0, 2.0, 3.0, 4.0])


def test_temperature_colour_is_fixed_and_clamped() -> None:
    assert temperature_rgb(-50.0, 10.0, 60.0) == temperature_rgb(10.0, 10.0, 60.0)
    assert temperature_rgb(500.0, 10.0, 60.0) == temperature_rgb(60.0, 10.0, 60.0)
    cold = temperature_rgb(10.0, 10.0, 60.0)
    hot = temperature_rgb(60.0, 10.0, 60.0)
    assert (cold & 0xFF) > ((cold >> 16) & 0xFF)
    assert ((hot >> 16) & 0xFF) > (hot & 0xFF)


def test_reader_remains_compatible_with_old_five_field_cloud() -> None:
    cloud = PointCloud2()
    cloud.height = 1
    cloud.width = 1
    cloud.fields = [
        PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
        for index, name in enumerate(("x", "y", "z", "temperature_c", "confidence"))
    ]
    cloud.point_step = 20
    cloud.row_step = 20
    cloud.data = list(struct.pack("<fffff", 1.0, 2.0, 3.0, 40.0, 0.9))
    restored = list(iter_thermal_cloud(cloud))[0]
    assert restored.temperature_c == pytest.approx(40.0)
    assert restored.pixel_u == -1.0
    assert restored.pixel_v == -1.0
