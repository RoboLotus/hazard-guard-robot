#!/usr/bin/env python3
"""Tiny standalone ThermoEye temperature viewer. ROS is not used."""

import sys
import time

import cv2
import numpy as np
from TmCore.TmCamera import TmCamera
from TmCore.TmTypes import ColorOrder, ColormapTypes, TempUnit


SENSOR_WIDTH = 160
SENSOR_HEIGHT = 120
DISPLAY_SCALE = 4
WINDOW_NAME = "ThermoEye Temperature"


def pixels_as_image(values, width, height):
    pixels = np.asarray(values)
    if pixels.shape == (width, height):
        pixels = pixels.T
    if pixels.shape != (height, width):
        raise ValueError(f"unexpected pixel shape: {pixels.shape}")
    return pixels.astype(np.uint16, copy=False)


def bitmap_as_bgr(values, width, height):
    rgb = np.frombuffer(values, dtype=np.uint8)
    return rgb.reshape(height, width, 3)[:, :, ::-1].copy()


def pick_camera(camera):
    return next(
        (item for item in camera.get_local_camera_list()
         if item.name.startswith("TMC")),
        None,
    )


def main():
    camera = TmCamera()
    info = pick_camera(camera)
    if info is None:
        sys.exit("No TMC camera found. Check USB and /dev/ttyACM* permission.")

    formats = [item.format for item in info.media_info_list]
    if "Y16" not in formats:
        sys.exit(f"Y16 is unavailable: {formats}")
    if not camera.open_local_camera(
        info.name, info.com_port, info.index, "Y16"
    ):
        sys.exit(
            "open_local_camera failed. Another process is probably using "
            "/dev/video2 or /dev/ttyACM0. Stop the ROS thermal camera launch "
            "with Ctrl+C, then run this viewer again."
        )

    # query_frame() is TmSDK's polling mode. begin_acquisition() is only for
    # callback mode and must not run at the same time.
    camera.set_temp_unit(TempUnit.CELSIUS)
    camera.set_color_map(ColormapTypes.Inferno)
    camera.set_noise_filtering(True)

    mouse = [SENSOR_WIDTH // 2, SENSOR_HEIGHT // 2]

    def on_mouse(event, x, y, _flags, _data):
        if event in (cv2.EVENT_MOUSEMOVE, cv2.EVENT_LBUTTONDOWN):
            mouse[0] = min(SENSOR_WIDTH - 1, max(0, x // DISPLAY_SCALE))
            mouse[1] = min(SENSOR_HEIGHT - 1, max(0, y // DISPLAY_SCALE))

    cv2.namedWindow(WINDOW_NAME)
    cv2.setMouseCallback(WINDOW_NAME, on_mouse)
    fps_time = time.monotonic()
    fps_frames = 0
    fps = 0.0

    print(f"{info.name} @ {info.com_port} | move mouse to measure | Q quit")
    try:
        while True:
            frame = camera.query_frame(SENSOR_WIDTH, SENSOR_HEIGHT)
            if frame is None:
                if not camera.is_connected():
                    print("camera disconnected")
                    break
                continue

            width = int(frame.width())
            height = int(frame.height())
            raw = pixels_as_image(
                frame.get_pixel(0, 0, width, height), width, height
            )
            image = bitmap_as_bgr(
                frame.to_bitmap(ColorOrder.COLOR_RGB), width, height
            )

            point_raw = int(raw[mouse[1], mouse[0]])
            point_c = float(camera.get_temperature(point_raw))
            minimum_c = float(camera.get_temperature(int(raw.min())))
            average_c = float(camera.get_temperature(int(round(raw.mean()))))
            maximum_c = float(camera.get_temperature(int(raw.max())))
            maximum_y, maximum_x = np.unravel_index(np.argmax(raw), raw.shape)

            image = cv2.resize(
                image,
                (width * DISPLAY_SCALE, height * DISPLAY_SCALE),
                interpolation=cv2.INTER_NEAREST,
            )
            point = (
                mouse[0] * DISPLAY_SCALE + DISPLAY_SCALE // 2,
                mouse[1] * DISPLAY_SCALE + DISPLAY_SCALE // 2,
            )
            maximum = (
                int(maximum_x) * DISPLAY_SCALE + DISPLAY_SCALE // 2,
                int(maximum_y) * DISPLAY_SCALE + DISPLAY_SCALE // 2,
            )
            cv2.drawMarker(
                image, point, (255, 255, 255), cv2.MARKER_CROSS, 20, 2
            )
            cv2.drawMarker(
                image, maximum, (0, 0, 255), cv2.MARKER_CROSS, 20, 2
            )

            fps_frames += 1
            elapsed = time.monotonic() - fps_time
            if elapsed >= 1.0:
                fps = fps_frames / elapsed
                fps_frames = 0
                fps_time = time.monotonic()

            cv2.rectangle(image, (0, 0), (image.shape[1], 58), (0, 0, 0), -1)
            cv2.putText(
                image,
                f"POINT {point_c:.1f} C   MAX {maximum_c:.1f} C",
                (10, 24), cv2.FONT_HERSHEY_SIMPLEX,
                0.65, (255, 255, 255), 2, cv2.LINE_AA,
            )
            cv2.putText(
                image,
                f"AVG {average_c:.1f} C   MIN {minimum_c:.1f} C   {fps:.1f} fps",
                (10, 49), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (230, 230, 230), 1, cv2.LINE_AA,
            )
            cv2.imshow(WINDOW_NAME, image)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    finally:
        cv2.destroyAllWindows()
        camera.close()


if __name__ == "__main__":
    main()
