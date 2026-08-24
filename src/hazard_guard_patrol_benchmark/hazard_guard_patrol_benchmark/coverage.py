from __future__ import annotations

import ast
from collections import deque
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable

from .metrics import PoseSample


def _yaml_values(path: Path) -> dict[str, object]:
    values: dict[str, object] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or ":" not in line:
            continue
        key, raw_value = line.split(":", 1)
        value = raw_value.strip()
        try:
            values[key.strip()] = ast.literal_eval(value)
        except (SyntaxError, ValueError):
            values[key.strip()] = value.strip("'\"")
    return values


def _next_token(data: bytes, offset: int) -> tuple[bytes, int]:
    length = len(data)
    while offset < length:
        if data[offset] in b" \t\r\n":
            offset += 1
            continue
        if data[offset] == ord("#"):
            newline = data.find(b"\n", offset)
            offset = length if newline < 0 else newline + 1
            continue
        break
    end = offset
    while end < length and data[end] not in b" \t\r\n#":
        end += 1
    if end == offset:
        raise ValueError("PGM 헤더 토큰을 읽을 수 없습니다")
    return data[offset:end], end


def read_pgm(path: Path) -> tuple[int, int, bytes]:
    data = path.read_bytes()
    magic, offset = _next_token(data, 0)
    width_token, offset = _next_token(data, offset)
    height_token, offset = _next_token(data, offset)
    maximum_token, offset = _next_token(data, offset)
    width = int(width_token)
    height = int(height_token)
    maximum = int(maximum_token)
    if width <= 0 or height <= 0 or maximum <= 0 or maximum > 255:
        raise ValueError(f"지원하지 않는 PGM 형식입니다: {path}")
    expected = width * height
    if magic == b"P5":
        if offset >= len(data) or data[offset] not in b" \t\r\n":
            raise ValueError("P5 PGM 헤더 뒤 구분 문자가 없습니다")
        if data[offset:offset + 2] == b"\r\n":
            offset += 2
        else:
            offset += 1
        pixels = data[offset:offset + expected]
    elif magic == b"P2":
        values: list[int] = []
        for _ in range(expected):
            token, offset = _next_token(data, offset)
            values.append(round(int(token) * 255 / maximum))
        pixels = bytes(values)
    else:
        raise ValueError(f"P2/P5 PGM만 지원합니다: {magic!r}")
    if len(pixels) != expected:
        raise ValueError(
            f"PGM 픽셀 수가 잘못되었습니다: {len(pixels)} != {expected}"
        )
    if maximum != 255 and magic == b"P5":
        pixels = bytes(round(value * 255 / maximum) for value in pixels)
    return width, height, pixels


@dataclass(frozen=True)
class CoverageResult:
    map_area_m2: float
    free_area_m2: float
    patrolable_area_m2: float
    observed_area_m2: float
    coverage_percent: float
    map_cell_count: int
    free_cell_count: int
    patrolable_cell_count: int
    observed_cell_count: int

    def as_dict(self) -> dict[str, float | int]:
        return {
            "map_area_m2": round(self.map_area_m2, 3),
            "free_area_m2": round(self.free_area_m2, 3),
            "patrolable_area_m2": round(self.patrolable_area_m2, 3),
            "observed_area_m2": round(self.observed_area_m2, 3),
            "coverage_percent": round(self.coverage_percent, 3),
            "map_cell_count": self.map_cell_count,
            "free_cell_count": self.free_cell_count,
            "patrolable_cell_count": self.patrolable_cell_count,
            "observed_cell_count": self.observed_cell_count,
        }


@dataclass
class MapGrid:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    free_mask: bytearray

    @classmethod
    def from_yaml(cls, path: Path) -> "MapGrid":
        values = _yaml_values(path)
        image_value = values.get("image")
        if not isinstance(image_value, str) or not image_value:
            raise ValueError(f"지도 YAML에 image가 없습니다: {path}")
        image_path = (path.parent / image_value).resolve()
        width, height, pixels = read_pgm(image_path)
        resolution = float(values["resolution"])
        origin = values.get("origin")
        if not isinstance(origin, (list, tuple)) or len(origin) < 3:
            raise ValueError(f"지도 YAML origin이 잘못되었습니다: {path}")
        negate = int(values.get("negate", 0))
        free_threshold = float(values.get("free_thresh", 0.196))
        free_mask = bytearray(width * height)
        for world_row in range(height):
            image_row = height - world_row - 1
            image_offset = image_row * width
            world_offset = world_row * width
            for column in range(width):
                pixel = pixels[image_offset + column] / 255.0
                occupancy = pixel if negate else 1.0 - pixel
                if occupancy < free_threshold:
                    free_mask[world_offset + column] = 1
        return cls(
            width=width,
            height=height,
            resolution=resolution,
            origin_x=float(origin[0]),
            origin_y=float(origin[1]),
            origin_yaw=float(origin[2]),
            free_mask=free_mask,
        )

    @property
    def cell_area_m2(self) -> float:
        return self.resolution * self.resolution

    def world_to_cell(self, x: float, y: float) -> tuple[int, int] | None:
        delta_x = x - self.origin_x
        delta_y = y - self.origin_y
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        local_x = cosine * delta_x + sine * delta_y
        local_y = -sine * delta_x + cosine * delta_y
        column = math.floor(local_x / self.resolution)
        row = math.floor(local_y / self.resolution)
        if 0 <= column < self.width and 0 <= row < self.height:
            return column, row
        return None

    def index(self, column: int, row: int) -> int:
        return row * self.width + column

    def _inflated_free_mask(self, clearance_m: float) -> bytearray:
        radius_cells = max(0, math.ceil(clearance_m / self.resolution))
        if radius_cells == 0:
            return bytearray(self.free_mask)
        size = self.width * self.height
        distance = bytearray([255]) * size
        queue: deque[int] = deque()
        for index, is_free in enumerate(self.free_mask):
            row, column = divmod(index, self.width)
            if not is_free:
                distance[index] = 0
                queue.append(index)
            elif row in (0, self.height - 1) or column in (0, self.width - 1):
                # Outside the occupancy grid is unavailable space. Give edge
                # cells distance one so clearance also respects map bounds.
                distance[index] = 1
                queue.append(index)
        neighbours = (
            (-1, -1),
            (0, -1),
            (1, -1),
            (-1, 0),
            (1, 0),
            (-1, 1),
            (0, 1),
            (1, 1),
        )
        while queue:
            current = queue.popleft()
            current_distance = distance[current]
            if current_distance >= radius_cells:
                continue
            row, column = divmod(current, self.width)
            next_distance = current_distance + 1
            for delta_x, delta_y in neighbours:
                next_column = column + delta_x
                next_row = row + delta_y
                if not (
                    0 <= next_column < self.width
                    and 0 <= next_row < self.height
                ):
                    continue
                neighbour = self.index(next_column, next_row)
                if distance[neighbour] > next_distance:
                    distance[neighbour] = next_distance
                    queue.append(neighbour)
        return bytearray(
            1
            if self.free_mask[index] and distance[index] > radius_cells
            else 0
            for index in range(size)
        )

    def reachable_mask(
        self,
        start_x: float,
        start_y: float,
        clearance_m: float,
    ) -> bytearray:
        traversable = self._inflated_free_mask(clearance_m)
        start = self.world_to_cell(start_x, start_y)
        if start is None:
            raise ValueError("시작 위치가 지도 범위 밖입니다")
        start_index = self.index(*start)
        if not traversable[start_index]:
            raise ValueError("시작 위치가 순찰 가능한 free 셀이 아닙니다")
        reachable = bytearray(self.width * self.height)
        reachable[start_index] = 1
        queue: deque[int] = deque([start_index])
        neighbours = ((-1, 0), (1, 0), (0, -1), (0, 1))
        while queue:
            current = queue.popleft()
            row, column = divmod(current, self.width)
            for delta_x, delta_y in neighbours:
                next_column = column + delta_x
                next_row = row + delta_y
                if not (
                    0 <= next_column < self.width
                    and 0 <= next_row < self.height
                ):
                    continue
                neighbour = self.index(next_column, next_row)
                if traversable[neighbour] and not reachable[neighbour]:
                    reachable[neighbour] = 1
                    queue.append(neighbour)
        return reachable

    def coverage(
        self,
        trajectory: Iterable[PoseSample],
        start_x: float,
        start_y: float,
        inspection_radius_m: float,
        clearance_m: float = 0.0,
    ) -> CoverageResult:
        samples = list(trajectory)
        reachable = self.reachable_mask(start_x, start_y, clearance_m)
        observed = bytearray(self.width * self.height)
        radius_cells = max(0, math.ceil(inspection_radius_m / self.resolution))
        disk = [
            (delta_x, delta_y)
            for delta_y in range(-radius_cells, radius_cells + 1)
            for delta_x in range(-radius_cells, radius_cells + 1)
            if math.hypot(delta_x, delta_y) * self.resolution
            <= inspection_radius_m
        ]
        visited_centres: set[int] = set()

        def mark(x: float, y: float) -> None:
            cell = self.world_to_cell(x, y)
            if cell is None:
                return
            column, row = cell
            centre_index = self.index(column, row)
            if centre_index in visited_centres:
                return
            visited_centres.add(centre_index)
            for delta_x, delta_y in disk:
                target_column = column + delta_x
                target_row = row + delta_y
                if not (
                    0 <= target_column < self.width
                    and 0 <= target_row < self.height
                ):
                    continue
                index = self.index(target_column, target_row)
                if reachable[index]:
                    observed[index] = 1

        for index, sample in enumerate(samples):
            mark(sample.x, sample.y)
            if index == 0:
                continue
            previous = samples[index - 1]
            length = math.hypot(sample.x - previous.x, sample.y - previous.y)
            steps = max(1, math.ceil(length / max(self.resolution / 2.0, 0.001)))
            for step in range(1, steps):
                ratio = step / steps
                mark(
                    previous.x + (sample.x - previous.x) * ratio,
                    previous.y + (sample.y - previous.y) * ratio,
                )

        map_cells = self.width * self.height
        free_cells = int(sum(self.free_mask))
        patrolable_cells = int(sum(reachable))
        observed_cells = int(sum(observed))
        coverage_percent = (
            observed_cells / patrolable_cells * 100.0
            if patrolable_cells
            else 0.0
        )
        return CoverageResult(
            map_area_m2=map_cells * self.cell_area_m2,
            free_area_m2=free_cells * self.cell_area_m2,
            patrolable_area_m2=patrolable_cells * self.cell_area_m2,
            observed_area_m2=observed_cells * self.cell_area_m2,
            coverage_percent=coverage_percent,
            map_cell_count=map_cells,
            free_cell_count=free_cells,
            patrolable_cell_count=patrolable_cells,
            observed_cell_count=observed_cells,
        )
