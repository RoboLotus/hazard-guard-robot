"""Generate the 8 m classroom demo world and verify it is actually drivable.

The full-scale facility layout runs equipment wall-to-wall, so a uniform shrink
leaves no circulation loop. This script instead scales the equipment meshes to
demo size and re-places a subset of them around a central island, then proves
the result with the same clearance/connectivity check used on the full world.

Requires numpy, pillow and scipy. Run it from anywhere; paths resolve
relative to this file, not to the container workspace.
"""
import math
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage

REPO = Path(__file__).resolve().parent.parent
PACKAGE = REPO / "src" / "hazard_guard_simulation"
MODELS = str(PACKAGE / "models")
OUT_WORLD = str(PACKAGE / "worlds" / "demo_facility.sdf")
OUT_PLAN = str(REPO / "runtime" / "debug" / "demo_floorplan.png")

SCALE = 0.1333          # 60 m facility -> 8 m classroom
HALL_X, HALL_Y = 8.0, 6.0
WALL_H, WALL_T = 1.0, 0.05

CELL = 0.02
Z_LO, Z_HI = 0.05, 2.5
ROBOT_W = 0.31
INFLATION = 0.35
NEED = ROBOT_W / 2 + INFLATION     # 0.505 m half-width -> 1.01 m corridor

# name -> (target centre x, y, yaw degrees)
# Centre island (sorting_line + baler) with the loop running around it.
# The shredder is turned broadside so the room's 6 m depth splits into two
# ~1.4 m corridors instead of two ~1.2 m ones.
LAYOUT = {
    "sorting_line":       (-1.25, 0.74, 0),
    "baler":              (1.35, 0.74, 90),
    "primary_shredder":   (-0.30, -2.26, 90),
    "bale_storage":       (2.85, -2.64, 90),
    "control_room":       (-3.00, -2.56, 0),
}


def load_model(name):
    path = f"{MODELS}/{name}/meshes/{name}.obj"
    verts, faces = [], []
    for line in open(path):
        if line.startswith("v "):
            verts.append([float(v) for v in line.split()[1:4]])
        elif line.startswith("f "):
            idx = [int(t.split("/")[0]) for t in line.split()[1:]]
            for i in range(1, len(idx) - 1):
                faces.append((idx[0], idx[i], idx[i + 1]))
    return np.array(verts), faces


def place(verts, yaw_deg, target_xy):
    """Scale, rotate about the world origin, then translate so the footprint
    centre lands on target_xy. Returns (offset, transformed vertices)."""
    v = verts * SCALE
    a = math.radians(yaw_deg)
    rot = np.array([[math.cos(a), -math.sin(a), 0],
                    [math.sin(a), math.cos(a), 0],
                    [0, 0, 1]])
    v = v @ rot.T
    centre = np.array([(v[:, 0].min() + v[:, 0].max()) / 2,
                       (v[:, 1].min() + v[:, 1].max()) / 2])
    offset = np.array([target_xy[0] - centre[0], target_xy[1] - centre[1], 0.0])
    return offset, v + offset


placed, spec = {}, []
for name, (tx, ty, yaw) in LAYOUT.items():
    verts, faces = load_model(name)
    offset, moved = place(verts, yaw, (tx, ty))
    placed[name] = (offset, yaw, moved, faces)
    spec.append((name, yaw,
                 moved[:, 0].max() - moved[:, 0].min(),
                 moved[:, 1].max() - moved[:, 1].min(),
                 moved[:, 2].max() - moved[:, 2].min(), tx, ty))

# ---- rasterise and verify -------------------------------------------------
nx = int(HALL_X / CELL)
ny = int(HALL_Y / CELL)
occ = np.zeros((ny, nx), bool)


def stamp(tri):
    if tri[:, 2].max() < Z_LO or tri[:, 2].min() > Z_HI:
        return
    x0 = int((tri[:, 0].min() + HALL_X / 2) / CELL)
    x1 = int(np.ceil((tri[:, 0].max() + HALL_X / 2) / CELL))
    y0 = int((tri[:, 1].min() + HALL_Y / 2) / CELL)
    y1 = int(np.ceil((tri[:, 1].max() + HALL_Y / 2) / CELL))
    occ[max(y0, 0):max(y1, 0), max(x0, 0):max(x1, 0)] = True


for name, (offset, yaw, moved, faces) in placed.items():
    for a, b, c in faces:
        stamp(moved[[a - 1, b - 1, c - 1]])

# The perimeter walls bound the room. Without them the distance transform
# treats everything past the grid edge as open and overstates the clearance of
# every wall-hugging corridor - which is exactly where the loop runs.
occ[0, :] = True
occ[-1, :] = True
occ[:, 0] = True
occ[:, -1] = True

free = ~occ
clear = ndimage.distance_transform_edt(free) * CELL
drivable = clear >= NEED

def biggest_component(mask):
    lab, n = ndimage.label(mask)
    if n == 0:
        return mask & False
    sizes = ndimage.sum(mask, lab, range(1, n + 1))
    return lab == (sizes.argmax() + 1)


def has_loop(mask):
    comp = biggest_component(mask)
    if not comp.any():
        return False, comp
    filled = ndimage.binary_fill_holes(comp)
    return filled.sum() > comp.sum() * 1.15, comp


comp = biggest_component(drivable)
loop, _ = has_loop(drivable)

# True bottleneck: the widest corridor threshold that still leaves a loop.
# Reporting clear[comp].min() would just echo NEED, since comp is defined by it.
bottleneck = 0.0
r = NEED
while r < 2.0:
    ok, _ = has_loop(clear >= r)
    if not ok:
        break
    bottleneck = r
    r += 0.01

print(f"데모 홀      : {HALL_X} x {HALL_Y} m   (배율 {SCALE}, 원본 60 x 35 m)")
print(f"설비 수      : {len(LAYOUT)}개")
print(f"주행 가능 면적: {comp.sum() * CELL**2:.2f} m^2 "
      f"(홀의 {comp.sum() / (nx*ny) * 100:.0f}%)")
print(f"순환로       : {'있음 (한 바퀴 주행 가능)' if loop else '없음 (왕복만)'}")
print(f"순환로 병목폭 : {bottleneck*2:.2f} m "
      f"(M1 최소 요구 {NEED*2:.2f} m, 권장 1.20 m 이상)")
print(f"최대 통로 폭  : {clear[comp].max()*2:.2f} m")

def to_world(iy, ix):
    return (ix * CELL - HALL_X / 2 + CELL / 2,
            iy * CELL - HALL_Y / 2 + CELL / 2)


def nearest_drivable(target):
    """Snap a desired corridor mid-point onto the drivable ring. Picking the
    'most open cell in a window' instead drags every waypoint into a corner,
    which collapses the four patrol points onto one side of the island."""
    iys, ixs = np.where(comp)
    wx = ixs * CELL - HALL_X / 2 + CELL / 2
    wy = iys * CELL - HALL_Y / 2 + CELL / 2
    d = (wx - target[0]) ** 2 + (wy - target[1]) ** 2
    k = d.argmin()
    return (wx[k], wy[k]), clear[iys[k], ixs[k]]


# Island = the equipment cluster the loop wraps around.
isl = np.zeros_like(occ)
for nm in ("sorting_line", "baler"):
    mv = placed[nm][2]
    ix0 = int((mv[:, 0].min() + HALL_X / 2) / CELL)
    ix1 = int((mv[:, 0].max() + HALL_X / 2) / CELL)
    iy0 = int((mv[:, 1].min() + HALL_Y / 2) / CELL)
    iy1 = int((mv[:, 1].max() + HALL_Y / 2) / CELL)
    isl[iy0:iy1, ix0:ix1] = True
iys, ixs = np.where(isl)
ix_lo, ix_hi = to_world(0, ixs.min())[0], to_world(0, ixs.max())[0]
iy_lo, iy_hi = to_world(iys.min(), 0)[1], to_world(iys.max(), 0)[1]

cx, cy = (ix_lo + ix_hi) / 2, (iy_lo + iy_hi) / 2
corners = {
    "북 (선별라인 뒤)": nearest_drivable((cx, (iy_hi + HALL_Y / 2) / 2)),
    "동 (압축기 옆)": nearest_drivable(((ix_hi + HALL_X / 2) / 2, cy)),
    "남 (파쇄기 앞)": nearest_drivable((cx, (iy_lo - HALL_Y / 2) / 2)),
    "서 (반입구)": nearest_drivable(((ix_lo - HALL_X / 2) / 2, cy)),
}
print("\n순찰 웨이포인트 (섬을 도는 4점)")
print("-" * 44)
for label, val in corners.items():
    if val is None:
        print(f"{label:<18} 확보 실패")
        continue
    (wx, wy), c = val
    print(f"{label:<18} x={wx:>6.2f}  y={wy:>6.2f}   여유 {c*2:.2f} m")

spawn = corners["남 (파쇄기 앞)"]
if spawn:
    print(f"\n권장 스폰 위치: x={spawn[0][0]:.2f}  y={spawn[0][1]:.2f}  yaw=0")

print()
print(f"{'설비':<20}{'회전':>5}{'가로':>8}{'세로':>8}{'높이':>8}"
      f"{'중심 X':>9}{'중심 Y':>9}")
print("-" * 68)
for name, yaw, dx, dy, dz, tx, ty in spec:
    print(f"{name:<20}{yaw:>4}°{dx:>7.2f}m{dy:>7.2f}m{dz:>7.2f}m"
          f"{tx:>8.2f}m{ty:>8.2f}m")

# ---- floor plan -----------------------------------------------------------
img = np.full(occ.shape + (3,), 250, np.uint8)
img[drivable] = (200, 232, 205)
img[comp] = (120, 205, 140)
img[occ] = (60, 60, 70)
Path(OUT_PLAN).parent.mkdir(parents=True, exist_ok=True)
Image.fromarray(np.flipud(img)).resize((nx * 2, ny * 2), Image.NEAREST).save(OUT_PLAN)

# ---- world SDF ------------------------------------------------------------
walls = [
    ("wall_north", 0, HALL_Y / 2, HALL_X + 2 * WALL_T, WALL_T),
    ("wall_south", 0, -HALL_Y / 2, HALL_X + 2 * WALL_T, WALL_T),
    ("wall_east", HALL_X / 2, 0, WALL_T, HALL_Y),
    ("wall_west", -HALL_X / 2, 0, WALL_T, HALL_Y),
]

parts = ['''<?xml version='1.0' encoding='utf-8'?>
<!--
  HazardGuard classroom demo world. GENERATED by tools/gen_demo_world.py
  - do not hand-edit; change LAYOUT in that script and re-run.

  The recycling facility equipment is scaled to {s} (60 m hall -> {hx} m room)
  and re-placed around a central island so the robot has a closed patrol loop.
  A literal uniform shrink of the full-scale layout does not: there the plant
  runs wall to wall and only a there-and-back corridor survives.
-->
<sdf version="1.7">
  <world name="demo_facility">
    <plugin filename="ignition-gazebo-physics-system"
      name="ignition::gazebo::systems::Physics" />
    <plugin filename="ignition-gazebo-user-commands-system"
      name="ignition::gazebo::systems::UserCommands" />
    <plugin filename="ignition-gazebo-scene-broadcaster-system"
      name="ignition::gazebo::systems::SceneBroadcaster" />
    <plugin filename="ignition-gazebo-sensors-system"
      name="ignition::gazebo::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="ignition-gazebo-imu-system"
      name="ignition::gazebo::systems::Imu" />

    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
      <ode>
        <solver><type>quick</type><iters>50</iters><sor>1.3</sor></solver>
        <constraints>
          <cfm>0.0</cfm><erp>0.2</erp>
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
    </physics>
    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.72 0.78 0.84 1</background>
      <shadows>true</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 6 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.3 -0.9</direction>
    </light>
    <light type="point" name="room_fill">
      <pose>0 0 2.4 0 0 0</pose>
      <diffuse>0.55 0.55 0.55 1</diffuse>
      <attenuation><range>14</range><linear>0.06</linear>
        <quadratic>0.004</quadratic></attenuation>
    </light>

    <model name="floor">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal>
            <size>{fx} {fy}</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal>
            <size>{fx} {fy}</size></plane></geometry>
          <material><ambient>0.34 0.34 0.36 1</ambient>
            <diffuse>0.40 0.40 0.42 1</diffuse></material>
        </visual>
      </link>
    </model>
'''.format(s=SCALE, hx=HALL_X, fx=HALL_X + 4, fy=HALL_Y + 4)]

for wname, wx, wy, sx, sy in walls:
    parts.append(f'''
    <model name="{wname}">
      <static>true</static>
      <pose>{wx:.3f} {wy:.3f} {WALL_H/2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx:.3f} {sy:.3f} {WALL_H:.3f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx:.3f} {sy:.3f} {WALL_H:.3f}</size></box></geometry>
          <material><ambient>0.62 0.62 0.64 1</ambient>
            <diffuse>0.72 0.72 0.74 1</diffuse></material>
        </visual>
      </link>
    </model>''')

for name, (offset, yaw, moved, faces) in placed.items():
    uri = f"model://{name}/meshes/{name}.obj"
    parts.append(f'''
    <model name="{name}">
      <static>true</static>
      <pose>{offset[0]:.4f} {offset[1]:.4f} 0 0 0 {math.radians(yaw):.6f}</pose>
      <link name="link">
        <visual name="visual">
          <geometry><mesh><uri>{uri}</uri>
            <scale>{SCALE} {SCALE} {SCALE}</scale></mesh></geometry>
        </visual>
        <collision name="collision">
          <geometry><mesh><uri>{uri}</uri>
            <scale>{SCALE} {SCALE} {SCALE}</scale></mesh></geometry>
        </collision>
      </link>
    </model>''')

parts.append("\n  </world>\n</sdf>\n")
open(OUT_WORLD, "w").write("".join(parts))
print(f"\n월드 생성: {OUT_WORLD}")
print(f"평면도    : {OUT_PLAN}")
