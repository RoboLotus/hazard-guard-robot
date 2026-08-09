"""Generate the classroom demo world from the original facility layout.

The layout is NOT redesigned. Every model keeps the pose it has in
RoboLotus/gazebo-simulator (all at the common world origin, no rotation), and
the whole plant - hall shell included - is shrunk by one uniform scale so it
fits the demo room. The scale is the largest that still fits, since corridor
widths shrink with it.

Note what that costs: the original plant runs wall to wall from the bunker to
the bale storage, so there is no circulation loop at any scale. The patrol
route is the open south aisle, driven there and back. Preserving the original
arrangement and having a loop are mutually exclusive.

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
MODELS = PACKAGE / "models"
OUT_WORLD = PACKAGE / "worlds" / "demo_facility.sdf"
OUT_PLAN = REPO / "runtime" / "debug" / "demo_floorplan.png"

# The demo room. Fixed input - measured from the actual space.
HALL_X, HALL_Y = 8.0, 6.0

# ROSMASTER M1: nav footprint 0.31 m wide, nav2 inflation_radius 0.35 m.
ROBOT_W, INFLATION = 0.31, 0.35
NEED = ROBOT_W / 2 + INFLATION          # 1.01 m corridor, hard minimum

CELL = 0.02
Z_LO, Z_HI = 0.05, 2.5

# Exactly the include list of gazebo-simulator/worlds/recycling_facility.sdf,
# in the same order. Poses stay at the origin - do not add offsets or yaw here.
ITEMS = ["hall_shell", "bunker", "primary_shredder", "sorting_line",
         "secondary_processor", "baler", "bale_storage", "control_room"]


def load(name):
    verts, faces = [], []
    for line in open(MODELS / name / "meshes" / f"{name}.obj"):
        if line.startswith("v "):
            verts.append([float(v) for v in line.split()[1:4]])
        elif line.startswith("f "):
            idx = [int(t.split("/")[0]) for t in line.split()[1:]]
            faces += [(idx[0], idx[i], idx[i + 1])
                      for i in range(1, len(idx) - 1)]
    return np.array(verts), faces


MESH = {name: load(name) for name in ITEMS}

# Largest uniform scale that fits the whole plant, hall shell included, into
# the room. Corridors scale with it, so bigger is strictly better here.
span_x = max(v[:, 0].max() for v, _ in MESH.values()) - \
    min(v[:, 0].min() for v, _ in MESH.values())
span_y = max(v[:, 1].max() for v, _ in MESH.values()) - \
    min(v[:, 1].min() for v, _ in MESH.values())
SCALE = min(HALL_X / span_x, HALL_Y / span_y)

print(f"원본 시설  {span_x:.2f} x {span_y:.2f} m")
print(f"데모 방    {HALL_X} x {HALL_Y} m")
print(f"균등 축척  {SCALE:.5f}  ->  시설 "
      f"{span_x*SCALE:.2f} x {span_y*SCALE:.2f} m")
print("모든 모델은 원본과 동일하게 원점 배치, 회전 없음\n")

# ---- rasterise ------------------------------------------------------------
nx, ny = int(HALL_X / CELL), int(HALL_Y / CELL)
occ = np.zeros((ny, nx), bool)
for name in ITEMS:
    verts, faces = MESH[name]
    v = verts * SCALE
    for a, b, c in faces:
        tri = v[[a - 1, b - 1, c - 1]]
        if tri[:, 2].max() < Z_LO or tri[:, 2].min() > Z_HI:
            continue
        x0 = max(int((tri[:, 0].min() + HALL_X / 2) / CELL), 0)
        x1 = max(int(np.ceil((tri[:, 0].max() + HALL_X / 2) / CELL)), 0)
        y0 = max(int((tri[:, 1].min() + HALL_Y / 2) / CELL), 0)
        y1 = max(int(np.ceil((tri[:, 1].max() + HALL_Y / 2) / CELL)), 0)
        occ[y0:y1, x0:x1] = True
occ[0, :] = occ[-1, :] = True
occ[:, 0] = occ[:, -1] = True

clear = ndimage.distance_transform_edt(~occ) * CELL
drivable = clear >= NEED
lab, n = ndimage.label(drivable)
sizes = ndimage.sum(drivable, lab, range(1, n + 1)) if n else np.array([0])
comp = (lab == (sizes.argmax() + 1)) if n else drivable
loop = ndimage.binary_fill_holes(comp).sum() > comp.sum() * 1.15

iys, ixs = np.where(comp)
wx = ixs * CELL - HALL_X / 2 + CELL / 2
wy = iys * CELL - HALL_Y / 2 + CELL / 2

print(f"주행 가능 면적 {comp.sum()*CELL**2:.2f} m^2 "
      f"(방의 {comp.sum()/(nx*ny)*100:.0f}%)")
print(f"주행 범위      x {wx.min():.2f} .. {wx.max():.2f} m, "
      f"y {wy.min():.2f} .. {wy.max():.2f} m")
print(f"최대 통로 폭   {clear[comp].max()*2:.2f} m "
      f"(M1 최소 요구 {NEED*2:.2f} m)")
print(f"순환로         {'있음' if loop else '없음 - 남측 통로 왕복'}\n")

print(f"{'설비':<22}{'가로':>9}{'세로':>9}{'높이':>9}"
      f"{'중심 X':>10}{'중심 Y':>10}")
print("-" * 70)
for name in ITEMS:
    v = MESH[name][0] * SCALE
    print(f"{name:<22}{v[:,0].ptp():>7.2f} m{v[:,1].ptp():>7.2f} m"
          f"{v[:,2].ptp():>7.2f} m"
          f"{(v[:,0].min()+v[:,0].max())/2:>9.2f} m"
          f"{(v[:,1].min()+v[:,1].max())/2:>9.2f} m")

# ---- patrol waypoints along the open aisle --------------------------------
def nearest_drivable(target):
    k = ((wx - target[0]) ** 2 + (wy - target[1]) ** 2).argmin()
    return (wx[k], wy[k]), clear[iys[k], ixs[k]]


aisle_y = wy[np.argmax(clear[comp])] if comp.any() else 0.0
WAYPOINTS = {}
for i, frac in enumerate((0.10, 0.37, 0.63, 0.90)):
    x = wx.min() + (wx.max() - wx.min()) * frac
    WAYPOINTS[f"P{i+1}"] = nearest_drivable((x, aisle_y))

print("\n순찰 웨이포인트 (남측 통로 서->동, 왕복)")
print("-" * 46)
for label, ((px, py), c) in WAYPOINTS.items():
    print(f"{label:<6} x={px:>6.2f}  y={py:>6.2f}   여유 {c*2:.2f} m")
# Spawn on the roomiest waypoint, not the first one. P1 sits at the west end
# of the aisle where clearance is only just above the M1 minimum.
(sx, sy), sc = max(WAYPOINTS.values(), key=lambda w: w[1])
print(f"\n권장 스폰 위치: x={sx:.2f}  y={sy:.2f}  yaw=0  (여유 {sc*2:.2f} m)")

# ---- floor plan -----------------------------------------------------------
img = np.full(occ.shape + (3,), 250, np.uint8)
img[drivable] = (200, 232, 205)
img[comp] = (120, 205, 140)
img[occ] = (60, 60, 70)
OUT_PLAN.parent.mkdir(parents=True, exist_ok=True)
Image.fromarray(np.flipud(img)).resize((nx * 2, ny * 2), Image.NEAREST).save(OUT_PLAN)

# ---- world SDF ------------------------------------------------------------
parts = ["""<?xml version='1.0' encoding='utf-8'?>
<!--
  HazardGuard classroom demo world. GENERATED by tools/gen_demo_world.py
  - do not hand-edit; change HALL_X/HALL_Y in that script and re-run.

  Layout is the RoboLotus/gazebo-simulator recycling facility, unmodified:
  the same eight models in the same include order, every one at the common
  world origin with no rotation. Only the uniform scale differs - {s:.5f},
  the largest that fits the {sx:.2f} x {sy:.2f} m plant into a {hx} x {hy} m
  room.

  The Fortress conversion is limited to three things the original could not
  provide: ignition-gazebo-* system plugin names instead of gz-sim-*, an IMU
  system for the robot IMU, and the ODE tuning the earlier worlds relied on.

  The plant runs wall to wall, so there is no circulation loop at any scale.
  The patrol route is the south aisle, driven there and back.
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
    <light type="point" name="hall_fill_west">
      <pose>{lw:.2f} 0 {lz:.2f} 0 0 0</pose>
      <diffuse>0.6 0.6 0.6 1</diffuse>
      <attenuation><range>{lr:.1f}</range><linear>0.05</linear>
        <quadratic>0.02</quadratic></attenuation>
    </light>
    <light type="point" name="hall_fill_east">
      <pose>{le:.2f} 0 {lz:.2f} 0 0 0</pose>
      <diffuse>0.6 0.6 0.6 1</diffuse>
      <attenuation><range>{lr:.1f}</range><linear>0.05</linear>
        <quadratic>0.02</quadratic></attenuation>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal>
            <size>{gx} {gy}</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal>
            <size>{gx} {gy}</size></plane></geometry>
          <material><ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.35 0.35 0.35 1</diffuse></material>
        </visual>
      </link>
    </model>
""".format(s=SCALE, sx=span_x, sy=span_y, hx=HALL_X, hy=HALL_Y,
           lw=-15 * SCALE, le=15 * SCALE, lz=10 * SCALE, lr=40 * SCALE,
           gx=HALL_X + 4, gy=HALL_Y + 4)]

for name in ITEMS:
    uri = f"model://{name}/meshes/{name}.obj"
    parts.append(f"""
    <model name="{name}">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><mesh><uri>{uri}</uri>
            <scale>{SCALE:.5f} {SCALE:.5f} {SCALE:.5f}</scale></mesh></geometry>
        </visual>
        <collision name="collision">
          <geometry><mesh><uri>{uri}</uri>
            <scale>{SCALE:.5f} {SCALE:.5f} {SCALE:.5f}</scale></mesh></geometry>
        </collision>
      </link>
    </model>""")

parts.append("\n  </world>\n</sdf>\n")
OUT_WORLD.write_text("".join(parts))
print(f"\n월드 생성: {OUT_WORLD}")
print(f"평면도    : {OUT_PLAN}")
