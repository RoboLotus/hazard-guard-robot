"""Generate the classroom demo world and verify it is actually drivable.

The full-scale facility layout runs equipment wall-to-wall, so a uniform shrink
of the original arrangement leaves no circulation loop. This script instead
places the equipment in three bands - a south wall row, a centre island, and a
north wall row - so the robot has a closed patrol loop around the island, then
proves the result with a clearance/connectivity check.

All seven plant items are included. To fit them the equipment scale is not
fixed: SCALE is searched downward until the loop bottleneck reaches
TARGET_CORRIDOR. The room dimensions are the fixed input, since they come from
the actual demo space.

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
WALL_H, WALL_T = 1.0, 0.05

# ROSMASTER M1: nav footprint 0.31 m wide, nav2 inflation_radius 0.35 m.
ROBOT_W, INFLATION = 0.31, 0.35
NEED = ROBOT_W / 2 + INFLATION          # 1.01 m corridor, hard minimum
TARGET_CORRIDOR = 1.20                  # what we actually aim for

SCALE_MAX, SCALE_MIN, SCALE_STEP = 0.1333, 0.060, 0.0025

CELL = 0.02
Z_LO, Z_HI = 0.05, 2.5

# Plant items by band, in west-to-east order. The island is what the patrol
# loop wraps around; the two wall rows sit flush against the room walls so they
# never pinch the loop. Yaw turns a machine broadside to spend the room's 6 m
# depth on corridors rather than on equipment.
ISLAND = [("sorting_line", 0), ("baler", 90)]
SOUTH = [("control_room", 0), ("primary_shredder", 90), ("bale_storage", 90)]
NORTH = [("bunker", 90), ("secondary_processor", 90)]
ALL_ITEMS = ISLAND + SOUTH + NORTH


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


MESH = {name: load(name) for name, _ in ALL_ITEMS}


def oriented(name, yaw, scale):
    v = MESH[name][0] * scale
    a = math.radians(yaw)
    rot = np.array([[math.cos(a), -math.sin(a), 0],
                    [math.sin(a), math.cos(a), 0],
                    [0, 0, 1]])
    return v @ rot.T


def footprint(name, yaw, scale):
    v = oriented(name, yaw, scale)
    return v[:, 0].max() - v[:, 0].min(), v[:, 1].max() - v[:, 1].min()


def build_layout(scale):
    """Centre the island in the depth left over by the two wall rows."""
    depth = {
        "south": max(footprint(n, y, scale)[1] for n, y in SOUTH),
        "north": max(footprint(n, y, scale)[1] for n, y in NORTH),
        "island": max(footprint(n, y, scale)[1] for n, y in ISLAND),
    }
    gap = (HALL_Y - depth["south"] - depth["north"] - depth["island"]) / 2
    y_island = -HALL_Y / 2 + depth["south"] + gap + depth["island"] / 2

    layout = {}
    for row, y_centre in (
        (ISLAND, y_island),
        (SOUTH, None),
        (NORTH, None),
    ):
        total = sum(footprint(n, y, scale)[0] for n, y in row)
        cursor = -total / 2
        for name, yaw in row:
            w, d = footprint(name, yaw, scale)
            if y_centre is not None:
                cy = y_centre
            elif row is SOUTH:
                cy = -HALL_Y / 2 + d / 2
            else:
                cy = HALL_Y / 2 - d / 2
            layout[name] = (cursor + w / 2, cy, yaw)
            cursor += w
    return layout, gap


def place(name, yaw, target_xy, scale):
    v = oriented(name, yaw, scale)
    centre = np.array([(v[:, 0].min() + v[:, 0].max()) / 2,
                       (v[:, 1].min() + v[:, 1].max()) / 2, 0.0])
    offset = np.array([target_xy[0], target_xy[1], 0.0]) - centre
    return offset, v + offset


def occupancy(layout, scale):
    nx, ny = int(HALL_X / CELL), int(HALL_Y / CELL)
    occ = np.zeros((ny, nx), bool)
    placed = {}
    for name, (tx, ty, yaw) in layout.items():
        offset, moved = place(name, yaw, (tx, ty), scale)
        placed[name] = (offset, yaw, moved)
        for a, b, c in MESH[name][1]:
            tri = moved[[a - 1, b - 1, c - 1]]
            if tri[:, 2].max() < Z_LO or tri[:, 2].min() > Z_HI:
                continue
            x0 = max(int((tri[:, 0].min() + HALL_X / 2) / CELL), 0)
            x1 = max(int(np.ceil((tri[:, 0].max() + HALL_X / 2) / CELL)), 0)
            y0 = max(int((tri[:, 1].min() + HALL_Y / 2) / CELL), 0)
            y1 = max(int(np.ceil((tri[:, 1].max() + HALL_Y / 2) / CELL)), 0)
            occ[y0:y1, x0:x1] = True
    # The room walls bound the space. Without them the distance transform
    # treats everything past the grid edge as open and overstates the clearance
    # of every wall-hugging corridor - which is where the loop runs.
    occ[0, :] = occ[-1, :] = True
    occ[:, 0] = occ[:, -1] = True
    return occ, placed


def biggest(mask):
    lab, n = ndimage.label(mask)
    if n == 0:
        return mask & False
    sizes = ndimage.sum(mask, lab, range(1, n + 1))
    return lab == (sizes.argmax() + 1)


def has_loop(mask):
    comp = biggest(mask)
    if not comp.any():
        return False
    return ndimage.binary_fill_holes(comp).sum() > comp.sum() * 1.15


def bottleneck_of(clear):
    """Widest corridor threshold that still leaves a loop. Reporting the
    minimum clearance of the drivable set would just echo NEED, since that set
    is defined by NEED."""
    if not has_loop(clear >= NEED):
        return 0.0
    best, r = NEED, NEED
    while r < 2.0 and has_loop(clear >= r):
        best, r = r, r + 0.01
    return best * 2


def evaluate(scale):
    layout, gap = build_layout(scale)
    if gap <= 0:
        return None
    occ, placed = occupancy(layout, scale)
    clear = ndimage.distance_transform_edt(~occ) * CELL
    return {
        "scale": scale, "layout": layout, "gap": gap, "occ": occ,
        "placed": placed, "clear": clear,
        "bottleneck": bottleneck_of(clear),
    }


# ---- pick the largest scale that keeps the loop comfortable ---------------
print(f"방 크기 {HALL_X} x {HALL_Y} m, 설비 {len(ALL_ITEMS)}종 전부 배치")
print(f"목표 순환로 폭 {TARGET_CORRIDOR:.2f} m (M1 최소 {NEED*2:.2f} m)\n")
print(f"{'축척':>8}{'설계 통로':>12}{'순환로 병목폭':>16}")
print("-" * 38)

best = None
scale = SCALE_MAX
while scale >= SCALE_MIN:
    result = evaluate(scale)
    if result:
        mark = ""
        if result["bottleneck"] >= TARGET_CORRIDOR and best is None:
            best, mark = result, "  <- 채택"
        print(f"{scale:>8.4f}{result['gap']:>10.2f} m"
              f"{result['bottleneck']:>13.2f} m{mark}")
    scale -= SCALE_STEP

if best is None:
    raise SystemExit("목표 통로 폭을 만족하는 축척이 없습니다.")

SCALE = best["scale"]
layout, placed, occ, clear = (best["layout"], best["placed"],
                              best["occ"], best["clear"])
drivable = clear >= NEED
comp = biggest(drivable)

print(f"\n채택 축척 {SCALE:.4f}  (원본 60 m 홀 -> {60*SCALE:.1f} m 상당)")
print(f"주행 가능 면적 {comp.sum()*CELL**2:.2f} m^2, "
      f"순환로 병목폭 {best['bottleneck']:.2f} m")

print(f"\n{'설비':<22}{'밴드':>6}{'회전':>6}{'가로':>8}{'세로':>8}"
      f"{'높이':>8}{'중심 X':>9}{'중심 Y':>9}")
print("-" * 78)
band_of = {n: b for b, row in
           (("중앙", ISLAND), ("남측", SOUTH), ("북측", NORTH)) for n, _ in row}
for name, (tx, ty, yaw) in layout.items():
    moved = placed[name][2]
    print(f"{name:<22}{band_of[name]:>6}{yaw:>5}°"
          f"{moved[:,0].ptp():>7.2f}m{moved[:,1].ptp():>7.2f}m"
          f"{moved[:,2].ptp():>7.2f}m{tx:>8.2f}m{ty:>8.2f}m")


# ---- patrol waypoints -----------------------------------------------------
def nearest_drivable(target):
    iys, ixs = np.where(comp)
    wx = ixs * CELL - HALL_X / 2 + CELL / 2
    wy = iys * CELL - HALL_Y / 2 + CELL / 2
    k = ((wx - target[0]) ** 2 + (wy - target[1]) ** 2).argmin()
    return (wx[k], wy[k]), clear[iys[k], ixs[k]]


isl_x = [v for n in (n for n, _ in ISLAND)
         for v in (placed[n][2][:, 0].min(), placed[n][2][:, 0].max())]
isl_y = [v for n in (n for n, _ in ISLAND)
         for v in (placed[n][2][:, 1].min(), placed[n][2][:, 1].max())]
ix_lo, ix_hi, iy_lo, iy_hi = min(isl_x), max(isl_x), min(isl_y), max(isl_y)
cx, cy = (ix_lo + ix_hi) / 2, (iy_lo + iy_hi) / 2

WAYPOINTS = {
    "북 (벙커 앞)": nearest_drivable((cx, (iy_hi + HALL_Y / 2) / 2)),
    "동 (압축기 옆)": nearest_drivable(((ix_hi + HALL_X / 2) / 2, cy)),
    "남 (파쇄기 앞)": nearest_drivable((cx, (iy_lo - HALL_Y / 2) / 2)),
    "서 (관리실 앞)": nearest_drivable(((ix_lo - HALL_X / 2) / 2, cy)),
}
print("\n순찰 웨이포인트 (섬을 도는 4점)")
print("-" * 46)
for label, ((wx, wy), c) in WAYPOINTS.items():
    print(f"{label:<18} x={wx:>6.2f}  y={wy:>6.2f}   여유 {c*2:.2f} m")
(sx, sy), _ = WAYPOINTS["남 (파쇄기 앞)"]
print(f"\n권장 스폰 위치: x={sx:.2f}  y={sy:.2f}  yaw=0")

# ---- floor plan -----------------------------------------------------------
img = np.full(occ.shape + (3,), 250, np.uint8)
img[drivable] = (200, 232, 205)
img[comp] = (120, 205, 140)
img[occ] = (60, 60, 70)
OUT_PLAN.parent.mkdir(parents=True, exist_ok=True)
nx, ny = occ.shape[1], occ.shape[0]
Image.fromarray(np.flipud(img)).resize((nx * 2, ny * 2), Image.NEAREST).save(OUT_PLAN)

# ---- world SDF ------------------------------------------------------------
walls = [
    ("wall_north", 0, HALL_Y / 2, HALL_X + 2 * WALL_T, WALL_T),
    ("wall_south", 0, -HALL_Y / 2, HALL_X + 2 * WALL_T, WALL_T),
    ("wall_east", HALL_X / 2, 0, WALL_T, HALL_Y),
    ("wall_west", -HALL_X / 2, 0, WALL_T, HALL_Y),
]

parts = ["""<?xml version='1.0' encoding='utf-8'?>
<!--
  HazardGuard classroom demo world. GENERATED by tools/gen_demo_world.py
  - do not hand-edit; change the band lists in that script and re-run.

  All seven plant items scaled to {s:.4f} and arranged in three bands inside a
  {hx} x {hy} m room, so the robot has a closed patrol loop around the centre
  island. A literal uniform shrink of the full-scale layout does not give one:
  there the plant runs wall to wall and only a there-and-back corridor
  survives. Loop bottleneck is {b:.2f} m against the M1 minimum of {n:.2f} m.
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
""".format(s=SCALE, hx=HALL_X, hy=HALL_Y, b=best["bottleneck"], n=NEED * 2,
           fx=HALL_X + 4, fy=HALL_Y + 4)]

for wname, wx, wy, sx_, sy_ in walls:
    parts.append(f"""
    <model name="{wname}">
      <static>true</static>
      <pose>{wx:.3f} {wy:.3f} {WALL_H/2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx_:.3f} {sy_:.3f} {WALL_H:.3f}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{sx_:.3f} {sy_:.3f} {WALL_H:.3f}</size></box></geometry>
          <material><ambient>0.62 0.62 0.64 1</ambient>
            <diffuse>0.72 0.72 0.74 1</diffuse></material>
        </visual>
      </link>
    </model>""")

for name, (offset, yaw, _) in placed.items():
    uri = f"model://{name}/meshes/{name}.obj"
    parts.append(f"""
    <model name="{name}">
      <static>true</static>
      <pose>{offset[0]:.4f} {offset[1]:.4f} 0 0 0 {math.radians(yaw):.6f}</pose>
      <link name="link">
        <visual name="visual">
          <geometry><mesh><uri>{uri}</uri>
            <scale>{SCALE:.4f} {SCALE:.4f} {SCALE:.4f}</scale></mesh></geometry>
        </visual>
        <collision name="collision">
          <geometry><mesh><uri>{uri}</uri>
            <scale>{SCALE:.4f} {SCALE:.4f} {SCALE:.4f}</scale></mesh></geometry>
        </collision>
      </link>
    </model>""")

parts.append("\n  </world>\n</sdf>\n")
OUT_WORLD.write_text("".join(parts))
print(f"\n월드 생성: {OUT_WORLD}")
print(f"평면도    : {OUT_PLAN}")
