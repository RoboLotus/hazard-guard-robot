"""Generate the classroom demo world from the original facility layout.

The arrangement is the RoboLotus/gazebo-simulator recycling facility: seven of
its eight models, every one at its source position and rotation relative to the
others. Nothing is moved or turned; only control_room is dropped.

Two scales, not one:

  hall_shell        SCALE          - the largest that fits the plant's
                                     60.60 x 35.60 m envelope into the room
  everything else   SCALE * SHRINK - the equipment island, shrunk as a whole
                                     and recentred in the hall

SHRINK is what buys the patrol route. At SHRINK = 1 the plant fills the hall
wall to wall and the robot has one dead-end aisle along the south side. Pulling
the island in leaves a corridor right around it, so the robot can drive a full
loop - and any U within it. Because the island is scaled and translated as a
single rigid group, every relative distance inside the plant is preserved
exactly; only the plant-to-robot ratio changes.

Measured at 6 x 4 m (hall interior 5.900 x 3.420 m). N/S is always the binding
pair - the hall is proportionally shallower than the plant, so the W/E aisle is
left wider than it needs to be:

  SHRINK   island size      W/E aisle   N/S aisle   loop?   8-waypoint drive
   0.60    3.44 x 1.77 m      1.23        0.83       no     -
   0.575   3.30 x 1.69 m      1.30        0.86       yes    -
   0.55    3.16 x 1.62 m      1.37        0.90       yes    FAILS - stuck at P6
   0.50    2.87 x 1.47 m      1.52        0.97       yes    8/8, 3 retries
   0.45    2.58 x 1.33 m      1.66        1.05       yes    8/8, clean

0.50 is chosen: the largest that actually drives. Note the last column is not
implied by the others - the loop closes geometrically at 0.575, but a corridor
being wider than the robot's turning circle is not the same as the controller
being able to work in it. 0.55 closes the loop and still traps the robot.

That gap is controller-bound, not layout-bound. It moved once config/nav2.yaml
stopped clamping the mecanum base to 0.08 m/s sideways; at 0.08 the same route
failed even at 0.45. If you want a bigger island, tune DWB further - raising
SHRINK alone will not do it.

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
HALL_X, HALL_Y = 6.0, 4.0

# ROSMASTER M1 nav footprint, from config/nav2.yaml:
# [[-0.410, -0.155], [-0.410, 0.155], [0.190, 0.155], [0.190, -0.155]]
#
# NEED is the robot's CIRCUMSCRIBED radius - the pose-independent requirement.
# A cell is usable if the robot can sit there at any heading, which it must be
# able to do: every patrol waypoint turns to face the conveyor.
#
# It is deliberately not `half_width + inflation_radius`. inflation_radius only
# shapes cost; what actually blocks the planner is the INSCRIBED radius
# (0.155 m), and what blocks the robot is the footprint polygon. Sizing aisles
# off the inflation radius understates them by ~0.2 m and, on a 1.05 m aisle,
# reports a few cm of drivable width where there is really ~0.16 m.
ROBOT_REAR, ROBOT_HALF_W = 0.410, 0.155
NEED = math.hypot(ROBOT_REAR, ROBOT_HALF_W)   # 0.438 m -> 0.877 m corridor

CELL = 0.02
Z_LO, Z_HI = 0.05, 2.5

# The include list of gazebo-simulator/worlds/recycling_facility.sdf in the
# same order, minus control_room. Every item keeps its source pose relative to
# the rest.
#
# control_room is dropped because it is the plant's southern extreme (source y
# -16.8) and a manned office is not a patrol target. Without it the island is
# 2.948 m deep instead of 3.188 m, and since depth is what limits SHRINK that
# alone lets every remaining machine be ~8% larger.
ITEMS = ["hall_shell", "bunker", "primary_shredder", "sorting_line",
         "secondary_processor", "baler", "bale_storage"]

# Everything except the hall is the equipment island, scaled by this on top of
# SCALE and recentred in the hall. See the docstring for the sweep behind 0.50.
ISLAND = [n for n in ITEMS if n != "hall_shell"]
SHRINK = 0.50

# Per-item pose overrides. name -> (x, y, yaw degrees) of the item's footprint
# centre, in SOURCE FACILITY metres - the same frame as the meshes, so a
# placement follows the scales when the room size changes. Demo-room metres
# would silently put the item outside a smaller room.
#
# Empty on purpose. Shrinking the island opens the corridor without moving
# anything relative to anything else, so the source arrangement stands as-is,
# including the floor overlaps the blockout ships with - the run prints them.
OVERRIDES = {}


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
print(f"홀 축척    {SCALE:.5f}  ->  홀 "
      f"{span_x*SCALE:.2f} x {span_y*SCALE:.2f} m")
print(f"설비 축척  {SCALE*SHRINK:.5f}  (홀의 {SHRINK:.2f} 배)")
print("설비 배치는 원본 그대로, 섬 전체를 축소해 홀 중앙에 배치\n")

# ---- hall interior --------------------------------------------------------
# Measured from the shell mesh, not assumed: the island is centred on this, so
# it has to be the real clear span between the walls at lidar height.
nx, ny = int(HALL_X / CELL), int(HALL_Y / CELL)


def footprint(v):
    """Bounding box of the part of a mesh the lidar plane can see."""
    seen = v[(v[:, 2] >= Z_LO) & (v[:, 2] <= Z_HI)]
    if not len(seen):
        seen = v
    return (seen[:, 0].min(), seen[:, 0].max(),
            seen[:, 1].min(), seen[:, 1].max())


def hall_interior():
    shell = np.zeros((ny, nx), bool)
    v = MESH["hall_shell"][0] * SCALE
    for a, b, c in MESH["hall_shell"][1]:
        tri = v[[a - 1, b - 1, c - 1]]
        if tri[:, 2].max() < Z_LO or tri[:, 2].min() > Z_HI:
            continue
        x0 = max(int((tri[:, 0].min() + HALL_X / 2) / CELL), 0)
        x1 = max(int(np.ceil((tri[:, 0].max() + HALL_X / 2) / CELL)), 0)
        y0 = max(int((tri[:, 1].min() + HALL_Y / 2) / CELL), 0)
        y1 = max(int(np.ceil((tri[:, 1].max() + HALL_Y / 2) / CELL)), 0)
        shell[y0:y1, x0:x1] = True
    def run(mask, start):
        """Extent of the free run containing `start`.

        Not the global free extent: the shell sits inside the room, so the
        strip between its walls and the room edge is free too and would
        otherwise be read as interior - which puts wall-hugging items outside
        the building.
        """
        lo = hi = start
        while lo > 0 and not mask[lo - 1]:
            lo -= 1
        while hi + 1 < len(mask) and not mask[hi + 1]:
            hi += 1
        return lo, hi

    x0, x1 = run(shell[ny // 2, :], nx // 2)
    y0, y1 = run(shell[:, nx // 2], ny // 2)
    return (x0 * CELL - HALL_X / 2, x1 * CELL - HALL_X / 2,
            y0 * CELL - HALL_Y / 2, y1 * CELL - HALL_Y / 2)


IX0, IX1, IY0, IY1 = hall_interior()
print(f"홀 내부    x {IX0:+.3f} .. {IX1:+.3f} ({IX1-IX0:.3f} m), "
      f"y {IY0:+.3f} .. {IY1:+.3f} ({IY1-IY0:.3f} m)")

occ = np.zeros((ny, nx), bool)


def mesh_scale(name):
    """The hall keeps SCALE; the island is shrunk on top of it."""
    return SCALE if name == "hall_shell" else SCALE * SHRINK


def transform(name, island_offset):
    """Source pose, any documented override, then the island recentring."""
    v = MESH[name][0] * mesh_scale(name)
    offset, yaw = np.zeros(3), 0.0
    if name in OVERRIDES:
        tx, ty, yaw = OVERRIDES[name]
        tx, ty = tx * mesh_scale(name), ty * mesh_scale(name)
        a = math.radians(yaw)
        rot = np.array([[math.cos(a), -math.sin(a), 0],
                        [math.sin(a), math.cos(a), 0],
                        [0, 0, 1]])
        v = v @ rot.T
        centre = np.array([(v[:, 0].min() + v[:, 0].max()) / 2,
                           (v[:, 1].min() + v[:, 1].max()) / 2, 0.0])
        offset = np.array([tx, ty, 0.0]) - centre
        v = v + offset
    if name != "hall_shell":
        v = v + island_offset
        offset = offset + island_offset
    return v, offset, yaw


def island_recentre():
    """Shift needed to put the shrunken island in the middle of the hall.

    One shift for the whole group, so nothing moves relative to anything else.
    """
    xs, ys = [], []
    for name in ISLAND:
        v, _, _ = transform(name, np.zeros(3))
        x0, x1, y0, y1 = footprint(v)
        xs += [x0, x1]
        ys += [y0, y1]
    return np.array([(IX0 + IX1) / 2 - (min(xs) + max(xs)) / 2,
                     (IY0 + IY1) / 2 - (min(ys) + max(ys)) / 2, 0.0])


ISLAND_OFFSET = island_recentre()
PLACED = {name: transform(name, ISLAND_OFFSET) for name in ITEMS}

_ix = [c for n in ISLAND for c in footprint(PLACED[n][0])[:2]]
_iy = [c for n in ISLAND for c in footprint(PLACED[n][0])[2:]]
print(f"설비 섬    {max(_ix)-min(_ix):.2f} x {max(_iy)-min(_iy):.2f} m,  "
      f"둘레 통로  서 {min(_ix)-IX0:.2f} / 동 {IX1-max(_ix):.2f} / "
      f"남 {min(_iy)-IY0:.2f} / 북 {IY1-max(_iy):.2f} m")
print(f"로봇 회전 필요 폭 {NEED*2:.2f} m\n")

for name in ITEMS:
    v, _, _ = PLACED[name]
    faces = MESH[name][1]
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

XC, YC = (IX0 + IX1) / 2, (IY0 + IY1) / 2

print(f"주행 가능 면적 {comp.sum()*CELL**2:.2f} m^2 "
      f"(방의 {comp.sum()/(nx*ny)*100:.0f}%)")
print(f"주행 범위      x {wx.min():.2f} .. {wx.max():.2f} m, "
      f"y {wy.min():.2f} .. {wy.max():.2f} m")
print(f"최대 통로 폭   {clear[comp].max()*2:.2f} m "
      f"(M1 회전 최소 {NEED*2:.2f} m)")
print(f"순환로         {'있음 - 설비 섬을 한 바퀴' if loop else '없음'}")

# The point of the shrink: is there a usable corridor on every side?
SIDES = (("남측", wy < YC - 0.3), ("동측", wx > XC + 0.3),
         ("북측", wy > YC + 0.3), ("서측", wx < XC - 0.3))
print(f"\n{'변':<8}{'통로 폭 (최소 / 최대)':>24}{'면적':>10}")
print("-" * 46)
for tag, sel in SIDES:
    if not sel.any():
        print(f"{tag:<8}{'없음':>24}")
        continue
    w = clear[iys[sel], ixs[sel]] * 2
    print(f"{tag:<8}{f'{w.min():.2f} / {w.max():.2f} m':>24}"
          f"{sel.sum()*CELL**2:>9.2f}m²")
print()

print(f"{'설비':<22}{'가로':>9}{'세로':>9}{'높이':>9}"
      f"{'중심 X':>10}{'중심 Y':>10}")
print("-" * 70)
for name in ITEMS:
    v = PLACED[name][0]
    mark = " *" if name in OVERRIDES else ""
    print(f"{name+mark:<22}{v[:,0].ptp():>7.2f} m{v[:,1].ptp():>7.2f} m"
          f"{v[:,2].ptp():>7.2f} m"
          f"{(v[:,0].min()+v[:,0].max())/2:>9.2f} m"
          f"{(v[:,1].min()+v[:,1].max())/2:>9.2f} m")

# ---- overlap report -------------------------------------------------------
# The source blockout has items sharing floor area. Report it every run so a
# relocation is not silently undone and a new one is not silently introduced.
def item_grid(name):
    v, _, _ = PLACED[name]
    g = np.zeros((ny, nx), bool)
    for a, b, c in MESH[name][1]:
        tri = v[[a - 1, b - 1, c - 1]]
        if tri[:, 2].max() < Z_LO or tri[:, 2].min() > Z_HI:
            continue
        x0 = max(int((tri[:, 0].min() + HALL_X / 2) / CELL), 0)
        x1 = max(int(np.ceil((tri[:, 0].max() + HALL_X / 2) / CELL)), 0)
        y0 = max(int((tri[:, 1].min() + HALL_Y / 2) / CELL), 0)
        y1 = max(int(np.ceil((tri[:, 1].max() + HALL_Y / 2) / CELL)), 0)
        g[y0:y1, x0:x1] = True
    return g


GRIDS = {n: item_grid(n) for n in ITEMS if n != "hall_shell"}
names = list(GRIDS)
overlaps = []
for i, a in enumerate(names):
    for b in names[i + 1:]:
        area = (GRIDS[a] & GRIDS[b]).sum() * CELL ** 2
        if area > 0.01:
            overlaps.append((area, a, b))
print("\n설비 간 바닥 겹침 (원본 블록아웃에서 유래)")
print("-" * 56)
if overlaps:
    for area, a, b in sorted(overlaps, reverse=True):
        print(f"{a:<22}{b:<22}{area:>7.3f} m^2")
else:
    print("없음")
for name in OVERRIDES:
    bad = [f"{a}/{b}" for _, a, b in overlaps if name in (a, b)]
    state = f"겹침 남음: {', '.join(bad)}" if bad else "겹침 해소됨"
    print(f"이동 대상 {name}: {state}")

# ---- patrol waypoints: once around the island -----------------------------
# The corridor is a ring, so the route is the ring's centre line: a rectangle
# halfway between the island and the walls, sampled at its corners and side
# midpoints. Picking by "roomiest cell in this direction" instead would bunch
# every waypoint into the corners, which are the widest part of the ring.
RX0 = (IX0 + min(_ix)) / 2
RX1 = (IX1 + max(_ix)) / 2
RY0 = (IY0 + min(_iy)) / 2
RY1 = (IY1 + max(_iy)) / 2
RING = [("남측", (RX0 + RX1) / 2, RY0), ("남동", RX1, RY0),
        ("동측", RX1, (RY0 + RY1) / 2), ("북동", RX1, RY1),
        ("북측", (RX0 + RX1) / 2, RY1), ("북서", RX0, RY1),
        ("서측", RX0, (RY0 + RY1) / 2), ("남서", RX0, RY0)]


SLACK = 0.5          # how far a waypoint may slide along the ring to breathe


def snap(tx, ty):
    """Roomiest drivable cell within SLACK of a point on the ring centre line.

    Not simply the nearest cell. The island is not a rectangle, so a side's
    narrowest point is wherever that side's extreme machine sits - and the
    geometric mid-side often lands right on it. Sliding along the ring to the
    roomiest cell nearby costs nothing and is what gives each waypoint room to
    turn in place.
    """
    d2 = (wx - tx) ** 2 + (wy - ty) ** 2
    near = d2 <= SLACK ** 2
    if not near.any():
        near = d2 <= d2.min() + 1e-9
    idx = np.where(near)[0]
    k = idx[np.argmax(clear[iys[idx], ixs[idx]])]
    # face the island: heading from the waypoint back towards the centre
    yaw = math.atan2(YC - wy[k], XC - wx[k])
    return (wx[k], wy[k], yaw), clear[iys[k], ixs[k]]


WAYPOINTS, LABELS = {}, {}
for tag, tx, ty in RING:
    label = f"P{len(WAYPOINTS)+1}"
    WAYPOINTS[label], LABELS[label] = snap(tx, ty), tag

print("\n순찰 웨이포인트 (설비 섬을 한 바퀴, 각 지점에서 설비를 바라봄)")
print("-" * 64)
for label, ((px, py, pyaw), c) in WAYPOINTS.items():
    print(f"{label:<5}{LABELS[label]:<8} x={px:>6.2f}  y={py:>6.2f}"
          f"  yaw={math.degrees(pyaw):>7.1f}deg   여유 {c*2:.2f} m")
# Spawn on the roomiest waypoint - the narrow sides are a bad place to start.
(sx, sy, syaw), sc = max(WAYPOINTS.values(), key=lambda w: w[1])
print(f"\n권장 스폰 위치: x={sx:.2f}  y={sy:.2f}  yaw={syaw:.4f}"
      f"  (여유 {sc*2:.2f} m)")

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

  Layout is the RoboLotus/gazebo-simulator recycling facility: seven of its
  eight models, same include order, each at its source position and rotation
  relative to the others. Nothing is moved or turned; control_room is dropped.

  Two scales. The hall shell is {s:.5f}, the largest that fits the
  {sx:.2f} x {sy:.2f} m plant into a {hx} x {hy} m room. The equipment is
  {e:.5f} - the same scale times {k:.2f} - applied to the island as one rigid
  group and recentred in the hall, so relative distances inside the plant are
  untouched and only the plant-to-robot ratio changes.

  That shrink is what makes the world drivable. At full size the plant fills
  the hall wall to wall and leaves one dead-end aisle; pulled in, it leaves a
  corridor right around itself and the robot can drive a closed loop.

  The Fortress conversion is limited to three things the original could not
  provide: ignition-gazebo-* system plugin names instead of gz-sim-*, an IMU
  system for the robot IMU, and the ODE tuning the earlier worlds relied on.
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
""".format(s=SCALE, e=SCALE * SHRINK, k=SHRINK,
           sx=span_x, sy=span_y, hx=HALL_X, hy=HALL_Y,
           lw=-15 * SCALE, le=15 * SCALE, lz=10 * SCALE, lr=40 * SCALE,
           gx=HALL_X + 4, gy=HALL_Y + 4)]

for name in ITEMS:
    uri = f"model://{name}/meshes/{name}.obj"
    _, off, yaw = PLACED[name]
    s = mesh_scale(name)
    # visual and collision take the same scale and the same model pose, so the
    # collision geometry moves and shrinks with what is drawn.
    parts.append(f"""
    <model name="{name}">
      <static>true</static>
      <pose>{off[0]:.4f} {off[1]:.4f} 0 0 0 {math.radians(yaw):.6f}</pose>
      <link name="link">
        <visual name="visual">
          <geometry><mesh><uri>{uri}</uri>
            <scale>{s:.5f} {s:.5f} {s:.5f}</scale></mesh></geometry>
        </visual>
        <collision name="collision">
          <geometry><mesh><uri>{uri}</uri>
            <scale>{s:.5f} {s:.5f} {s:.5f}</scale></mesh></geometry>
        </collision>
      </link>
    </model>""")

parts.append("\n  </world>\n</sdf>\n")
OUT_WORLD.write_text("".join(parts))
print(f"\n월드 생성: {OUT_WORLD}")
print(f"평면도    : {OUT_PLAN}")
