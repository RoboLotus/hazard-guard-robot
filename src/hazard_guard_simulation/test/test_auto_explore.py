from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "auto_explore.py"
LAUNCH = Path(__file__).parents[1] / "launch" / "explore.launch.py"
SPEC = spec_from_file_location("auto_explore", SCRIPT)
MODULE = module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


def grid(rows, resolution=0.5, origin=(0.0, 0.0)):
    """Build a FrontierMap from row strings: '.' free, '?' unknown, '#' wall."""
    values = {".": 0, "?": MODULE.UNKNOWN, "#": 100}
    cells = [values[character] for row in rows for character in row]
    return MODULE.FrontierMap(cells, len(rows[0]), len(rows), resolution,
                              origin[0], origin[1])


def test_selftest_passes():
    MODULE.selftest()


def test_open_side_is_a_frontier():
    found = grid(["...???", "...???", "######"]).clusters(minimum_cells=1)
    assert len(found) == 1
    assert found[0][2] == 2


def test_closed_room_has_no_frontier():
    assert grid(["######", "#....#", "######"]).clusters(minimum_cells=1) == []


def test_two_doorways_stay_separate():
    # Free cells top-left and bottom-left, unknown beyond each, wall between.
    found = grid(["..??", "####", "..??"]).clusters(minimum_cells=1)
    assert len(found) == 2, found
    assert {round(point[1], 3) for point in found} == {0.25, 1.25}


def test_larger_frontier_is_listed_first():
    found = grid(["..??", "..??", "####", "..??"]).clusters(minimum_cells=1)
    assert [point[2] for point in found] == [2, 1]


def test_goal_lands_on_a_cell_centre_inside_the_known_area():
    # Column 1 is the frontier; its world x must be the centre of that column,
    # not the corner it shares with the unknown column 2.
    world_x, world_y, _ = grid(["#.??", "#.??", "####"],
                               resolution=0.2,
                               origin=(-1.0, -1.0)).clusters(1)[0]
    assert abs(world_x - (-1.0 + 1.5 * 0.2)) < 1e-9
    assert abs(world_y - (-1.0 + 0.5 * 0.2)) < 1e-9


def test_launch_keeps_slam_toolbox_as_the_map_authority():
    source = LAUNCH.read_text()
    # navigation_launch.py is the bringup without map_server and AMCL. Using
    # bringup_launch.py instead would start a second /map publisher and fight
    # the mapping session this attaches to.
    assert "navigation_launch.py" in source
    assert "bringup_launch.py" not in source
