from pathlib import Path

import yaml


CONFIG = Path(__file__).parents[1] / "config"


def test_simulation_and_physical_patrol_controllers_disallow_reverse_x():
    for filename in ("nav2.yaml", "physical_nav2.yaml"):
        params = yaml.safe_load((CONFIG / filename).read_text(encoding="utf-8"))
        follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]

        assert follow_path["min_vel_x"] == 0.0, filename
        assert follow_path["max_vel_x"] > 0.0, filename
        # The M1 is mecanum: lateral avoidance stays available even though
        # reverse longitudinal trajectories are prohibited during patrol.
        assert follow_path["min_vel_y"] < 0.0, filename
        assert follow_path["max_vel_y"] > 0.0, filename
