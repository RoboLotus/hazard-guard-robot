from __future__ import annotations

import sys

from hazard_guard_patrol_benchmark.runner import _run_patrol_script


def test_run_patrol_script_requires_action_success(tmp_path) -> None:
    script = tmp_path / "patrol.sh"
    script.write_text(
        "#!/usr/bin/env bash\necho 'Result: success: false'\nexit 0\n",
        encoding="utf-8",
    )

    exit_code, action_succeeded, reason = _run_patrol_script(
        str(script),
        "mission-1",
        {},
    )

    assert exit_code == 0
    assert action_succeeded is False
    assert "did not report" in reason


def test_run_patrol_script_accepts_successful_action(tmp_path) -> None:
    script = tmp_path / "patrol.sh"
    script.write_text(
        "#!/usr/bin/env bash\necho 'Result:'\necho '  success: true'\n",
        encoding="utf-8",
    )

    environment = {"PATH": sys.path[0]}
    environment.update(__import__("os").environ)
    exit_code, action_succeeded, reason = _run_patrol_script(
        str(script),
        "mission-2",
        environment,
    )

    assert exit_code == 0
    assert action_succeeded is True
    assert reason == "completed"
