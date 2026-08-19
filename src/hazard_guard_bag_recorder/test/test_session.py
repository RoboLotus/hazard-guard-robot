import json
from pathlib import Path

from hazard_guard_bag_recorder.preflight import PreflightReport
from hazard_guard_bag_recorder.session import BagSession, SessionError, create_session_paths, safe_session_name


class FakeProcess:
    def __init__(self):
        self.running = True
        self.terminated = False

    def poll(self):
        return None if self.running else 0

    def terminate(self):
        self.terminated = True
        self.running = False

    def wait(self, timeout=None):
        self.running = False
        return 0

    def kill(self):
        self.running = False


def report():
    return PreflightReport("navigation-core", {"scan": "/scan", "tf": "/tf"}, (), (), 123)


def test_session_path_and_name_cannot_escape_storage_root(tmp_path):
    paths = create_session_paths(tmp_path, "../../bad name")
    assert paths.root in paths.session_dir.parents
    assert ".." not in paths.session_dir.name
    assert safe_session_name("../../bad name") == "bad-name"


def test_recorder_uses_argument_list_and_writes_manifest(tmp_path):
    calls = []

    def runner(*args, **kwargs):
        calls.append((args, kwargs))
        return FakeProcess()

    session = BagSession(create_session_paths(tmp_path, "test"), "navigation-core", report(), command_runner=runner)
    session.start()
    command = calls[0][0][0]
    assert command[:4] == ["ros2", "bag", "record", "--storage"]
    assert calls[0][1]["shell"] is False
    assert "/scan" in command
    manifest = session.stop()
    saved = json.loads(session.paths.manifest_path.read_text(encoding="utf-8"))
    assert manifest["status"] == "completed"
    assert saved["selected_topics"]["scan"] == "/scan"


def test_recorder_rejects_missing_required_topics(tmp_path):
    invalid = PreflightReport("navigation-core", {}, ("scan",), (), 0)
    try:
        BagSession(create_session_paths(tmp_path, "test"), "navigation-core", invalid)
    except SessionError as exc:
        assert "missing required" in str(exc)
    else:
        raise AssertionError("missing required topics must prevent subprocess creation")


def test_recorder_stops_before_exhausting_reserved_free_space(tmp_path):
    session = BagSession(
        create_session_paths(tmp_path, "reserved-space"),
        "navigation-core",
        report(),
        minimum_free_bytes=2**63,
        command_runner=lambda *_args, **_kwargs: FakeProcess(),
    )
    session.start()
    assert session.enforce_limits() == "min-free-space"
    assert session.stop("min-free-space")["status"] == "limited"


def test_recorder_uses_limits_captured_when_the_session_is_created(tmp_path):
    session = BagSession(
        create_session_paths(tmp_path, "frozen-limit"),
        "navigation-core",
        report(),
        max_duration_seconds=0,
        max_size_bytes=1,
        command_runner=lambda *_args, **_kwargs: FakeProcess(),
    )
    session.start()
    session.paths.bag_dir.mkdir()
    (session.paths.bag_dir / "sample.db3").write_bytes(b"12")
    assert session.enforce_limits() == "max-size"
