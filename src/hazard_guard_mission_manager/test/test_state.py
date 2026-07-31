import json

from hazard_guard_mission_manager.state import (
    MissionStateStore,
    initial_mission_state,
)


def test_initial_state_is_safe_to_mutate() -> None:
    first = initial_mission_state()
    second = initial_mission_state()

    first["waypoints"].append({"id": "A"})

    assert second["waypoints"] == []


def test_store_publishes_replaced_and_updated_state() -> None:
    payloads: list[str] = []
    store = MissionStateStore(payloads.append)
    state = initial_mission_state()
    state["waypoints"] = [{"id": "A", "status": "pending"}]

    store.replace(state)
    store.update(status="running")
    store.update_waypoint(0, "completed", "완료", yaw_error_deg=1.2)

    latest = json.loads(payloads[-1])
    assert latest["status"] == "running"
    assert latest["waypoints"][0] == {
        "id": "A",
        "status": "completed",
        "message": "완료",
        "yaw_error_deg": 1.2,
    }


def test_snapshot_does_not_expose_internal_state() -> None:
    store = MissionStateStore(lambda _payload: None)

    snapshot = store.snapshot()
    snapshot["status"] = "corrupted"

    assert store.snapshot()["status"] == "idle"
