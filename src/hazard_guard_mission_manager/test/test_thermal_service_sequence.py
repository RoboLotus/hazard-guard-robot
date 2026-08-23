from types import SimpleNamespace

from hazard_guard_mission_manager.node import HazardGuardMissionManager


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(("info", message))

    def warning(self, message):
        self.messages.append(("warning", message))

    def error(self, message):
        self.messages.append(("error", message))


class FakeFuture:
    def __init__(self, response=None, *, complete=True):
        self._response = response
        self._complete = complete
        self.cancelled = False

    def add_done_callback(self, callback):
        if self._complete:
            callback(self)

    def result(self):
        return self._response

    def cancel(self):
        self.cancelled = True


class FakeClient:
    def __init__(self, future, *, ready=True):
        self.future = future
        self.ready = ready
        self.calls = 0

    def service_is_ready(self):
        return self.ready

    def call_async(self, _request):
        self.calls += 1
        return self.future


def make_manager(timeout_sec=0.1):
    manager = object.__new__(HazardGuardMissionManager)
    manager._thermal_sequence_faulted = False
    manager._logger = FakeLogger()
    manager.get_logger = lambda: manager._logger
    manager.get_parameter = lambda _name: SimpleNamespace(value=timeout_sec)
    return manager


def test_thermal_service_completes_before_returning():
    manager = make_manager()
    response = SimpleNamespace(success=True, message="recorded")
    client = FakeClient(FakeFuture(response))

    assert manager._call_thermal_service(client, "cycle 1") is response
    assert client.calls == 1
    assert manager._thermal_sequence_faulted is False


def test_thermal_timeout_disables_later_calls_for_the_mission():
    manager = make_manager(timeout_sec=0.01)
    future = FakeFuture(complete=False)
    client = FakeClient(future)

    assert manager._call_thermal_service(client, "cycle 1") is None
    assert future.cancelled is True
    assert manager._thermal_sequence_faulted is True

    assert manager._call_thermal_service(client, "cycle 2") is None
    assert client.calls == 1


def test_inactive_optional_service_does_not_fault_the_mission():
    manager = make_manager()
    client = FakeClient(FakeFuture(), ready=False)

    assert manager._call_thermal_service(client, "cycle 1") is None
    assert manager._thermal_sequence_faulted is False
    assert client.calls == 0
