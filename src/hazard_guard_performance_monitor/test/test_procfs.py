from pathlib import Path

from hazard_guard_performance_monitor.procfs import (
    CpuTicks,
    cpu_percentages,
    process_label,
    read_memory,
)


def test_cpu_percentages_include_total_and_cores():
    previous = {
        "cpu": CpuTicks(idle=100, total=200),
        "cpu0": CpuTicks(idle=50, total=100),
    }
    current = {
        "cpu": CpuTicks(idle=120, total=300),
        "cpu0": CpuTicks(idle=60, total=150),
    }

    assert cpu_percentages(previous, current) == {
        "cpu": 80.0,
        "cpu0": 80.0,
    }


def test_memory_uses_available_ram_and_swap(tmp_path: Path):
    (tmp_path / "meminfo").write_text(
        "MemTotal:       1000 kB\n"
        "MemAvailable:    250 kB\n"
        "SwapTotal:       200 kB\n"
        "SwapFree:         50 kB\n",
        encoding="ascii",
    )

    memory = read_memory(tmp_path)

    assert memory["used_percent"] == 75.0
    assert memory["swap_used_bytes"] == 150 * 1024


def test_process_patterns_keep_expensive_nodes_separate():
    command = "/opt/ros/humble/lib/nav2_controller/controller_server"
    assert process_label(command) == "nav2_controller"
    assert process_label("python3 -m uvicorn app.main:app") == "fastapi"
    assert process_label("unrelated-daemon") is None
