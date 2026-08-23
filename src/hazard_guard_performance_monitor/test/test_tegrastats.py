from hazard_guard_performance_monitor.tegrastats import parse_tegrastats_line


def test_parse_tegrastats_line_extracts_jetson_metrics():
    parsed = parse_tegrastats_line(
        "RAM 3120/7620MB GR3D_FREQ 47% CPU@51.2C GPU@49.8C "
        "VDD_IN 8234mW/7100mW VDD_CPU_GPU_CV 3412mW/3000mW"
    )

    assert parsed["gpu_percent"] == 47.0
    assert parsed["tegrastats_ram_used_mb"] == 3120.0
    assert parsed["temperatures_c"] == {"cpu": 51.2, "gpu": 49.8}
    assert parsed["power_mw"]["vdd_in"] == 8234.0


def test_non_jetson_output_returns_empty_payload():
    assert parse_tegrastats_line("command not found") == {}
