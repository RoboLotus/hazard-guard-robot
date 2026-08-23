#!/usr/bin/env python3
"""Read-only environment inventory for Hazard Guard YOLO deployments.

The script intentionally uses only the Python standard library. It does not
install packages, write files, or change the host configuration.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import os
import platform
import shutil
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional


UNKNOWN = "not detected"


def run_command(command: Iterable[str], timeout: float = 8.0) -> Dict[str, Any]:
    """Run an inventory command without failing when it is unavailable."""
    argv = list(command)
    executable = shutil.which(argv[0])
    if executable is None:
        return {
            "available": False,
            "command": argv,
            "value": None,
            "error": "command not found",
        }

    argv[0] = executable
    try:
        completed = subprocess.run(
            argv,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return {
            "available": True,
            "command": list(command),
            "value": None,
            "error": str(exc),
        }

    stdout = completed.stdout.strip()
    stderr = completed.stderr.strip()
    return {
        "available": True,
        "command": list(command),
        "value": stdout or None,
        "returncode": completed.returncode,
        "error": stderr or (None if completed.returncode == 0 else "command failed"),
    }


def read_text(path: str) -> Optional[str]:
    """Read a small system metadata file, returning None when unavailable."""
    try:
        return (
            Path(path)
            .read_text(encoding="utf-8", errors="replace")
            .strip()
            .rstrip("\x00")
            or None
        )
    except (OSError, UnicodeError):
        return None


def parse_os_release(raw: Optional[str]) -> Dict[str, str]:
    values: Dict[str, str] = {}
    if not raw:
        return values
    for line in raw.splitlines():
        if "=" not in line or line.lstrip().startswith("#"):
            continue
        key, value = line.split("=", 1)
        values[key] = value.strip().strip('"')
    return values


def import_version(module_name: str, attribute: str = "__version__") -> Dict[str, Any]:
    """Import a module defensively and report its version."""
    try:
        module = importlib.import_module(module_name)
        version = getattr(module, attribute, None)
        return {
            "installed": True,
            "version": str(version) if version is not None else UNKNOWN,
            "location": getattr(module, "__file__", None),
            "error": None,
        }
    except Exception as exc:  # Third-party modules may raise more than ImportError.
        return {
            "installed": False,
            "version": None,
            "location": None,
            "error": f"{type(exc).__name__}: {exc}",
        }


def inspect_torch() -> Dict[str, Any]:
    result = import_version("torch")
    if not result["installed"]:
        return result
    try:
        torch = importlib.import_module("torch")
        cuda_available = bool(torch.cuda.is_available())
        result.update(
            {
                "cuda_build": str(getattr(torch.version, "cuda", None)),
                "cuda_available": cuda_available,
                "cudnn_version": (
                    str(torch.backends.cudnn.version())
                    if torch.backends.cudnn.is_available()
                    else None
                ),
                "device_count": int(torch.cuda.device_count()) if cuda_available else 0,
                "devices": [
                    str(torch.cuda.get_device_name(index))
                    for index in range(torch.cuda.device_count())
                ]
                if cuda_available
                else [],
            }
        )
    except Exception as exc:
        result["runtime_error"] = f"{type(exc).__name__}: {exc}"
    return result


def hash_file(path: str) -> Dict[str, Any]:
    candidate = Path(path).expanduser()
    info: Dict[str, Any] = {
        "path": str(candidate.resolve(strict=False)),
        "exists": candidate.is_file(),
        "size_bytes": None,
        "sha256": None,
        "error": None,
    }
    if not candidate.is_file():
        info["error"] = "file not found"
        return info

    digest = hashlib.sha256()
    try:
        with candidate.open("rb") as model_file:
            for chunk in iter(lambda: model_file.read(1024 * 1024), b""):
                digest.update(chunk)
        info["size_bytes"] = candidate.stat().st_size
        info["sha256"] = digest.hexdigest()
    except OSError as exc:
        info["error"] = str(exc)
    return info


def collect_environment(model_paths: Iterable[str]) -> Dict[str, Any]:
    os_release = parse_os_release(read_text("/etc/os-release"))
    packages = run_command(
        [
            "dpkg-query",
            "-W",
            "-f=${binary:Package}\t${Version}\n",
            "nvidia-jetpack",
            "nvidia-l4t-core",
            "libcudnn*",
            "libnvinfer*",
        ]
    )
    nvcc = run_command(["nvcc", "--version"])
    ros = run_command(["ros2", "doctor", "--report"], timeout=20.0)
    nvidia_smi = run_command(
        ["nvidia-smi", "--query-gpu=name,driver_version", "--format=csv,noheader"]
    )

    return {
        "schema_version": 1,
        "system": {
            "hostname": platform.node(),
            "architecture": platform.machine(),
            "jetson_model": read_text("/proc/device-tree/model"),
            "platform": platform.platform(),
            "kernel": platform.release(),
            "ubuntu": os_release.get("PRETTY_NAME") or os_release.get("NAME") or UNKNOWN,
            "os_release": os_release,
            "l4t_release": read_text("/etc/nv_tegra_release"),
            "jetpack_and_nvidia_packages": packages,
        },
        "python": {
            "version": platform.python_version(),
            "executable": sys.executable,
            "implementation": platform.python_implementation(),
            "virtual_environment": os.environ.get("VIRTUAL_ENV"),
            "base_prefix": sys.base_prefix,
            "prefix": sys.prefix,
        },
        "ros": {
            "distro": os.environ.get("ROS_DISTRO"),
            "version": os.environ.get("ROS_VERSION"),
            "python_version": os.environ.get("ROS_PYTHON_VERSION"),
            "doctor_report": ros,
        },
        "acceleration": {
            "cuda_nvcc": nvcc,
            "cuda_version_file": read_text("/usr/local/cuda/version.json")
            or read_text("/usr/local/cuda/version.txt"),
            "nvidia_smi": nvidia_smi,
            "tensorrt": import_version("tensorrt"),
            "torch": inspect_torch(),
        },
        "python_packages": {
            "torchvision": import_version("torchvision"),
            "ultralytics": import_version("ultralytics"),
            "opencv": import_version("cv2"),
            "numpy": import_version("numpy"),
        },
        "models": [hash_file(path) for path in model_paths],
    }


def nested(data: Dict[str, Any], *keys: str, default: Any = None) -> Any:
    value: Any = data
    for key in keys:
        if not isinstance(value, dict):
            return default
        value = value.get(key)
    return default if value is None else value


def line(label: str, value: Any) -> str:
    if isinstance(value, bool):
        rendered = "yes" if value else "no"
    elif isinstance(value, list):
        rendered = ", ".join(str(item) for item in value) if value else UNKNOWN
    else:
        rendered = str(value) if value not in (None, "") else UNKNOWN
    return f"{label:<27} {rendered}"


def command_summary(command: Dict[str, Any]) -> str:
    value = command.get("value")
    if value:
        return str(value).splitlines()[-1]
    return command.get("error") or UNKNOWN


def human_report(data: Dict[str, Any]) -> str:
    torch = nested(data, "acceleration", "torch", default={})
    packages = data["python_packages"]
    output = [
        "Hazard Guard YOLO environment (read-only)",
        "=" * 45,
        line("Architecture", nested(data, "system", "architecture")),
        line("Jetson model", nested(data, "system", "jetson_model")),
        line("Operating system", nested(data, "system", "ubuntu")),
        line("Kernel", nested(data, "system", "kernel")),
        line("Jetson L4T", nested(data, "system", "l4t_release")),
        line(
            "JetPack/packages",
            command_summary(nested(data, "system", "jetpack_and_nvidia_packages", default={})),
        ),
        line("Python", nested(data, "python", "version")),
        line("Python executable", nested(data, "python", "executable")),
        line("Virtual environment", nested(data, "python", "virtual_environment")),
        line("ROS distro", nested(data, "ros", "distro")),
        line("CUDA (nvcc)", command_summary(nested(data, "acceleration", "cuda_nvcc", default={}))),
        line("torch", torch.get("version")),
        line("torch CUDA build", torch.get("cuda_build")),
        line("torch GPU available", torch.get("cuda_available")),
        line("GPU devices", torch.get("devices")),
        line("cuDNN (via torch)", torch.get("cudnn_version")),
        line("torchvision", packages["torchvision"].get("version")),
        line("TensorRT", nested(data, "acceleration", "tensorrt", "version")),
        line("ultralytics", packages["ultralytics"].get("version")),
        line("OpenCV", packages["opencv"].get("version")),
        line("numpy", packages["numpy"].get("version")),
    ]

    if data["models"]:
        output.extend(["", "Model files"])
        for model in data["models"]:
            status = model["sha256"] or model["error"] or UNKNOWN
            output.append(line(model["path"], status))

    warnings: List[str] = []
    if platform.machine().lower() not in {"aarch64", "arm64"}:
        warnings.append("This is not an aarch64 Jetson host; treat results as a development/simulation inventory.")
    if nested(data, "ros", "distro") != "humble":
        warnings.append("ROS_DISTRO is not 'humble' (or ROS has not been sourced).")
    if not torch.get("installed"):
        warnings.append("torch could not be imported.")
    elif not torch.get("cuda_available"):
        warnings.append("torch imported, but CUDA GPU acceleration is unavailable.")

    if warnings:
        output.extend(["", "Warnings"])
        output.extend(f"- {warning}" for warning in warnings)
    return "\n".join(output)


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Inspect the YOLO/Jetson/ROS environment without changing it."
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="emit machine-readable JSON instead of the human-readable report",
    )
    parser.add_argument(
        "--model",
        action="append",
        default=[],
        metavar="PATH",
        help="calculate SHA-256 for a model/engine file (repeatable)",
    )
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    result = collect_environment(args.model)
    if args.json:
        print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    else:
        print(human_report(result))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
