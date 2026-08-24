[CmdletBinding()]
param(
    [ValidateRange(1, 50)]
    [int]$Runs = 1,
    [ValidateRange(0, 60)]
    [double]$DwellSeconds = 3.0,
    [switch]$SmokeTest,
    [switch]$ContinueOnFailure,
    [switch]$KeepEnvironment,
    [string]$WslDistro = "Ubuntu-22.04-D"
)

$ErrorActionPreference = "Stop"
$robotRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$workspaceRoot = Split-Path $robotRoot -Parent
$environmentRoot = Join-Path $workspaceRoot "slam-jetson-env"
$simulationRoot = Join-Path $workspaceRoot "Simulation_env"
$composeRoot = Join-Path $PSScriptRoot "docker"
$runId = Get-Date -Format "yyyyMMddTHHmmss"
$logRoot = Join-Path $robotRoot "runtime\benchmarks\harness\$runId"
New-Item -ItemType Directory -Force -Path $logRoot | Out-Null

foreach ($required in @($environmentRoot, $simulationRoot)) {
    if (-not (Test-Path -LiteralPath $required)) {
        throw "필수 저장소를 찾을 수 없습니다: $required"
    }
}

$env:HAZARD_GUARD_SIM_ENV_HOST_PATH = $simulationRoot
$env:ROSMASTER_IMAGE = "rosmaster-team:humble"
$wslIp = ((wsl.exe -d $WslDistro -- hostname -I).Trim().Split(' ')[0])
if (-not $wslIp) {
    throw "WSL IP를 확인하지 못했습니다: $WslDistro"
}
$env:DISPLAY = "${wslIp}:0"
$proxyWindowsPath = Join-Path $PSScriptRoot "x11_proxy.py"
$proxyWslPath = (
    wsl.exe -d $WslDistro -- wslpath -a ($proxyWindowsPath -replace '\\', '/')
).Trim()
$compose = @(
    "compose",
    "--project-directory", $environmentRoot,
    "-f", (Join-Path $environmentRoot "compose.yaml"),
    "-f", (Join-Path $environmentRoot "compose.desktop.yaml"),
    "-f", (Join-Path $composeRoot "compose.benchmark.yaml"),
    "-f", (Join-Path $environmentRoot "compose.gpu.yaml"),
    "-f", (Join-Path $composeRoot "compose.gpu-wsl.yaml")
)

function Invoke-Compose([string[]]$Arguments) {
    & docker @compose @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "docker compose 실패: $($Arguments -join ' ')"
    }
}

function Invoke-Container([string]$Command) {
    & docker exec rosmaster-team-dev bash -lc $Command
    if ($LASTEXITCODE -ne 0) {
        throw "컨테이너 명령 실패"
    }
}

function Stop-Environment {
    try {
        Invoke-Compose @("down", "--remove-orphans")
    } catch {
        docker rm -f rosmaster-team-dev 2>$null | Out-Null
    }
}

function Start-X11Proxy {
    wsl.exe -d $WslDistro -- bash -lc "pkill -f '$proxyWslPath' || true" 2>$null | Out-Null
    Start-Process -FilePath "wsl.exe" `
        -ArgumentList @("-d", $WslDistro, "--", "python3", $proxyWslPath) `
        -WindowStyle Hidden | Out-Null
    Start-Sleep -Seconds 2
}

$completed = $false
try {
    Stop-Environment
    Start-X11Proxy
    Invoke-Compose @("up", "-d", "--no-build", "--force-recreate", "dev")

    $build = @'
source /opt/ros/humble/setup.bash
cd /workspace
colcon --log-base /workspace/log build --symlink-install \
  --base-paths /workspace/hazard_guard_robot/src \
  --build-base /workspace/build \
  --install-base /workspace/install \
  --packages-up-to hazard_guard_simulation hazard_guard_mission_manager hazard_guard_patrol_benchmark
'@
    Invoke-Container $build

    Invoke-Container "sed 's#<render_engine>ogre2</render_engine>#<render_engine>ogre</render_engine>#' /workspace/Simulation_env/gazebo/worlds/real_factory.sdf >/tmp/benchmark_world.sdf"

    $launch = @'
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
exec ros2 launch hazard_guard_simulation localization.launch.py \
  gui:=false \
  world:=/tmp/benchmark_world.sdf \
  world_name:=real_factory \
  map:=/workspace/Simulation_env/gazebo/maps/real_factory.yaml \
  spawn_x:=0.0 spawn_y:=3.45 spawn_z:=0.05 spawn_yaw:=0.0 \
  include_dispenser:=false \
  heat_source_profile:=/workspace/Simulation_env/gazebo/config/heat_sources/real_factory.json \
  use_person_safety:=false use_thermal_pipeline:=false \
  enable_rgbd_mapping:=false use_performance_monitor:=false \
  >/tmp/patrol_benchmark_launch.log 2>&1
'@
    & docker exec -d rosmaster-team-dev bash -lc $launch

    $benchmark = @'
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
export HAZARD_GUARD_SIMULATION_ENV=/workspace/Simulation_env
export HAZARD_GUARD_WORKSPACE=/workspace/hazard_guard_robot
exec ros2 launch hazard_guard_patrol_benchmark patrol_benchmark.launch.py \
  world_id:=real_factory compare_localization:=true \
  >/tmp/patrol_benchmark_node.log 2>&1
'@
    & docker exec -d rosmaster-team-dev bash -lc $benchmark

    $ready = @'
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
for _ in $(seq 1 90); do
  if ros2 action list 2>/dev/null | grep -qx /hazard_guard/run_patrol && \
     ros2 topic list 2>/dev/null | grep -qx /hazard_guard/mission/status && \
     ros2 topic list 2>/dev/null | grep -qx /scan && \
     test "$(grep -c 'Managed nodes are active' /tmp/patrol_benchmark_launch.log 2>/dev/null || true)" -ge 2; then
    sleep 5
    exit 0
  fi
  sleep 2
done
echo 'Gazebo, Nav2 또는 benchmark 준비 시간 초과' >&2
exit 1
'@
    Invoke-Container $ready

    if ($SmokeTest) {
        $run = @"
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
for index in `$(seq 1 $Runs); do
  mission_id="smoke-$runId-`$(printf '%03d' "`$index")"
  ros2 action send_goal --feedback /hazard_guard/run_patrol \
    hazard_guard_interfaces/action/RunPatrol \
    "{mission_id: '`$mission_id', name: 'benchmark smoke patrol', frame_id: 'map', return_to_start: false, waypoints: [{id: 'S01', name: 'short straight', x: 4.0, y: 3.45, yaw: 0.0, dwell_seconds: $DwellSeconds}]}" \
    | tee "/tmp/`$mission_id.action.log"
  grep -q 'success: true' "/tmp/`$mission_id.action.log" || exit 1
done
"@
    } else {
        $continueFlag = if ($ContinueOnFailure) { " --continue-on-failure" } else { "" }
        $run = @"
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
export HAZARD_GUARD_SIMULATION_ENV=/workspace/Simulation_env
export HAZARD_GUARD_WORKSPACE=/workspace/hazard_guard_robot
ros2 run hazard_guard_patrol_benchmark patrol_benchmark_run \
  --world-id real_factory --runs $Runs --dwell-seconds $DwellSeconds$continueFlag
"@
    }
    Invoke-Container $run

    Start-Sleep -Seconds 3
    $aggregatePath = "/workspace/hazard_guard_robot/runtime/benchmarks/aggregates/$runId-real_factory"
    $aggregate = @"
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
export HAZARD_GUARD_WORKSPACE=/workspace/hazard_guard_robot
ros2 run hazard_guard_patrol_benchmark patrol_benchmark_aggregate \
  --world-id real_factory --latest $Runs --output $aggregatePath
"@
    Invoke-Container $aggregate

    & docker exec rosmaster-team-dev bash -lc "cp /tmp/patrol_benchmark_launch.log '/workspace/hazard_guard_robot/runtime/benchmarks/harness/$runId/launch.log'; cp /tmp/patrol_benchmark_node.log '/workspace/hazard_guard_robot/runtime/benchmarks/harness/$runId/benchmark-node.log'"
    $completed = $true
    Write-Host "Benchmark aggregate: $robotRoot\runtime\benchmarks\aggregates\$runId-real_factory.json" -ForegroundColor Green
} finally {
    if (-not $KeepEnvironment) {
        Stop-Environment
    } elseif ($completed) {
        Write-Host "Container kept running: rosmaster-team-dev" -ForegroundColor Yellow
    }
    if (-not $KeepEnvironment) {
        wsl.exe -d $WslDistro -- bash -lc "pkill -f '$proxyWslPath' || true" 2>$null | Out-Null
    }
}
