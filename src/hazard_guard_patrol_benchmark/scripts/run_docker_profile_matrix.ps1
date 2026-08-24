[CmdletBinding()]
param(
    [ValidateSet("all", "cpu", "gpu")]
    [string]$Mode = "all",
    [ValidateRange(1, 10)]
    [int]$HeadlessRuns = 2,
    [ValidateRange(0, 10)]
    [int]$GuiRuns = 1,
    [ValidateRange(10, 600)]
    [int]$DurationSeconds = 45,
    [ValidateRange(5, 120)]
    [int]$WarmupSeconds = 15,
    [string]$WslDistro = "Ubuntu-22.04-D"
)

$ErrorActionPreference = "Stop"
$robotRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..\..")).Path
$workspaceRoot = Split-Path $robotRoot -Parent
$environmentRoot = Join-Path $workspaceRoot "slam-jetson-env"
$simulationRoot = Join-Path $workspaceRoot "Simulation_env"
$packageRoot = Join-Path $robotRoot "src\hazard_guard_patrol_benchmark"
$composeRoot = Join-Path $PSScriptRoot "docker"
$batchId = Get-Date -Format "yyyyMMddTHHmmss"
$outputRoot = Join-Path $robotRoot "runtime\benchmarks\docker_profiles\$batchId"
New-Item -ItemType Directory -Force -Path $outputRoot | Out-Null

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
$proxyWslPath = (wsl.exe -d $WslDistro -- wslpath -a ($proxyWindowsPath -replace '\\', '/')).Trim()

function Get-ComposeArgs([bool]$Gpu) {
    $arguments = @(
        "compose",
        "--project-directory", $environmentRoot,
        "-f", (Join-Path $environmentRoot "compose.yaml"),
        "-f", (Join-Path $environmentRoot "compose.desktop.yaml"),
        "-f", (Join-Path $composeRoot "compose.benchmark.yaml")
    )
    if ($Gpu) {
        $arguments += @(
            "-f", (Join-Path $environmentRoot "compose.gpu.yaml"),
            "-f", (Join-Path $composeRoot "compose.gpu-wsl.yaml")
        )
    } else {
        $arguments += @("-f", (Join-Path $composeRoot "compose.cpu.yaml"))
    }
    return $arguments
}

function Invoke-Compose([bool]$Gpu, [string[]]$Arguments) {
    $compose = Get-ComposeArgs $Gpu
    & docker @compose @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "docker compose 실패: $($Arguments -join ' ')"
    }
}

function Stop-Profile([bool]$Gpu) {
    try {
        Invoke-Compose $Gpu @("down", "--remove-orphans")
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

function Build-Probe {
    $command = @'
source /opt/ros/humble/setup.bash
source /opt/rosmaster_ws/install/setup.bash 2>/dev/null || true
cd /workspace
colcon --log-base /workspace/log build --symlink-install \
  --base-paths /workspace/hazard_guard_robot/src \
  --build-base /workspace/build \
  --install-base /workspace/install \
  --packages-select hazard_guard_patrol_benchmark
'@
    & docker exec rosmaster-team-dev bash -lc $command
    if ($LASTEXITCODE -ne 0) { throw "프로파일 probe 빌드 실패" }
}

function Wait-Ready {
    $command = @'
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
for _ in $(seq 1 60); do
  if ign topic -l | grep -qx /odom && \
     ign topic -l | grep -qx /scan && \
     test "$(grep -c 'Managed nodes are active' /tmp/profile_launch.log 2>/dev/null || true)" -ge 2; then
    exit 0
  fi
  sleep 2
done
echo 'Gazebo/Nav2 readiness timeout' >&2
exit 1
'@
    & docker exec rosmaster-team-dev bash -lc $command
    if ($LASTEXITCODE -ne 0) {
        docker exec rosmaster-team-dev bash -lc "tail -n 120 /tmp/profile_launch.log" 2>$null
        throw "Gazebo/Nav2 준비 실패"
    }
}

function Invoke-Profile([pscustomobject]$Profile, [int]$RunIndex) {
    Write-Host "[$($Profile.Id) $RunIndex/$($Profile.Runs)] starting" -ForegroundColor Cyan
    Stop-Profile $Profile.Gpu
    Start-X11Proxy
    Invoke-Compose $Profile.Gpu @("up", "-d", "--no-build", "--force-recreate", "dev")
    Build-Probe

    $renderEngine = $Profile.Engine
    $prepare = if ($renderEngine -eq "ogre") {
        "sed 's#<render_engine>ogre2</render_engine>#<render_engine>ogre</render_engine>#' /workspace/Simulation_env/gazebo/worlds/real_factory.sdf >/tmp/profile_world.sdf"
    } else {
        "cp /workspace/Simulation_env/gazebo/worlds/real_factory.sdf /tmp/profile_world.sdf"
    }
    & docker exec rosmaster-team-dev bash -lc $prepare
    if ($LASTEXITCODE -ne 0) { throw "프로필 월드 생성 실패" }

    $launch = @"
source /opt/ros/humble/setup.bash
source /opt/rosmaster_ws/install/setup.bash 2>/dev/null || true
source /workspace/install/setup.bash
exec ros2 launch hazard_guard_simulation localization.launch.py \
  gui:=false \
  world:=/tmp/profile_world.sdf \
  world_name:=real_factory \
  map:=/workspace/Simulation_env/gazebo/maps/real_factory.yaml \
  spawn_x:=0.0 spawn_y:=3.45 spawn_z:=0.05 spawn_yaw:=0.0 \
  include_dispenser:=false \
  heat_source_profile:=/workspace/Simulation_env/gazebo/config/heat_sources/real_factory.json \
  use_person_safety:=false use_thermal_pipeline:=false \
  enable_rgbd_mapping:=false use_performance_monitor:=false \
  >/tmp/profile_launch.log 2>&1
"@
    & docker exec -d rosmaster-team-dev bash -lc $launch
    Wait-Ready

    if ($Profile.Gui) {
        & docker exec -d rosmaster-team-dev bash -lc "ign gazebo -g --render-engine-gui $renderEngine --force-version 6 >/tmp/profile_gui.log 2>&1"
        Start-Sleep -Seconds 5
    }

    $goalDelay = $WarmupSeconds + 2
    $goal = @"
sleep $goalDelay
source /opt/ros/humble/setup.bash
source /workspace/install/setup.bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  '{pose: {header: {frame_id: map}, pose: {position: {x: 4.0, y: 3.45}, orientation: {w: 1.0}}}}' \
  >/tmp/profile_goal.log 2>&1
"@
    & docker exec -d rosmaster-team-dev bash -lc $goal

    $fileName = "{0}-run-{1:D2}.json" -f $Profile.Id, $RunIndex
    $containerOutput = "/workspace/hazard_guard_robot/runtime/benchmarks/docker_profiles/$batchId/$fileName"
    $probeCommand = @(
        "source /opt/ros/humble/setup.bash;",
        "source /workspace/install/setup.bash;",
        "ros2 run hazard_guard_patrol_benchmark docker_profile_probe",
        "--profile-id $($Profile.Id)",
        "--duration $DurationSeconds",
        "--warmup $WarmupSeconds",
        "--output $containerOutput",
        "--render-engine $renderEngine"
    ) -join " "
    if ($Profile.Gui) { $probeCommand += " --gui" }
    if ($Profile.Gpu) { $probeCommand += " --gpu-enabled" }
    & docker exec rosmaster-team-dev bash -lc $probeCommand
    $exitCode = $LASTEXITCODE
    & docker exec rosmaster-team-dev bash -lc "cp /tmp/profile_launch.log '${containerOutput}.launch.log'; test ! -f /tmp/profile_goal.log || cp /tmp/profile_goal.log '${containerOutput}.goal.log'"
    if ($exitCode -ne 0) {
        Write-Warning "$($Profile.Id) run $RunIndex health check failed"
    }
}

$profiles = @()
if ($Mode -in @("all", "cpu")) {
    $profiles += [pscustomobject]@{ Id = "cpu-ogre2-headless"; Gpu = $false; Gui = $false; Engine = "ogre2"; Runs = $HeadlessRuns }
    if ($GuiRuns -gt 0) {
        $profiles += [pscustomobject]@{ Id = "cpu-ogre2-gui"; Gpu = $false; Gui = $true; Engine = "ogre2"; Runs = $GuiRuns }
    }
}
if ($Mode -in @("all", "gpu")) {
    $profiles += [pscustomobject]@{ Id = "gpu-ogre-headless"; Gpu = $true; Gui = $false; Engine = "ogre"; Runs = $HeadlessRuns }
    if ($GuiRuns -gt 0) {
        $profiles += [pscustomobject]@{ Id = "gpu-ogre-gui"; Gpu = $true; Gui = $true; Engine = "ogre"; Runs = $GuiRuns }
    }
}

try {
    foreach ($profile in $profiles) {
        for ($run = 1; $run -le $profile.Runs; $run++) {
            Invoke-Profile $profile $run
        }
    }
} finally {
    if ($profiles.Count -gt 0) { Stop-Profile $profiles[-1].Gpu }
    wsl.exe -d $WslDistro -- bash -lc "pkill -f '$proxyWslPath' || true" 2>$null | Out-Null
}

$inputs = @(Get-ChildItem -LiteralPath $outputRoot -Filter "*.json" | ForEach-Object { $_.FullName })
if ($inputs.Count -eq 0) { throw "생성된 프로파일 결과가 없습니다" }
$env:PYTHONPATH = "$packageRoot;$env:PYTHONPATH"
$summaryPath = Join-Path $outputRoot "summary.json"
& python -m hazard_guard_patrol_benchmark.profile_select @inputs --output $summaryPath
if ($LASTEXITCODE -ne 0) { throw "프로파일 집계 실패" }

$summary = Get-Content -Raw -LiteralPath $summaryPath | ConvertFrom-Json
$recommended = [ordered]@{
    generated_at = (Get-Date).ToString("o")
    batch_id = $batchId
    automated = $summary.recommended.automated
    visual = $summary.recommended.visual
    summary = $summaryPath
}
$recommendedPath = Join-Path (Split-Path $outputRoot -Parent) "recommended-profile.json"
$recommended | ConvertTo-Json | Set-Content -Encoding UTF8 -LiteralPath $recommendedPath
Write-Host "Summary: $summaryPath" -ForegroundColor Green
Write-Host "Recommended: $recommendedPath" -ForegroundColor Green
