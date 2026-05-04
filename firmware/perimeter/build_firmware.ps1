# build_firmware.ps1 — GlassHouse v2 perimeter firmware build+flash helper
#
# What it does:
#   Runs `idf.py fullclean` -> `idf.py build` -> (optionally) `idf.py flash`
#   against the perimeter firmware at firmware/perimeter/ using ESP-IDF v5.x.
#
# Prerequisites:
#   - ESP-IDF v5.x installed (https://docs.espressif.com/projects/esp-idf/)
#   - Either:
#       (a) $env:IDF_PATH set before invocation, OR
#       (b) ESP-IDF at the Windows default $env:USERPROFILE\esp\v5.x\esp-idf
#   - IDF Python venv at $env:IDF_TOOLS_PATH\python_env\idf5.x_py3.x_env
#     (defaults to $env:USERPROFILE\.espressif — the ESP-IDF installer default)
#
# Usage:
#   .\build_firmware.ps1                     # default: build + flash on COM7
#   .\build_firmware.ps1 -Port COM9          # build + flash on COM9
#   .\build_firmware.ps1 -NoFlash            # build only, skip flash (safe for
#                                             # machines where COM7 targets the
#                                             # wrong device)
#   .\build_firmware.ps1 -Port COM5 -NoFlash # build only (explicit no-flash)

param(
    [string]$Port = "COM7",
    [switch]$NoFlash
)

# Remove MSYS environment variables that trigger ESP-IDF's MinGW rejection
Remove-Item env:MSYSTEM -ErrorAction SilentlyContinue
Remove-Item env:MSYSTEM_CARCH -ErrorAction SilentlyContinue
Remove-Item env:MSYSTEM_CHOST -ErrorAction SilentlyContinue
Remove-Item env:MSYSTEM_PREFIX -ErrorAction SilentlyContinue
Remove-Item env:MINGW_CHOST -ErrorAction SilentlyContinue
Remove-Item env:MINGW_PACKAGE_PREFIX -ErrorAction SilentlyContinue
Remove-Item env:MINGW_PREFIX -ErrorAction SilentlyContinue

# Escalate soft errors to hard failures so build/flash problems don't cascade silently
$ErrorActionPreference = 'Stop'

# Anchor working directory to this script's location (firmware/perimeter/),
# regardless of where the developer invoked it from.
Set-Location $PSScriptRoot

# Resolve IDF_PATH: prefer explicit env var, else try the default Windows install location.
if (-not $env:IDF_PATH) {
    $defaultIdfPath = Join-Path $env:USERPROFILE "esp\v5.4\esp-idf"
    if (Test-Path $defaultIdfPath) {
        $env:IDF_PATH = $defaultIdfPath
    } else {
        $alt = Join-Path $env:USERPROFILE "esp\v5.2\esp-idf"
        if (Test-Path $alt) {
            $env:IDF_PATH = $alt
        } else {
            Write-Error @"
ESP-IDF not found. Set `$env:IDF_PATH` to your ESP-IDF install, e.g.:
    `$env:IDF_PATH = 'C:\Users\you\esp\v5.4\esp-idf'`
or install ESP-IDF via https://docs.espressif.com/projects/esp-idf/.
Checked defaults: $defaultIdfPath, $alt
"@
            exit 1
        }
    }
}

# Resolve IDF_TOOLS_PATH: prefer explicit env var, else default to the Espressif installer location.
if (-not $env:IDF_TOOLS_PATH) {
    $env:IDF_TOOLS_PATH = Join-Path $env:USERPROFILE ".espressif"
}

# Resolve IDF Python venv. Try common names; fall back to env var if already set.
if (-not $env:IDF_PYTHON_ENV_PATH) {
    $candidates = @(
        (Join-Path $env:IDF_TOOLS_PATH "python_env\idf5.4_py3.11_env"),
        (Join-Path $env:IDF_TOOLS_PATH "python_env\idf5.2_py3.11_env"),
        (Join-Path $env:IDF_TOOLS_PATH "python_env\idf5.4_py3.13_env"),
        (Join-Path $env:IDF_TOOLS_PATH "python_env\idf5.2_py3.13_env")
    )
    foreach ($c in $candidates) {
        if (Test-Path $c) { $env:IDF_PYTHON_ENV_PATH = $c; break }
    }
    if (-not $env:IDF_PYTHON_ENV_PATH) {
        Write-Error @"
Could not find an ESP-IDF Python venv under $env:IDF_TOOLS_PATH\python_env\.
Set `$env:IDF_PYTHON_ENV_PATH` manually, or run ESP-IDF's install script so the
venv is created.
"@
        exit 1
    }
}

# Prefer ESP-IDF's own export.ps1 for PATH setup when available — it handles
# toolchain versioning more robustly than hard-coded paths.
$exportScript = Join-Path $env:IDF_PATH "export.ps1"
if (Test-Path $exportScript) {
    & $exportScript
} else {
    # Fallback: minimal PATH additions, derived from IDF_TOOLS_PATH rather than
    # hard-coded absolute paths.
    $env:PATH = "$env:IDF_PYTHON_ENV_PATH\Scripts;$env:PATH"
}

$python = Join-Path $env:IDF_PYTHON_ENV_PATH "Scripts\python.exe"
$idf    = Join-Path $env:IDF_PATH "tools\idf.py"

Write-Host "=== Cleaning stale build cache ==="
& $python $idf fullclean

Write-Host "=== Building firmware ==="
& $python $idf build

if ($LASTEXITCODE -eq 0) {
    if (-not $NoFlash) {
        Write-Host "=== Build succeeded! Flashing to $Port ==="
        & $python $idf -p $Port flash
    } else {
        Write-Host "=== Build succeeded. Skipping flash (-NoFlash). To flash manually: ==="
        Write-Host "    & '$python' '$idf' -p <COMx> flash"
    }
} else {
    Write-Host "=== Build failed with exit code $LASTEXITCODE ==="
    exit $LASTEXITCODE
}
