# AstraScanner - Quick Test Script
Write-Host "=== AstraScanner Quick Test ===" -ForegroundColor Green

$root = "C:\AstraScanner"
$ok = $true

Write-Host "`nChecking source files..." -ForegroundColor Cyan
$files = @(
    "CMakeLists.txt",
    "src/main.cpp",
    "src/gui/MainWindow.cpp",
    "src/capture/AstraCamera.cpp",
    "src/calibration/CameraCalibrator.cpp",
    "src/filters/PointCloudFilters.cpp",
    "src/export/ExportManager.cpp",
    "src/settings/SettingsManager.cpp"
)
foreach ($f in $files) {
    $p = Join-Path $root $f
    if (Test-Path $p) { Write-Host "  OK $f" -ForegroundColor Green }
    else { Write-Host "  MISSING $f" -ForegroundColor Red; $ok = $false }
}

Write-Host "`nChecking executable..." -ForegroundColor Cyan
$exe1 = Join-Path $root "build\Release\AstraScanner.exe"
$exe2 = Join-Path $root "out\build\Release\Release\AstraScanner.exe"
if (Test-Path $exe1) { Write-Host "  OK build\Release\AstraScanner.exe" -ForegroundColor Green }
elseif (Test-Path $exe2) { Write-Host "  OK out\build\Release\Release\AstraScanner.exe" -ForegroundColor Green }
else { Write-Host "  Executable not found" -ForegroundColor Red; $ok = $false }

Write-Host "`nChecking logs..." -ForegroundColor Cyan
$logs = Join-Path $root "logs"
if (Test-Path $logs) {
    $logFiles = Get-ChildItem $logs -Filter "*.log"
    if ($logFiles.Count -gt 0) { Write-Host "  OK $($logFiles.Count) log file(s)" -ForegroundColor Green }
    else { Write-Host "  No log files" -ForegroundColor Yellow }
} else { Write-Host "  logs directory missing" -ForegroundColor Red }

Write-Host "`nChecking directories..." -ForegroundColor Cyan
$dirs = @("data", "projects")
foreach ($d in $dirs) {
    $path = Join-Path $root $d
    if (Test-Path $path) { Write-Host "  OK $d/" -ForegroundColor Green }
    else { Write-Host "  MISSING $d/" -ForegroundColor Red; $ok = $false }
}

Write-Host "`n=== Result ===" -ForegroundColor Magenta
if ($ok) { Write-Host "SUCCESS: All critical components found." -ForegroundColor Green }
else { Write-Host "FAILURE: Some components missing." -ForegroundColor Red }
Write-Host "=== End ===" -ForegroundColor Magenta
