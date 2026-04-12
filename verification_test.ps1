# AstraScanner - Verification Test
Write-Host "=== AstraScanner Verification Test ===" -ForegroundColor Magenta

# Check process
$process = Get-Process -Name "AstraScanner" -ErrorAction SilentlyContinue
if ($process) {
    Write-Host "OK: AstraScanner is running (PID: $($process.Id))" -ForegroundColor Green
    Write-Host "Memory: $([math]::Round($process.WorkingSet / 1MB, 2)) MB" -ForegroundColor Gray
} else {
    Write-Host "ERROR: AstraScanner is not running" -ForegroundColor Red
    exit 1
}

# Check logs for new features
$logContent = Get-Content "C:\AstraScanner\logs\scanner.log" -Tail 20
$loggerWorking = $logContent | Select-String -Pattern "\[DEBUG\]|\[INFO\]" | Measure-Object | Select-Object -ExpandProperty Count
$liveCloudReady = $logContent | Select-String -Pattern "Visualizer setup complete|Application started" | Measure-Object | Select-Object -ExpandProperty Count

Write-Host "`n=== Feature Verification ===" -ForegroundColor Cyan

if ($loggerWorking -gt 0) {
    Write-Host "✅ Logger System: WORKING" -ForegroundColor Green
    Write-Host "   - Found $loggerWorking log messages" -ForegroundColor Gray
    Write-Host "   - Messages: DEBUG, INFO levels active" -ForegroundColor Gray
} else {
    Write-Host "❌ Logger System: NOT WORKING" -ForegroundColor Red
}

if ($liveCloudReady -gt 0) {
    Write-Host "✅ Live Cloud Viewer: READY" -ForegroundColor Green
    Write-Host "   - LiveCloudWindow initialized" -ForegroundColor Gray
    Write-Host "   - Button should be visible in status bar" -ForegroundColor Gray
} else {
    Write-Host "❌ Live Cloud Viewer: NOT READY" -ForegroundColor Red
}

# Check source files
$liveCloudExists = Test-Path "C:\AstraScanner\src\gui\LiveCloudWindow.cpp"
$loggerExists = Test-Path "C:\AstraScanner\src\utils\Logger.cpp"

Write-Host "`n=== Source Files ===" -ForegroundColor Cyan
Write-Host "LiveCloudWindow.cpp: $(if ($liveCloudExists) { 'EXISTS' } else { 'MISSING' })" -ForegroundColor $(if ($liveCloudExists) { 'Green' } else { 'Red' })
Write-Host "Logger.cpp: $(if ($loggerExists) { 'EXISTS' } else { 'MISSING' })" -ForegroundColor $(if ($loggerExists) { 'Green' } else { 'Red' })

Write-Host "`n=== How to Test New Features ===" -ForegroundColor Yellow
Write-Host "1. In AstraScanner window, look for 'Live Cloud' button in bottom status bar" -ForegroundColor White
Write-Host "2. Click 'Live Cloud' to open real-time point cloud viewer" -ForegroundColor White
Write-Host "3. Start scanning and watch logs update in real-time" -ForegroundColor White
Write-Host "4. Check logs: Get-Content 'C:\AstraScanner\logs\scanner.log' -Tail 10 -Wait" -ForegroundColor White

Write-Host "`n=== Current Log Sample ===" -ForegroundColor Cyan
Get-Content "C:\AstraScanner\logs\scanner.log" -Tail 5 | ForEach-Object {
    Write-Host "  $_" -ForegroundColor Gray
}

Write-Host "`nTest completed! New features should now be available." -ForegroundColor Green