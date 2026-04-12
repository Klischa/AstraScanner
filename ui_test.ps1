# AstraScanner - Final UI Test
Write-Host "=== AstraScanner UI Test ===" -ForegroundColor Magenta

# Check process
$process = Get-Process -Name "AstraScanner" -ErrorAction SilentlyContinue
if ($process) {
    Write-Host "✅ AstraScanner is running (PID: $($process.Id))" -ForegroundColor Green
    Write-Host "   Memory: $([math]::Round($process.WorkingSet / 1MB, 2)) MB" -ForegroundColor Gray
} else {
    Write-Host "❌ AstraScanner is not running" -ForegroundColor Red
    exit 1
}

Write-Host "`n=== UI Elements Check ===" -ForegroundColor Cyan

# Check if LiveCloudWindow was compiled
$liveCloudCompiled = Test-Path "C:\AstraScanner\build\Release\AstraScanner.exe"
$loggerCompiled = Test-Path "C:\AstraScanner\src\utils\Logger.h"

Write-Host "LiveCloudWindow compiled: $(if ($liveCloudCompiled) { 'YES' } else { 'NO' })" -ForegroundColor $(if ($liveCloudCompiled) { 'Green' } else { 'Red' })
Write-Host "Logger compiled: $(if ($loggerCompiled) { 'YES' } else { 'NO' })" -ForegroundColor $(if ($loggerCompiled) { 'Green' } else { 'Red' })

# Check logs for proper initialization
$logContent = Get-Content "C:\AstraScanner\logs\scanner.log" -Tail 20
$initMessages = $logContent | Select-String -Pattern "Visualizer setup complete|Application started" | Measure-Object | Select-Object -ExpandProperty Count

Write-Host "Initialization messages: $initMessages" -ForegroundColor $(if ($initMessages -gt 0) { 'Green' } else { 'Red' })

Write-Host "`n=== What You Should See in AstraScanner ===" -ForegroundColor Yellow
Write-Host "1. Main window with tabs: Главная, Калибровка, Обработка, Экспорт" -ForegroundColor White
Write-Host "2. Bottom status bar with:" -ForegroundColor White
Write-Host "   - Cloud: X (point count)" -ForegroundColor Gray
Write-Host "   - Distance indicator" -ForegroundColor Gray
Write-Host "   - FPS display" -ForegroundColor Gray
Write-Host "   - Frame count" -ForegroundColor Gray
Write-Host "   - ⬜ Live Cloud (button) ← THIS IS THE NEW BUTTON!" -ForegroundColor Green

Write-Host "`n=== How to Test Live Cloud ===" -ForegroundColor Cyan
Write-Host "1. Look at the bottom status bar of AstraScanner window" -ForegroundColor White
Write-Host "2. Find the button labeled 'Live Cloud'" -ForegroundColor White
Write-Host "3. Click it - it should become 'Live Cloud (ON)'" -ForegroundColor White
Write-Host "4. A new window should open for real-time point cloud viewing" -ForegroundColor White
Write-Host "5. Start scanning to see the cloud update in real-time" -ForegroundColor White

Write-Host "`n=== Troubleshooting ===" -ForegroundColor Yellow
Write-Host "If you don't see the Live Cloud button:" -ForegroundColor White
Write-Host "1. Make sure AstraScanner window is not minimized" -ForegroundColor Gray
Write-Host "2. Check that the status bar is visible (View → Status Bar)" -ForegroundColor Gray
Write-Host "3. Try resizing the window" -ForegroundColor Gray
Write-Host "4. Restart AstraScanner if needed" -ForegroundColor Gray

Write-Host "`n=== Current Log Status ===" -ForegroundColor Cyan
Get-Content "C:\AstraScanner\logs\scanner.log" -Tail 3 | ForEach-Object {
    Write-Host "  $_" -ForegroundColor Gray
}

Write-Host "If you still don't see it, let me know what you see in the status bar." -ForegroundColor White