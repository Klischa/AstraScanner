# AstraScanner - Simple UI Test
Write-Host "=== AstraScanner UI Test ===" -ForegroundColor Magenta

$process = Get-Process -Name "AstraScanner" -ErrorAction SilentlyContinue
if ($process) {
    Write-Host "AstraScanner is running (PID: $($process.Id))" -ForegroundColor Green
} else {
    Write-Host "AstraScanner is not running" -ForegroundColor Red
    exit 1
}

Write-Host "`n=== What You Should See ===" -ForegroundColor Cyan
Write-Host "1. Main window with tabs" -ForegroundColor White
Write-Host "2. Bottom status bar with:" -ForegroundColor White
Write-Host "   - Cloud: X (point count)" -ForegroundColor Gray
Write-Host "   - Live Cloud button (NEW!)" -ForegroundColor Green

Write-Host "`n=== Test Live Cloud Button ===" -ForegroundColor Yellow
Write-Host "Look for 'Live Cloud' button in bottom status bar" -ForegroundColor White
Write-Host "Click it to open real-time viewer window" -ForegroundColor White

Write-Host "`n=== Current Status ===" -ForegroundColor Cyan
Get-Content "C:\AstraScanner\logs\scanner.log" -Tail 3