# AstraScanner - Processing Test
Write-Host "=== AstraScanner Processing Test ===" -ForegroundColor Cyan

# Check if AstraScanner is running
$process = Get-Process -Name "AstraScanner" -ErrorAction SilentlyContinue
if ($process) {
    Write-Host "OK: AstraScanner is running (PID: $($process.Id))" -ForegroundColor Green
    Write-Host "Memory usage: $([math]::Round($process.WorkingSet / 1MB, 2)) MB" -ForegroundColor Gray
} else {
    Write-Host "ERROR: AstraScanner is not running" -ForegroundColor Red
    Write-Host "Please start AstraScanner first" -ForegroundColor Yellow
    exit 1
}

Write-Host "`n=== Available Processing Filters ===" -ForegroundColor Cyan
Write-Host "1. Statistical Outlier Removal (SOR)" -ForegroundColor White
Write-Host "   - Removes noise points using statistical analysis" -ForegroundColor Gray
Write-Host "   - Parameters: Mean K (neighbors), StdDev threshold" -ForegroundColor Gray

Write-Host "2. Radius Outlier Removal (ROR)" -ForegroundColor White
Write-Host "   - Removes isolated points based on radius" -ForegroundColor Gray
Write-Host "   - Parameters: Radius, Min neighbors" -ForegroundColor Gray

Write-Host "3. Voxel Grid Downsampling" -ForegroundColor White
Write-Host "   - Reduces point density using voxel grid" -ForegroundColor Gray
Write-Host "   - Parameters: Voxel size" -ForegroundColor Gray

Write-Host "4. Magic Wand Selection" -ForegroundColor White
Write-Host "   - Interactive region selection tool" -ForegroundColor Gray
Write-Host "   - Click and drag to select regions" -ForegroundColor Gray

Write-Host "`n=== How to Test Processing ===" -ForegroundColor Cyan
Write-Host "1. Start scanning in AstraScanner (click 'Scan')" -ForegroundColor White
Write-Host "2. Wait for point cloud to accumulate (~10-30 seconds)" -ForegroundColor White
Write-Host "3. Go to 'Обработка' (Processing) tab" -ForegroundColor White
Write-Host "4. Try each filter:" -ForegroundColor White
Write-Host "   - SOR: Set Mean K=50, StdDev=1.0, click 'Применить'" -ForegroundColor Gray
Write-Host "   - ROR: Set Radius=0.05, Min Neighbors=10, click 'Применить'" -ForegroundColor Gray
Write-Host "   - Voxel: Set size=0.01, click 'Применить'" -ForegroundColor Gray
Write-Host "   - Magic Wand: Click on unwanted regions" -ForegroundColor Gray

Write-Host "`n=== Monitoring Processing ===" -ForegroundColor Cyan
Write-Host "Watch the logs for processing messages:" -ForegroundColor White

# Monitor logs in real-time
Write-Host "`nStarting log monitoring (Ctrl+C to stop)..." -ForegroundColor Yellow
Write-Host "Look for messages like:" -ForegroundColor Gray
Write-Host "  [INFO] SOR: 15432 -> 14256 points" -ForegroundColor Gray
Write-Host "  [INFO] ROR: 14256 -> 13890 points" -ForegroundColor Gray
Write-Host "  [INFO] Voxel: 13890 -> 8756 points" -ForegroundColor Gray

try {
    Get-Content "C:\AstraScanner\logs\scanner.log" -Tail 10 -Wait
} catch {
    Write-Host "Log monitoring stopped" -ForegroundColor Yellow
}

Write-Host "`n=== Processing Test Complete ===" -ForegroundColor Green
Write-Host "Try the filters in AstraScanner and watch the point count changes!" -ForegroundColor White