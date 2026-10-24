param([switch]$NewInstance)
$ErrorActionPreference = 'Stop'
$projectDirectory = Split-Path -Parent $PSScriptRoot
$repositoryDirectory = Split-Path -Parent $projectDirectory
$sessionFile = Join-Path $projectDirectory 'runtime\slicer_session.json'
if (-not $NewInstance -and (Test-Path -LiteralPath $sessionFile)) {
    try {
        $sessionInfo = Get-Content -LiteralPath $sessionFile -Raw | ConvertFrom-Json
        $sessionAge = [DateTimeOffset]::UtcNow.ToUnixTimeSeconds() - [double]$sessionInfo.heartbeat
        $existingSlicer = Get-Process -Id ([int]$sessionInfo.pid) -ErrorAction SilentlyContinue
        if ($existingSlicer -and $existingSlicer.ProcessName -eq 'SlicerApp-real' -and $sessionAge -lt 10) {
            Write-Host "UR12e Slicer is already running (PID $($sessionInfo.pid))."
            return
        }
    } catch {
        Write-Verbose 'No active UR12e Slicer session was found.'
    }
}
$slicerExecutable = Join-Path $repositoryDirectory 'tools\Slicer-5.12.4\Slicer.exe'
if (-not (Test-Path -LiteralPath $slicerExecutable)) {
    throw "Slicer was not found at $slicerExecutable"
}
$startupScript = Join-Path $PSScriptRoot 'start_slicer.py'
Start-Process -FilePath $slicerExecutable -ArgumentList @('--no-splash', '--python-script', ('"' + $startupScript + '"')) -WindowStyle Hidden
Write-Host 'Opening the UR12e simulation panel in 3D Slicer.'
