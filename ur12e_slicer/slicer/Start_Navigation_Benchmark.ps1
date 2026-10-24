param([string]$ResultPath)
$ErrorActionPreference = 'Stop'
$projectDirectory = Split-Path -Parent $PSScriptRoot
$repositoryDirectory = Split-Path -Parent $projectDirectory
if (-not $ResultPath) {
    $ResultPath = Join-Path $projectDirectory 'output\navigation_benchmark\latest_result.json'
}
$resolvedResult = (Resolve-Path -LiteralPath $ResultPath).Path
$slicerExecutable = Join-Path $repositoryDirectory 'tools\Slicer-5.12.4\Slicer.exe'
if (-not (Test-Path -LiteralPath $slicerExecutable)) {
    throw "Slicer was not found at $slicerExecutable"
}
$startupScript = Join-Path $PSScriptRoot 'navigation_benchmark_scene.py'
$previousResult = $env:UR12E_NAVBENCH_RESULT
try {
    $env:UR12E_NAVBENCH_RESULT = $resolvedResult
    $viewerProcess = Start-Process -FilePath $slicerExecutable -ArgumentList @('--no-splash', '--python-script', ('"' + $startupScript + '"')) -WindowStyle Hidden -PassThru
    Write-Host "Opening a separate read-only simulation result viewer (launcher PID $($viewerProcess.Id))."
    Write-Host 'No bridge or robot backend is started. The Slicer application PID is recorded in runtime\navigation_benchmark_viewer.json.'
} finally {
    if ($null -eq $previousResult) {
        Remove-Item Env:\UR12E_NAVBENCH_RESULT -ErrorAction SilentlyContinue
    } else {
        $env:UR12E_NAVBENCH_RESULT = $previousResult
    }
}
