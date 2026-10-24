param([string]$Distro='UR12e-Jazzy')
$ErrorActionPreference='Stop'
$navTaskRoot=$PSScriptRoot
$navRepository=Split-Path -Parent $navTaskRoot
$navPython=Join-Path $navRepository 'tools\Slicer-5.12.4\bin\PythonSlicer.exe'
if(-not (Test-Path -LiteralPath $navPython)){throw 'Slicer Python was not found.'}
function Convert-NavWslPath([string]$Path) {
    $navAbsolute=[IO.Path]::GetFullPath($Path)
    if($navAbsolute -notmatch '^([A-Za-z]):\\(.*)$'){throw 'A local drive path is required.'}
    return '/mnt/'+$Matches[1].ToLowerInvariant()+'/'+$Matches[2].Replace('\','/')
}
$navStamp=(Get-Date -Format 'yyyyMMdd_HHmmss')+'_'+[guid]::NewGuid().ToString('N').Substring(0,6)
$navOffline=Join-Path $navTaskRoot ('output\navigation_offline\'+$navStamp)
New-Item -ItemType Directory -Path $navOffline -Force | Out-Null
Write-Output 'Running deterministic geometry and fault scenarios...'
& $navPython (Join-Path $navTaskRoot 'navigation_benchmark\run_offline.py') --output $navOffline
if($LASTEXITCODE -ne 0){throw 'Offline navigation checks failed. Their records were preserved.'}
& $navPython (Join-Path $navTaskRoot 'validation\evaluate_recorded_points.py') --input (Join-Path $navTaskRoot 'research\benchmark_evidence\astm_audit\analytic_input_example.json') --output (Join-Path $navOffline 'analytic_point_statistics.json')
if($LASTEXITCODE -ne 0){throw 'Analytic point-statistics check failed.'}
& (Join-Path $navTaskRoot 'Start_UR12e_Demo.ps1') -Distro $Distro
$navClient=Join-Path $navTaskRoot 'slicer\task_client.py'
try {
    & $navPython $navClient (Join-Path $navTaskRoot 'validation\prepare_navigation_validation.py') --timeout 30
    if($LASTEXITCODE -ne 0){throw 'Could not acquire the Slicer manual-input lock.'}
    $navLinuxScript=Convert-NavWslPath (Join-Path $navTaskRoot 'validation\run_navigation_integration.py')
    Write-Output 'Running image/optical calibration through genuine MoveIt on mock hardware...'
    & wsl.exe -d $Distro -u root --exec python3 $navLinuxScript
    if($LASTEXITCODE -ne 0){throw 'MoveIt navigation validation failed. Its failed run was preserved.'}
    & $navPython $navClient (Join-Path $navTaskRoot 'validation\show_navigation_result.py') --timeout 90
    if($LASTEXITCODE -ne 0){throw 'Slicer result scene could not be built.'}
    @{offline_report=(Join-Path $navOffline 'report.html');integration_result=(Join-Path $navTaskRoot 'output\navigation_benchmark\latest_result.json');finished_at=(Get-Date).ToString('o');simulation_only=$true} |
        ConvertTo-Json | Set-Content -LiteralPath (Join-Path $navTaskRoot 'output\navigation_benchmark\latest_bundle.json') -Encoding UTF8
    Write-Output "Navigation evaluation completed. Offline report: $navOffline\report.html"
    Write-Output 'The mock backend remains available. Use Stop_UR12e_Demo.cmd to close it.'
} finally {
    & $navPython $navClient (Join-Path $navTaskRoot 'validation\release_navigation_validation.py') --timeout 15
}
