param(
    [ValidateSet('start','status','stop','run')][string]$Action='start',
    [string]$Distribution,
    [string]$LinuxUser
)
$ErrorActionPreference='Stop'
$localConfig=Join-Path $PSScriptRoot 'runtime\ursim_windows.json'
if (Test-Path -LiteralPath $localConfig) {
    $config=Get-Content -Raw -LiteralPath $localConfig | ConvertFrom-Json
    if (-not $Distribution) {$Distribution=$config.distribution}
    if (-not $LinuxUser) {$LinuxUser=$config.user}
}
if (-not $Distribution -or -not $LinuxUser) {
    throw 'Supply -Distribution <WSL name> -LinuxUser <ordinary Linux user>. The optional runtime/ursim_windows.json is machine-local and is not published.'
}
$linuxProject=(& wsl.exe -d $Distribution -u $LinuxUser -- wslpath -u $PSScriptRoot | Out-String).Trim()
if ($LASTEXITCODE -ne 0 -or -not $linuxProject) {throw 'Cannot resolve the project path in the selected WSL environment.'}
if ($Action -eq 'start') {
    if ($Distribution -match '\s' -or $LinuxUser -match '\s') {
        throw 'For this WSL launcher use a distribution and ordinary user name without spaces; alternatively run the documented Linux entrypoint in a terminal.'
    }
    $logs=Join-Path $PSScriptRoot 'output\ursim_windows'
    New-Item -ItemType Directory -Force -Path $logs | Out-Null
    $stamp=Get-Date -Format 'yyyyMMdd_HHmmss_fff'
    $script="$linuxProject/bootstrap/ursim_wsl_session.sh"
    # WSL's detached command-line parser retains explicit quote characters in
    # some versions. Use an owned no-space Linux symlink, with distinct arguments.
    $linuxHome=(& wsl.exe -d $Distribution -u $LinuxUser -- printenv HOME | Out-String).Trim()
    if ($LASTEXITCODE -ne 0 -or -not $linuxHome.StartsWith('/') -or $linuxHome -match '\s') {throw 'Cannot resolve an ordinary Linux home directory.'}
    $sha=[Security.Cryptography.SHA256]::Create()
    try {$key=([BitConverter]::ToString($sha.ComputeHash([Text.Encoding]::UTF8.GetBytes($linuxProject)))).Replace('-','').Substring(0,12).ToLowerInvariant()} finally {$sha.Dispose()}
    $aliasDir="$linuxHome/.cache/ur12e_slicer/windows"
    $alias="$aliasDir/session_$key.sh"
    & wsl.exe -d $Distribution -u $LinuxUser -- mkdir -p -- $aliasDir
    if ($LASTEXITCODE -ne 0) {throw 'Cannot prepare the WSL launcher cache.'}
    $existing=(& wsl.exe -d $Distribution -u $LinuxUser -- readlink -- $alias 2>$null | Out-String).Trim()
    if ($existing -and $existing -ne $script) {throw 'The local WSL launcher alias points to another file; no replacement was made.'}
    if (-not $existing) {
        & wsl.exe -d $Distribution -u $LinuxUser -- ln -s -- $script $alias
        if ($LASTEXITCODE -ne 0) {throw 'Cannot create the local WSL launcher alias.'}
    }
    $arguments=@('-d',$Distribution,'-u',$LinuxUser,'--','bash',$alias)
    $child=Start-Process -FilePath 'wsl.exe' -ArgumentList $arguments -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $logs "$stamp.stdout.log") -RedirectStandardError (Join-Path $logs "$stamp.stderr.log")
    Write-Output "URSim startup dispatched (Windows PID $($child.Id)). Logs: $logs"
    Write-Output 'Use URSim_Windows.ps1 -Action status to verify readiness. Virtual pendant: http://localhost:6080/vnc.html'
} else {
    $entry=@{status='status_ursim_linux.sh';stop='stop_ursim_linux.sh';run='run_ursim_linux.sh'}[$Action]
    & wsl.exe -d $Distribution -u $LinuxUser -- bash "$linuxProject/$entry"
    if ($LASTEXITCODE -ne 0) {throw "URSim $Action returned exit code $LASTEXITCODE; inspect the retained logs."}
    if ($Action -eq 'stop') {
        & wsl.exe -d $Distribution -u $LinuxUser -- bash "$linuxProject/bootstrap/ursim_wsl_session.sh" --wait-stopped
        if ($LASTEXITCODE -ne 0) {throw 'URSim components stopped, but the recorded WSL guardian exit was not confirmed; inspect its owner record and retained logs.'}
    }
}
