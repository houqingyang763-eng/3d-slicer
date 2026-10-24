param([string]$Distro='UR12e-Jazzy')
$ErrorActionPreference='Stop'
$taskRoot=$PSScriptRoot
$runtime=Join-Path $taskRoot 'runtime'
New-Item -ItemType Directory -Force -Path $runtime | Out-Null
$statusFile=Join-Path $runtime 'wsl_install_status.json'
if (Test-Path -LiteralPath $statusFile) {
    $install=Get-Content -LiteralPath $statusFile -Raw -Encoding UTF8 | ConvertFrom-Json
    $lastBoot=(Get-CimInstance Win32_OperatingSystem).LastBootUpTime
    if ($install.reboot_required -and $lastBoot -lt [datetime]$install.finished_at) {
        throw 'Restart Windows once to activate the installed WSL features. Then run bootstrap/prepare_linux.ps1.'
    }
}
function Convert-TaskWslPath([string]$Path) {
    $absolute=[IO.Path]::GetFullPath($Path)
    if ($absolute -notmatch '^([A-Za-z]):\\(.*)$') { throw 'A local Windows drive path is required.' }
    return '/mnt/'+$Matches[1].ToLowerInvariant()+'/'+$Matches[2].Replace('\','/')
}
$wsl=Join-Path $env:ProgramFiles 'WSL\wsl.exe'
if (-not (Test-Path -LiteralPath $wsl)) { $wsl='wsl.exe' }
& $wsl -d $Distro -u root --exec test -f /opt/ros/jazzy/setup.bash
if ($LASTEXITCODE -ne 0) { throw 'Linux/ROS installation is incomplete. Run bootstrap/prepare_linux.ps1 first.' }
$checkScript=Convert-TaskWslPath (Join-Path $taskRoot 'bootstrap/check_simulation.py')
function Get-TaskBackendStatus {
    $statusText=& $wsl -d $Distro -u root --exec python3 $checkScript 2> (Join-Path $runtime 'status_check_stderr.log')
    if ($LASTEXITCODE -ne 0) { throw 'Could not check the dedicated backend process.' }
    return ($statusText | ConvertFrom-Json)
}
$observed=Get-TaskBackendStatus
$backend=$null
if (-not $observed.running) {
    $logArchive=Join-Path $runtime ('backend_logs/'+(Get-Date -Format 'yyyyMMdd_HHmmss_fff'))
    New-Item -ItemType Directory -Force -Path $logArchive | Out-Null
    foreach($logName in @('backend_stdout.log','backend_stderr.log','linux_launch.json','backend_windows_process.json')) {
        $oldLog=Join-Path $runtime $logName
        if(Test-Path -LiteralPath $oldLog){Copy-Item -LiteralPath $oldLog -Destination (Join-Path $logArchive $logName)}
    }
    $script=Convert-TaskWslPath (Join-Path $taskRoot 'bootstrap/run_simulation.sh')
    $arguments=@('-d',$Distro,'-u','root','--exec','bash',('"'+$script+'"'))
    $backend=Start-Process -FilePath $wsl -ArgumentList $arguments -WindowStyle Hidden -PassThru `
        -RedirectStandardOutput (Join-Path $runtime 'backend_stdout.log') `
        -RedirectStandardError (Join-Path $runtime 'backend_stderr.log')
    @{pid=$backend.Id;started_at=(Get-Date).ToString('o');distro=$Distro;mode='mock_hardware'} |
        ConvertTo-Json | Set-Content -LiteralPath (Join-Path $runtime 'backend_windows_process.json') -Encoding UTF8
}
Write-Output 'Waiting for actual ROS joint/TF and MoveIt readiness...'
$deadline=(Get-Date).AddSeconds(90)
while (-not $observed.ready) {
    if($backend -and $backend.HasExited){throw "Backend exited before readiness. See $runtime\backend_stderr.log"}
    if((Get-Date) -gt $deadline){throw 'Backend did not become ready within 90 seconds.'}
    Start-Sleep -Milliseconds 500
    $observed=Get-TaskBackendStatus
}
$existing=@(Get-CimInstance Win32_Process -Filter "Name='SlicerApp-real.exe'" |
    Where-Object { $_.CommandLine -like '*ur12e_slicer*slicer*start_slicer.py*' })
if ($existing.Count -eq 0) {
    $slicer=Join-Path (Split-Path -Parent $taskRoot) 'tools\Slicer-5.12.4\Slicer.exe'
    $frontend=Join-Path $taskRoot 'slicer\start_slicer.py'
    Start-Process -FilePath $slicer -ArgumentList @('--no-splash','--python-script',('"'+$frontend+'"')) -WindowStyle Hidden | Out-Null
}
Write-Output 'Backend ready. Waiting for the dedicated Slicer panel...'
$deadline=(Get-Date).AddSeconds(90)
$frontendReady=$false
while (-not $frontendReady) {
    $sessionPath=Join-Path $runtime 'slicer_bridge/session.json'
    if(Test-Path -LiteralPath $sessionPath) {
        try {
            $session=Get-Content -LiteralPath $sessionPath -Raw -Encoding UTF8 | ConvertFrom-Json
            $sessionStatePath=Join-Path $runtime ('slicer_bridge/state_'+$session.session+'.json')
            $sessionState=Get-Content -LiteralPath $sessionStatePath -Raw -Encoding UTF8 | ConvertFrom-Json
            $panelSession=Get-Content -LiteralPath (Join-Path $runtime 'slicer_session.json') -Raw -Encoding UTF8 | ConvertFrom-Json
            $sessionProcess=Get-CimInstance Win32_Process -Filter "ProcessId=$([int]$session.pid)" -ErrorAction SilentlyContinue
            $frontendReady=($sessionProcess -and $sessionProcess.CommandLine -like '*ur12e_slicer*slicer*start_slicer.py*' -and
                $sessionState.task -eq 'ur12e_slicer' -and ([DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds()/1000-$sessionState.heartbeat) -lt 5 -and
                [int]$panelSession.pid -eq [int]$session.pid -and
                ([DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds()/1000-$panelSession.heartbeat) -lt 5)
        } catch { $frontendReady=$false }
    }
    if((Get-Date) -gt $deadline){throw 'The dedicated Slicer panel did not become ready within 90 seconds.'}
    if(-not $frontendReady){Start-Sleep -Milliseconds 500}
}
@{ready=$true;checked_at=(Get-Date).ToString('o');backend=$observed;slicer_pid=$session.pid;mode='mock_hardware'} |
    ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $runtime 'startup_status.json') -Encoding UTF8
Write-Output "UR12e simulation and Slicer panel are ready. Backend logs: $runtime"
