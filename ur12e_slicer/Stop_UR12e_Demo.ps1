param([string]$Distro='UR12e-Jazzy')
$ErrorActionPreference='Stop'
$taskRoot=$PSScriptRoot
$runtime=Join-Path $taskRoot 'runtime'
$queue=Join-Path $runtime 'slicer_bridge'
$sessionPath=Join-Path $queue 'session.json'
if (Test-Path -LiteralPath $sessionPath) {
    $session=Get-Content -LiteralPath $sessionPath -Raw -Encoding UTF8 | ConvertFrom-Json
    $frontend=Get-CimInstance Win32_Process -Filter "ProcessId=$([int]$session.pid)" -ErrorAction SilentlyContinue
    if ($frontend -and $frontend.Name -eq 'SlicerApp-real.exe' -and $frontend.CommandLine -like '*ur12e_slicer*slicer*start_slicer.py*') {
        $requestId=[guid]::NewGuid().ToString()
        $request=@{id=$requestId;session=$session.session;code="import qt`nqt.QTimer.singleShot(1000, slicer.app.quit)`n_result = {'shutdown_requested': True}"}
        $inbox=Join-Path $queue ('inbox/'+$requestId+'.json')
        $temporary=$inbox+'.tmp'
        [IO.File]::WriteAllText($temporary,($request | ConvertTo-Json -Depth 4),[Text.UTF8Encoding]::new($false))
        Move-Item -LiteralPath $temporary -Destination $inbox
        Write-Output 'Requested normal exit of this task''s Slicer window.'
    }
}
$absolute=[IO.Path]::GetFullPath((Join-Path $taskRoot 'bootstrap/stop_simulation.sh'))
if ($absolute -notmatch '^([A-Za-z]):\\(.*)$') { throw 'A local Windows drive path is required.' }
$linuxScript='/mnt/'+$Matches[1].ToLowerInvariant()+'/'+$Matches[2].Replace('\','/')
$wsl=Join-Path $env:ProgramFiles 'WSL/wsl.exe'
if (-not (Test-Path -LiteralPath $wsl)) { $wsl='wsl.exe' }
& $wsl -d $Distro -u root --exec bash $linuxScript
if ($LASTEXITCODE -ne 0) { throw 'Backend stop did not complete; see the message above.' }
if($frontend) {
    $deadline=(Get-Date).AddSeconds(15)
    while(Get-Process -Id ([int]$session.pid) -ErrorAction SilentlyContinue) {
        if((Get-Date) -gt $deadline){throw 'The dedicated Slicer window did not exit within 15 seconds; no force kill was sent.'}
        Start-Sleep -Milliseconds 200
    }
}
Write-Output 'Verified: the recorded simulation group and dedicated Slicer window have exited.'
