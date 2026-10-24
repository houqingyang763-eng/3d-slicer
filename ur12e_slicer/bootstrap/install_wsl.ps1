param([Parameter(Mandatory=$true)][string]$MsiPath)
$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
$runtime = Join-Path $taskRoot 'runtime'
New-Item -ItemType Directory -Force -Path $runtime | Out-Null
$statusPath = Join-Path $runtime 'wsl_install_status.json'
$record = [ordered]@{status='starting';started_at=(Get-Date).ToString('o');steps=@();reboot_required=$false}
function Save-InstallState { $record | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath $statusPath -Encoding UTF8 }
Save-InstallState
try {
    $principal = [Security.Principal.WindowsPrincipal]::new([Security.Principal.WindowsIdentity]::GetCurrent())
    if (-not $principal.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) { throw 'This installer needs an elevated Windows process.' }
    $package = (Resolve-Path -LiteralPath $MsiPath).Path
    $expectedHash = 'ae977286974255bd26bf925f00f40f5692c6b2084155cbc0563c09aea091f211'
    $actualHash = (Get-FileHash -LiteralPath $package -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($actualHash -ne $expectedHash) { throw 'Official WSL 3.0.3 x64 package hash does not match.' }
    $record.status = 'installing_wsl_msi'; Save-InstallState
    $msiLog = Join-Path $runtime 'wsl_msi_install.log'
    $arguments = @('/i',('"'+$package+'"'),'/qn','/norestart','/L*v',('"'+$msiLog+'"'))
    $installer = Start-Process -FilePath (Join-Path $env:SystemRoot 'System32\msiexec.exe') -ArgumentList $arguments -Wait -PassThru -WindowStyle Hidden
    $record.steps += [ordered]@{step='wsl_msi';exit_code=$installer.ExitCode}
    if ($installer.ExitCode -eq 3010) { $record.reboot_required = $true }
    elseif ($installer.ExitCode -ne 0) { throw "WSL MSI failed with exit code $($installer.ExitCode). See $msiLog" }
    foreach ($featureName in @('VirtualMachinePlatform','Microsoft-Windows-Subsystem-Linux')) {
        $record.status = "enabling_$featureName"; Save-InstallState
        $feature = Get-WindowsOptionalFeature -Online -FeatureName $featureName
        if ($feature.State -eq 'Enabled') {
            $record.steps += [ordered]@{step=$featureName;state='already_enabled'}
        } else {
            $result = Enable-WindowsOptionalFeature -Online -FeatureName $featureName -All -NoRestart
            $record.steps += [ordered]@{step=$featureName;state=[string]$result.Online;restart_needed=[bool]$result.RestartNeeded}
            if ($result.RestartNeeded) { $record.reboot_required=$true }
        }
        Save-InstallState
    }
    $record.status = $(if ($record.reboot_required) {'reboot_required'} else {'installed'})
    $record.finished_at=(Get-Date).ToString('o')
    Save-InstallState
} catch {
    $record.status='failed'; $record.error=$_.Exception.Message; $record.finished_at=(Get-Date).ToString('o')
    Save-InstallState
    exit 1
}
