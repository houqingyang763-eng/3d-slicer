[CmdletBinding()]
param(
    [string]$DistributionName = 'UR12e-Jazzy',
    [string]$InstallDirectory,
    [string]$ArchivePath
)

$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
$runtime = Join-Path $taskRoot 'runtime'
New-Item -ItemType Directory -Force -Path $runtime | Out-Null
$statusPath = Join-Path $runtime 'prepare_linux_status.json'
$logPath = Join-Path $runtime ('prepare_linux_{0}_{1}.log' -f (Get-Date -Format 'yyyyMMdd_HHmmss'), $PID)
$record = [ordered]@{
    status = 'starting'
    started_at = (Get-Date).ToString('o')
    distribution = $DistributionName
    log_path = $logPath
    reboot_required = $false
    steps = @()
}

function Save-PrepareState {
    $record | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $statusPath -Encoding UTF8
}

function Write-PrepareMessage([string]$Message) {
    $line = '{0} {1}' -f (Get-Date).ToString('o'), $Message
    Add-Content -LiteralPath $logPath -Value $line -Encoding UTF8
    Write-Host $Message
}

function Invoke-LoggedWsl {
    param([string[]]$ArgumentList, [switch]$CaptureOutput)
    Write-PrepareMessage ('wsl.exe ' + ($ArgumentList -join ' '))
    $lines = [System.Collections.Generic.List[string]]::new()
    $savedPreference = $ErrorActionPreference
    try {
        # PS 5.1 may wrap native stderr in ErrorRecord; log it without ending the pipeline.
        $ErrorActionPreference = 'Continue'
        & $script:wslExe @ArgumentList 2>&1 | ForEach-Object {
            $line = $_.ToString().Replace("`0", '')
            # Native stderr (for example WSL proxy warnings) is an ErrorRecord
            # in Windows PowerShell. Keep it in the log, not parsed stdout.
            if ($CaptureOutput -and $_ -isnot [System.Management.Automation.ErrorRecord]) { $lines.Add($line) }
            Add-Content -LiteralPath $logPath -Value $line -Encoding UTF8
            Write-Host $line
        }
        $nativeExitCode = $LASTEXITCODE
    } finally {
        $ErrorActionPreference = $savedPreference
    }
    $record.steps += [ordered]@{
        arguments = $ArgumentList
        exit_code = $nativeExitCode
        finished_at = (Get-Date).ToString('o')
    }
    Save-PrepareState
    return [pscustomobject]@{ ExitCode = $nativeExitCode; Output = ($lines -join "`n") }
}

function ConvertTo-DistributionPath([string]$WindowsPath) {
    $absolutePath = [System.IO.Path]::GetFullPath($WindowsPath)
    if ($absolutePath -notmatch '^[A-Za-z]:\\') {
        throw "A local Windows drive path is required: $absolutePath"
    }
    # One native argument preserves spaces. Do not concatenate a bash -c command.
    $result = Invoke-LoggedWsl -ArgumentList @('-d', $DistributionName, '-u', 'root', '--', 'wslpath', '-a', '-u', $absolutePath) -CaptureOutput
    if ($result.ExitCode -ne 0) { throw "wslpath failed for $absolutePath" }
    $linuxPath = $result.Output.Trim()
    if (-not $linuxPath.StartsWith('/') -or $linuxPath.Contains("`n")) {
        throw "wslpath did not return one absolute Linux path for $absolutePath"
    }
    return $linuxPath
}

Save-PrepareState
try {
    if ($DistributionName -notmatch '^[A-Za-z0-9][A-Za-z0-9._-]{0,63}$') {
        throw 'DistributionName must contain only letters, numbers, dot, underscore or hyphen.'
    }

    $installStatusPath = Join-Path $runtime 'wsl_install_status.json'
    if (Test-Path -LiteralPath $installStatusPath) {
        $installState = Get-Content -Raw -LiteralPath $installStatusPath | ConvertFrom-Json
        if ($installState.status -eq 'failed') {
            throw "Windows WSL installation failed; inspect $installStatusPath before continuing."
        }
        if ($installState.reboot_required -eq $true) {
            $record.reboot_required = $true
            $record.status = 'checking_reboot'
            Save-PrepareState
            if (-not $installState.finished_at) {
                throw 'WSL installation is not finished. Wait for its status file to finish, then rerun.'
            }
            $installFinished = [DateTimeOffset]::Parse([string]$installState.finished_at)
            $lastBoot = [DateTimeOffset](Get-CimInstance Win32_OperatingSystem).LastBootUpTime
            $record.last_boot_at = $lastBoot.ToString('o')
            $record.wsl_install_finished_at = $installFinished.ToString('o')
            if ($lastBoot -le $installFinished) {
                $record.status = 'reboot_required'
                $record.finished_at = (Get-Date).ToString('o')
                Save-PrepareState
                Write-PrepareMessage 'WSL features require one Windows restart. Save your work, restart Windows, then run this same script again. No restart was initiated.'
                exit 3010
            }
            $record.reboot_required = $false
            Write-PrepareMessage 'Windows has restarted since WSL feature installation; continuing.'
        } elseif ($installState.status -notin @('installed', 'reboot_required')) {
            throw "Windows WSL installation is not complete (status=$($installState.status))."
        }
    }

    $script:wslExe = Join-Path $env:SystemRoot 'System32\wsl.exe'
    if (-not (Test-Path -LiteralPath $script:wslExe)) {
        throw 'wsl.exe is unavailable. Complete the Windows WSL installer first.'
    }
    $record.status = 'checking_wsl'
    Save-PrepareState
    $versionResult = Invoke-LoggedWsl -ArgumentList @('--version') -CaptureOutput
    if ($versionResult.ExitCode -ne 0) { throw 'WSL version check failed. See the preparation log.' }
    $listResult = Invoke-LoggedWsl -ArgumentList @('--list', '--quiet') -CaptureOutput
    if ($listResult.ExitCode -ne 0) { throw 'WSL distribution listing failed. See the preparation log.' }
    $distributionNames = @($listResult.Output -split '\r?\n' | ForEach-Object { $_.Trim() } | Where-Object { $_ })

    if ($distributionNames -contains $DistributionName) {
        $record.status = 'reusing_distribution'
        Save-PrepareState
        Write-PrepareMessage "Reusing existing WSL distribution $DistributionName."
    } else {
        if ([string]::IsNullOrWhiteSpace($ArchivePath)) {
            $ArchivePath = Join-Path $taskRoot 'downloads\ubuntu-24.04-amd64.wsl'
        }
        $resolvedArchive = (Resolve-Path -LiteralPath $ArchivePath).Path
        $sourceManifestPath = Join-Path $taskRoot 'assets\ubuntu_source.json'
        $sourceManifest = Get-Content -Raw -LiteralPath $sourceManifestPath | ConvertFrom-Json
        $expectedHash = ([string]$sourceManifest.Amd64Url.Sha256).ToLowerInvariant()
        if ($expectedHash -notmatch '^[a-f0-9]{64}$') {
            throw 'The Ubuntu source manifest does not contain a valid AMD64 SHA256.'
        }
        $record.status = 'verifying_archive'
        $record.archive_path = $resolvedArchive
        $record.archive_source = [string]$sourceManifest.Amd64Url.Url
        $record.archive_size_bytes = (Get-Item -LiteralPath $resolvedArchive).Length
        Save-PrepareState
        $actualHash = (Get-FileHash -LiteralPath $resolvedArchive -Algorithm SHA256).Hash.ToLowerInvariant()
        $record.archive_sha256 = $actualHash
        if ($actualHash -ne $expectedHash) {
            throw 'Ubuntu archive SHA256 differs from the saved official manifest. If its download is still in progress, let that download finish before rerunning. Nothing was imported.'
        }

        if ([string]::IsNullOrWhiteSpace($InstallDirectory)) {
            $InstallDirectory = if (Test-Path -LiteralPath 'D:\') {
                'D:\CodexEnvs\UR12e-Jazzy'
            } else {
                Join-Path $runtime 'wsl_distributions\UR12e-Jazzy'
            }
        }
        $resolvedInstallDirectory = [System.IO.Path]::GetFullPath($InstallDirectory)
        if ($resolvedInstallDirectory -notmatch '^[A-Za-z]:\\') {
            throw 'InstallDirectory must be a local Windows drive path.'
        }
        if (Test-Path -LiteralPath $resolvedInstallDirectory) {
            if (@(Get-ChildItem -Force -LiteralPath $resolvedInstallDirectory).Count -gt 0) {
                throw "The unregistered distribution directory is not empty: $resolvedInstallDirectory. Inspect it before continuing; no files were removed."
            }
        } else {
            New-Item -ItemType Directory -Force -Path $resolvedInstallDirectory | Out-Null
        }
        $record.install_directory = $resolvedInstallDirectory
        $record.status = 'importing_distribution'
        Save-PrepareState
        $importResult = Invoke-LoggedWsl -ArgumentList @('--import', $DistributionName, $resolvedInstallDirectory, $resolvedArchive, '--version', '2')
        if ($importResult.ExitCode -ne 0) { throw 'WSL import failed. Existing data was retained; inspect the log.' }
    }

    $record.status = 'checking_ubuntu'
    Save-PrepareState
    $osResult = Invoke-LoggedWsl -ArgumentList @('-d', $DistributionName, '-u', 'root', '--', 'cat', '/etc/os-release') -CaptureOutput
    if ($osResult.ExitCode -ne 0 -or $osResult.Output -notmatch '(?m)^ID=ubuntu\r?$' -or $osResult.Output -notmatch '(?m)^VERSION_ID="24\.04"\r?$') {
        throw "Distribution $DistributionName must be bootable Ubuntu 24.04. No ROS installation was started."
    }

    $setupScriptPath = ConvertTo-DistributionPath (Join-Path $PSScriptRoot 'setup_ros2.sh')
    $linuxLogDirectory = ConvertTo-DistributionPath (Join-Path $runtime 'ros2_setup')
    $localSourceEnvironment = @()
    $sourceCheck = Invoke-LoggedWsl -ArgumentList @('-d', $DistributionName, '-u', 'root', '--exec', 'dpkg-query', '--show', '--showformat=${Status}', 'ros2-apt-source') -CaptureOutput
    if ($sourceCheck.ExitCode -ne 0 -or $sourceCheck.Output -notmatch 'install ok installed') {
        # Windows can reach official GitHub releases even if the WSL NAT cannot.
        $sourceManifest = Get-Content -LiteralPath (Join-Path $taskRoot 'assets\ros_apt_source.json') -Raw -Encoding UTF8 | ConvertFrom-Json
        $sourceAsset = @($sourceManifest.assets)[0]
        if ($sourceAsset.name -notmatch '^ros2-apt-source_[A-Za-z0-9.+_-]+\.noble_all\.deb$') { throw 'Invalid official ROS apt source package name.' }
        $expectedSourceHash = ([string]$sourceAsset.digest).Replace('sha256:', '').ToLowerInvariant()
        if ($expectedSourceHash -notmatch '^[a-f0-9]{64}$') { throw 'Missing official ROS apt source package SHA256.' }
        $sourcePackagePath = Join-Path (Join-Path $taskRoot 'downloads') $sourceAsset.name
        $sourceIsVerified = (Test-Path -LiteralPath $sourcePackagePath) -and ((Get-FileHash -LiteralPath $sourcePackagePath -Algorithm SHA256).Hash.ToLowerInvariant() -eq $expectedSourceHash)
        if (-not $sourceIsVerified) {
            Write-PrepareMessage 'Downloading the pinned official ROS apt source package through Windows.'
            Invoke-WebRequest -UseBasicParsing -Uri $sourceAsset.browser_download_url -OutFile $sourcePackagePath
        }
        if ((Get-FileHash -LiteralPath $sourcePackagePath -Algorithm SHA256).Hash.ToLowerInvariant() -ne $expectedSourceHash) { throw 'Official ROS apt source package SHA256 mismatch.' }
        $record.ros_apt_source_sha256 = $expectedSourceHash
        $localSourceEnvironment = @('ROS_APT_SOURCE_DEB=' + (ConvertTo-DistributionPath $sourcePackagePath))
    }
    $record.status = 'installing_ros2'
    $record.linux_setup_script = $setupScriptPath
    $record.linux_log_directory = $linuxLogDirectory
    Save-PrepareState
    $setupArguments = @('-d', $DistributionName, '-u', 'root', '--', 'env', "UR12E_SETUP_LOG_DIR=$linuxLogDirectory") + $localSourceEnvironment + @('bash', $setupScriptPath)
    $setupResult = Invoke-LoggedWsl -ArgumentList $setupArguments
    if ($setupResult.ExitCode -ne 0) { throw "ROS 2 package setup failed with exit code $($setupResult.ExitCode). See runtime\ros2_setup and the preparation log." }

    $record.status = 'ready_for_runtime_validation'
    $record.finished_at = (Get-Date).ToString('o')
    Save-PrepareState
    Write-PrepareMessage 'Ubuntu/ROS package checks passed. Runtime planning and Slicer integration still require their own validation.'
    exit 0
} catch {
    $record.status = 'failed'
    $record.error = $_.Exception.Message
    $record.finished_at = (Get-Date).ToString('o')
    Save-PrepareState
    Write-PrepareMessage ('ERROR: ' + $_.Exception.Message)
    exit 1
}
