#Requires -Version 7.0
[CmdletBinding()]
param(
    [string]$ManifestPath,
    [string]$CacheDirectory,
    [ValidateRange(1, 12)][int]$ThrottleLimit = 8,
    [ValidateRange(1, 5)][int]$Attempts = 3,
    [ValidateRange(10, 1800)][int]$TimeoutSeconds = 300,
    [string[]]$PackageNames,
    [switch]$RosOnly,
    [ValidateRange(0, 10000)][int]$MaximumPackages = 0,
    [switch]$SmallestFirst
)

$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path -Parent $PSScriptRoot
if (-not $ManifestPath) { $ManifestPath = Join-Path $taskRoot 'runtime/apt_package_uris.json' }
if (-not $CacheDirectory) { $CacheDirectory = Join-Path $taskRoot 'downloads/apt_cache' }
$manifest = Get-Content -LiteralPath $ManifestPath -Raw | ConvertFrom-Json
if ($manifest.schema_version -ne 1) { throw 'Unsupported package manifest schema.' }
$CacheDirectory = [IO.Path]::GetFullPath($CacheDirectory)
New-Item -ItemType Directory -Force -Path $CacheDirectory | Out-Null
$runtime = Join-Path $taskRoot 'runtime'
New-Item -ItemType Directory -Force -Path $runtime | Out-Null
$runName = 'apt_prefetch_{0}_{1}' -f (Get-Date -Format 'yyyyMMdd_HHmmss'), $PID
$resultPath = Join-Path $runtime ($runName + '.json')
$progressPath = Join-Path $runtime ($runName + '.jsonl')
$packages = @($manifest.packages)
if ($PackageNames) {
    foreach ($name in $PackageNames) {
        if ($name -notin $packages.package) { throw "Package is absent from the manifest: $name" }
    }
    $packages = @($packages | Where-Object { $_.package -in $PackageNames })
}
if ($RosOnly) { $packages = @($packages | Where-Object { $_.host -eq 'packages.ros.org' }) }
if ($SmallestFirst) { $packages = @($packages | Sort-Object size) }
if ($MaximumPackages -gt 0) { $packages = @($packages | Select-Object -First $MaximumPackages) }

# Validate the complete selection before creating any network request.
foreach ($package in $packages) {
    $uri = [Uri]$package.uri
    if ($uri.Scheme -notin @('http', 'https') -or $uri.UserInfo -or
        $uri.Host -notin @('packages.ros.org', 'archive.ubuntu.com', 'security.ubuntu.com')) {
        throw "Manifest has an unexpected download origin for $($package.package)."
    }
    if ($package.cache_filename -ne [IO.Path]::GetFileName([string]$package.cache_filename) -or
        $package.cache_filename -notmatch '^[A-Za-z0-9.+~%_-]+\.deb$' -or
        $package.sha256 -notmatch '^[a-fA-F0-9]{64}$' -or [long]$package.size -le 0) {
        throw "Manifest has an invalid file/hash/size for $($package.package)."
    }
    if (-not @($package.trusted_origins | Where-Object {
        $_.trusted -eq $true -and $_.site -eq $uri.Host -and $_.origin -in @('ROS', 'Ubuntu')
    }).Count) {
        throw "Manifest lacks a trusted apt origin for $($package.package)."
    }
}

$started = Get-Date
Write-Host ('Prefetching {0} packages, {1:N1} MiB; concurrency {2}.' -f
    $packages.Count, (($packages | Measure-Object size -Sum).Sum / 1MB), $ThrottleLimit)
$results = @($packages | ForEach-Object -ThrottleLimit $ThrottleLimit -Parallel {
    $package = $_
    $destination = Join-Path $using:CacheDirectory $package.cache_filename
    $partial = $destination + '.prefetch-' + [guid]::NewGuid().ToString('N') + '.part'
    $timer = [Diagnostics.Stopwatch]::StartNew()
    $status = 'failed'
    $errorType = $null
    $attemptUsed = 0
    $downloadedBytes = 0L
    try {
        if ((Test-Path -LiteralPath $destination) -and
            (Get-Item -LiteralPath $destination).Length -eq [long]$package.size -and
            (Get-FileHash -LiteralPath $destination -Algorithm SHA256).Hash -eq $package.sha256) {
            $status = 'cached_verified'
        } else {
            for ($attempt = 1; $attempt -le $using:Attempts; $attempt++) {
                $attemptUsed = $attempt
                try {
                    # Invoke-WebRequest uses the configured Windows/.NET network settings.
                    # Do not print proxy settings, credentials, or exception messages.
                    $ProgressPreference = 'SilentlyContinue'
                    Invoke-WebRequest -Uri $package.uri -OutFile $partial -TimeoutSec $using:TimeoutSeconds | Out-Null
                    $downloadedBytes = (Get-Item -LiteralPath $partial).Length
                    if ($downloadedBytes -ne [long]$package.size) { throw [IO.InvalidDataException]::new('Size mismatch') }
                    if ((Get-FileHash -LiteralPath $partial -Algorithm SHA256).Hash -ne $package.sha256) {
                        throw [IO.InvalidDataException]::new('SHA256 mismatch')
                    }
                    Move-Item -LiteralPath $partial -Destination $destination -Force
                    $status = 'downloaded_verified'
                    $errorType = $null
                    break
                } catch {
                    $errorType = $_.Exception.GetType().Name
                    if (Test-Path -LiteralPath $partial) { Remove-Item -LiteralPath $partial -Force }
                    if ($attempt -lt $using:Attempts) { Start-Sleep -Seconds ([math]::Min(8, [math]::Pow(2, $attempt))) }
                }
            }
        }
    } catch {
        $errorType = $_.Exception.GetType().Name
    } finally {
        if (Test-Path -LiteralPath $partial) { Remove-Item -LiteralPath $partial -Force }
    }
    [pscustomobject]@{
        package = $package.package
        version = $package.version
        host = $package.host
        cache_filename = $package.cache_filename
        status = $status
        indexed_bytes = [long]$package.size
        downloaded_bytes = $downloadedBytes
        sha256 = $package.sha256
        attempts = $attemptUsed
        seconds = [math]::Round($timer.Elapsed.TotalSeconds, 3)
        error_type = $errorType
    }
} | ForEach-Object {
    $_ | ConvertTo-Json -Compress | Add-Content -LiteralPath $progressPath -Encoding utf8
    Write-Host ('{0}: {1} ({2:N1} KiB, {3:N2}s)' -f $_.status, $_.package, ($_.indexed_bytes / 1KB), $_.seconds)
    $_
})
$failed = @($results | Where-Object status -eq 'failed')
$record = [ordered]@{
    started_at = $started.ToString('o')
    finished_at = (Get-Date).ToString('o')
    seconds = [math]::Round(((Get-Date) - $started).TotalSeconds, 3)
    manifest_path = [IO.Path]::GetFullPath($ManifestPath)
    cache_directory = $CacheDirectory
    throttle_limit = $ThrottleLimit
    selected_count = $packages.Count
    verified_count = @($results | Where-Object status -ne 'failed').Count
    failed_count = $failed.Count
    results = $results
}
$record | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath $resultPath -Encoding utf8
Write-Host ('Verified {0}/{1}; failures {2}; report {3}' -f
    $record.verified_count, $record.selected_count, $record.failed_count, $resultPath)
if ($failed.Count -gt 0) { exit 1 }
