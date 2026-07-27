param(
    [string]$RobotStudioRoot = "D:\Software\Industrial Software\Mechanical engineering\Robot\RobotStudio_6.08.01"
)

$ErrorActionPreference = "Stop"
$experimentRoot = Split-Path -Parent $PSScriptRoot
$project = Join-Path $experimentRoot "robotstudio_addin\ABB6700.RobotStudioExport.csproj"
$buildDirectory = Join-Path $experimentRoot "robotstudio_addin\bin\Release"
$addinSource = Join-Path $experimentRoot "robotstudio_addin\ABB6700.RobotStudioExport.rsaddin"
$installDirectory = Join-Path $RobotStudioRoot "Bin\Addins"
$addinManifestDestination = Join-Path $installDirectory "ABB6700.RobotStudioExport.rsaddin"
$staleInstallDirectory64 = Join-Path $RobotStudioRoot "Bin64\Addins"
$commonAddinsRoot = "C:\Program Files (x86)\Common Files\ABB\RobotStudio\Addins"
$commonManifestDestination = Join-Path $commonAddinsRoot "ABB6700.RobotStudioExport.rsaddin"
$msbuild = "C:\Windows\Microsoft.NET\Framework64\v4.0.30319\MSBuild.exe"

if (-not (Test-Path -LiteralPath $msbuild)) {
    throw "MSBuild not found: $msbuild"
}
if (-not (Test-Path -LiteralPath (Join-Path $RobotStudioRoot "Bin64\RobotStudio.exe"))) {
    throw "RobotStudio 6.08 root is invalid: $RobotStudioRoot"
}

& $msbuild $project /t:Rebuild /p:Configuration=Release /p:Platform=x64 /v:minimal
if ($LASTEXITCODE -ne 0) {
    throw "RobotStudio add-in build failed with exit code $LASTEXITCODE"
}

New-Item -ItemType Directory -Path $installDirectory -Force | Out-Null
Copy-Item -LiteralPath (Join-Path $buildDirectory "ABB6700.RobotStudioExport.dll") -Destination $installDirectory -Force
Copy-Item -LiteralPath (Join-Path $buildDirectory "ABB6700.RobotStudioExport.pdb") -Destination $installDirectory -Force
Copy-Item -LiteralPath $addinSource -Destination $addinManifestDestination -Force

# Earlier development builds used Bin64 as a second discovery root. RobotStudio
# scans both roots and reports a duplicate ApplicationId, so remove only our
# exact stale files there.
foreach ($staleName in @(
    "ABB6700.RobotStudioExport.dll",
    "ABB6700.RobotStudioExport.pdb",
    "ABB6700.RobotStudioExport.rsaddin"
)) {
    Remove-Item -LiteralPath (Join-Path $staleInstallDirectory64 $staleName) -Force -ErrorAction SilentlyContinue
}

$commonInstalled = $false
try {
    New-Item -ItemType Directory -Path $commonAddinsRoot -Force -ErrorAction Stop | Out-Null
    Copy-Item -LiteralPath (Join-Path $buildDirectory "ABB6700.RobotStudioExport.dll") -Destination $commonAddinsRoot -Force -ErrorAction Stop
    Copy-Item -LiteralPath (Join-Path $buildDirectory "ABB6700.RobotStudioExport.pdb") -Destination $commonAddinsRoot -Force -ErrorAction Stop
    Copy-Item -LiteralPath $addinSource -Destination $commonManifestDestination -Force -ErrorAction Stop
    $commonInstalled = $true
}
catch {
    Write-Warning "Common RobotStudio Addins directory was not writable; custom 6.08 directories were installed successfully."
}

Write-Output "ROBOTSTUDIO_ADDIN_DIR=$installDirectory"
Write-Output "ROBOTSTUDIO_ADDIN_COMMON_INSTALLED=$commonInstalled"
if (Get-Process RobotStudio -ErrorAction SilentlyContinue) {
    Write-Output "RESTART_REQUIRED=RobotStudio is running; save work and restart it before processing jobs."
}
