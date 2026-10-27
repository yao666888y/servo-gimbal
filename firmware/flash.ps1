<#
  用 ST-Link 把 build/gimbal.hex 烧进芯片（走 STM32CubeProgrammer CLI）

  用法:
    .\flash.ps1            # 烧录 + 校验 + 复位运行
    .\flash.ps1 -Erase     # 先全片擦除再烧
    .\flash.ps1 -List      # 只列出探针，确认 ST-Link 是否被识别
#>
param(
    [switch]$Erase,
    [switch]$List
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $root

# --------------------------------------------------------------------------
# 1) 定位 STM32_Programmer_CLI
# --------------------------------------------------------------------------
$cli = $null
$candidates = @(
    'D:\CLT\STM32CubeCLT_1.18.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe',
    'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe',
    'C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
)
foreach ($c in $candidates) {
    if (Test-Path $c) { $cli = $c; break }
}
if (-not $cli) {
    $cmd = Get-Command 'STM32_Programmer_CLI.exe' -ErrorAction SilentlyContinue
    if ($cmd) { $cli = $cmd.Source }
}
if (-not $cli) {
    throw '找不到 STM32_Programmer_CLI.exe，请先安装 STM32CubeProgrammer / STM32CubeCLT'
}

if ($List) {
    & $cli -l
    exit $LASTEXITCODE
}

# --------------------------------------------------------------------------
# 2) 检查产物
# --------------------------------------------------------------------------
$hex = Join-Path $root 'build\gimbal.hex'
if (-not (Test-Path $hex)) {
    throw "找不到 build\gimbal.hex，请先运行 .\build.ps1"
}

# --------------------------------------------------------------------------
# 3) 烧录
# --------------------------------------------------------------------------
if ($Erase) {
    Write-Host '[1/2] 全片擦除...' -ForegroundColor Cyan
    & $cli -c port=SWD -e all
    if ($LASTEXITCODE -ne 0) { throw "擦除失败 (exit $LASTEXITCODE)" }
}

Write-Host '[2/2] 烧录 + 校验 + 复位...' -ForegroundColor Cyan
& $cli -c port=SWD -w $hex -v -rst
if ($LASTEXITCODE -ne 0) {
    throw "烧录失败 (exit $LASTEXITCODE)。检查 ST-Link 接线：SWDIO/SWCLK/GND/3V3，必要时接 NRST。"
}

Write-Host ''
Write-Host '烧录完成，芯片已复位运行。' -ForegroundColor Green
Write-Host '  期望：PB0 心跳灯 1Hz 闪；串口每 100ms 收到 AA 55 05 84 ...'
