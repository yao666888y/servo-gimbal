<#
  Phase 1 固件构建脚本
  ---------------------------------------------------------------------------
  直接调用 arm-none-eabi-gcc 一次完成 编译 + 链接，不依赖 CMake / Ninja。

  用法:
    .\build.ps1            # 增量构建（全量重编，20 秒内）
    .\build.ps1 -Clean     # 先删 build 目录

  产物:
    build/gimbal.elf  build/gimbal.hex  build/gimbal.bin
#>
param(
    [switch]$Clean
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $root

# --------------------------------------------------------------------------
# 1) 定位 ARM 工具链
# --------------------------------------------------------------------------
if (-not (Get-Command arm-none-eabi-gcc -ErrorAction SilentlyContinue)) {
    $probe = @(
        'D:\CLT\STM32CubeCLT_1.18.0\GNU-tools-for-STM32\bin',
        'C:\ST\STM32CubeCLT_1.18.0\GNU-tools-for-STM32\bin',
        'C:\Program Files\STMicroelectronics\STM32CubeCLT\GNU-tools-for-STM32\bin'
    ) | Where-Object { Test-Path (Join-Path $_ 'arm-none-eabi-gcc.exe') }

    if (-not $probe) {
        throw "找不到 arm-none-eabi-gcc。请安装 STM32CubeCLT 或 ARM GNU Toolchain，并把它加入 PATH。"
    }
    $env:PATH = "$($probe[0]);$env:PATH"
}

if ($Clean -and (Test-Path 'build')) {
    Remove-Item 'build' -Recurse -Force
}
New-Item -ItemType Directory -Force 'build' | Out-Null

# --------------------------------------------------------------------------
# 2) 源文件
# --------------------------------------------------------------------------
$sources = @(
    (Get-ChildItem 'Core\Src\*.c' | ForEach-Object { $_.FullName })
    (Get-ChildItem 'Drivers\STM32H7xx_HAL_Driver\Src\*.c' | ForEach-Object { $_.FullName })
    'Core\Startup\startup_stm32h743zitx.s'
)

$gccArgs = @(
    '-mcpu=cortex-m7', '-mthumb', '-mfpu=fpv5-d16', '-mfloat-abi=hard'
    '-Og', '-g3'
    '-Wall', '-Wextra', '-Wno-unused-parameter'
    '-ffunction-sections', '-fdata-sections'
    '-DUSE_HAL_DRIVER', '-DSTM32H743xx'
    '-ICore/Inc'
    '-IDrivers/STM32H7xx_HAL_Driver/Inc'
    '-IDrivers/CMSIS/Device/ST/STM32H7xx/Include'
    '-IDrivers/CMSIS/Include'
) + $sources + @(
    '-T', 'STM32H743ZITx_FLASH_DTCMRAM.ld'
    '-Wl,--gc-sections'
    '--specs=nano.specs'
    '-Wl,--print-memory-usage'
    '-o', 'build/gimbal.elf'
)

# --------------------------------------------------------------------------
# 3) 编译 + 链接
# --------------------------------------------------------------------------
& arm-none-eabi-gcc @gccArgs
if ($LASTEXITCODE -ne 0) {
    throw "编译失败 (exit $LASTEXITCODE)"
}

& arm-none-eabi-objcopy -O binary build/gimbal.elf build/gimbal.bin
& arm-none-eabi-objcopy -O ihex   build/gimbal.elf build/gimbal.hex
& arm-none-eabi-size build/gimbal.elf

Write-Host ""
Write-Host "构建完成:" -ForegroundColor Green
Write-Host "  build\gimbal.hex  (STM32CubeProgrammer / ST-Link Utility 烧这个)"
Write-Host "  build\gimbal.bin  build\gimbal.elf"
