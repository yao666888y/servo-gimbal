cmake_minimum_required(VERSION 3.20)

project(gimbal C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(MCU_ARCH -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard)

add_compile_options(${MCU_ARCH} -Wall -Wextra -Wno-unused-parameter
                    -ffunction-sections -fdata-sections -Og -g3)
add_compile_definitions(USE_HAL_DRIVER STM32H743xx)

set(LD_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/STM32H743ZITx_FLASH_DTCMRAM.ld)
add_link_options(${MCU_ARCH} -T${LD_SCRIPT} -Wl,--gc-sections
                 -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/gimbal.map
                 --specs=nano.specs -Wl,--print-memory-usage)

include_directories(
  Core/Inc
  Drivers/STM32H7xx_HAL_Driver/Inc
  Drivers/CMSIS/Device/ST/STM32H7xx/Include
  Drivers/CMSIS/Include
)

file(GLOB APP_SOURCES CONFIGURE_DEPENDS Core/Src/*.c)
file(GLOB HAL_SOURCES CONFIGURE_DEPENDS Drivers/STM32H7xx_HAL_Driver/Src/*.c)

add_executable(gimbal.elf
  ${APP_SOURCES}
  ${HAL_SOURCES}
  Core/Startup/startup_stm32h743zitx.s
)

add_custom_command(TARGET gimbal.elf POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:gimbal.elf> ${CMAKE_CURRENT_BINARY_DIR}/gimbal.bin
  COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:gimbal.elf> ${CMAKE_CURRENT_BINARY_DIR}/gimbal.hex
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:gimbal.elf>
  COMMENT "Generate gimbal.bin / gimbal.hex"
)
