cmake_minimum_required(VERSION 3.22)
option(RFID_HOST_TESTS "Build native regression tests instead of firmware" OFF)
if(RFID_HOST_TESTS)
  project(rfid_tests C)
  enable_testing()
  add_subdirectory(Tests)
  return()
endif()
project(RFID_CC1101_V38 C ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
option(RFID_DIAGNOSTICS "Enable bounded UART diagnostics" OFF)
option(RFID_DIAGNOSTIC_HOLD_AWAKE "Bench only: disable STOP to isolate sample timing" OFF)
if(RFID_DIAGNOSTIC_HOLD_AWAKE AND NOT RFID_DIAGNOSTICS)
  message(FATAL_ERROR "HOLD_AWAKE is allowed only in Diagnostics")
endif()
set(HAL Drivers/STM32L0xx_HAL_Driver)
set(HAL_MODULES hal hal_cortex hal_rcc hal_rcc_ex hal_flash hal_flash_ex
  hal_gpio hal_pwr hal_pwr_ex hal_rtc hal_rtc_ex hal_adc hal_adc_ex hal_dma hal_crc hal_crc_ex
  ll_gpio ll_rcc ll_spi ll_usart ll_utils ll_exti)
set(HAL_SOURCES)
foreach(module IN LISTS HAL_MODULES)
  list(APPEND HAL_SOURCES "${HAL}/Src/stm32l0xx_${module}.c")
endforeach()
set_source_files_properties(${HAL_SOURCES} PROPERTIES COMPILE_OPTIONS "-Wno-unused-parameter")
add_library(stm32_support OBJECT ${HAL_SOURCES}
  Core/Src/system_stm32l0xx.c Core/Src/stm32l0xx_hal_msp.c
  Core/Src/gpio.c Core/Src/spi.c Core/Src/usart.c Core/Src/adc.c Core/Src/rtc.c Core/Src/iwdg.c Core/Src/crc.c)
add_library(firmware_options INTERFACE)
target_compile_options(firmware_options INTERFACE -mcpu=cortex-m0plus -mthumb
  -ffunction-sections -fdata-sections -fstack-usage -Wall -Wextra
  $<$<CONFIG:Debug>:-Og;-g3> $<$<CONFIG:Release>:-Os;-g3>)
target_compile_definitions(firmware_options INTERFACE USE_HAL_DRIVER USE_FULL_LL_DRIVER STM32L051xx
  HSE_VALUE=12000000 RFID_DIAGNOSTICS=$<BOOL:${RFID_DIAGNOSTICS}>)
target_compile_definitions(firmware_options INTERFACE
  RFID_DIAGNOSTIC_HOLD_AWAKE=$<BOOL:${RFID_DIAGNOSTIC_HOLD_AWAKE}>)
target_include_directories(firmware_options INTERFACE Core/Inc App/Inc Platform/Inc
  ${HAL}/Inc Drivers/CMSIS/Include Drivers/CMSIS/Device/ST/STM32L0xx/Include
  Drivers/ADI Drivers/PCG)
target_link_libraries(stm32_support PUBLIC firmware_options)
add_executable(${PROJECT_NAME}
  Core/Startup/startup_stm32l051xx.s Platform/Src/entry.c Platform/Src/interrupts.c
  App/Src/behavior.c App/Src/fifo_parser.c App/Src/protocol.c App/Src/storage.c
  App/Src/history.c App/Src/app.c Platform/Src/board.c Platform/Src/sensor.c
  Platform/Src/no_os_port.c Platform/Src/radio.c Platform/Src/syscalls.c Platform/Src/clock.c
  Drivers/ADI/adxl362.c Drivers/PCG/pcg_basic.c)
set_target_properties(${PROJECT_NAME} PROPERTIES SUFFIX ".elf")
target_link_libraries(${PROJECT_NAME} PRIVATE stm32_support firmware_options m c)
target_link_options(${PROJECT_NAME} PRIVATE -mcpu=cortex-m0plus -mthumb --specs=nano.specs
  -T${CMAKE_SOURCE_DIR}/cmake/stm32l051.ld -Wl,--gc-sections
  -Wl,-Map=${PROJECT_NAME}.map -Wl,--print-memory-usage)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.hex
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}> ${PROJECT_NAME}.bin
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}>)
