# STM32F446RE firmware. Configure with the cross toolchain:
#
#   cmake -S firmware -B build-fw -G Ninja \
#         -DCMAKE_TOOLCHAIN_FILE=firmware/cmake/arm-none-eabi.cmake
#   cmake --build build-fw
#
# Produces autopilot_fw.elf / .bin / .hex. Flash the .bin by copying it to the
# NUCLEO's USB drive, or use STM32CubeProgrammer / openocd.
cmake_minimum_required(VERSION 3.20)
project(autopilot_fw LANGUAGES C CXX ASM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE MinSizeRel)
endif()

set(AP_FW_MODE "HIL" CACHE STRING "HIL (simulated sensors over serial) or BENCH (real IMU)")
set_property(CACHE AP_FW_MODE PROPERTY STRINGS HIL BENCH)

include(FetchContent)
# Pinned dependencies. Device headers + startup code from ST, core headers
# from Arm, and the FreeRTOS kernel.
FetchContent_Declare(cmsis_device_f4
  GIT_REPOSITORY https://github.com/STMicroelectronics/cmsis_device_f4.git
  GIT_TAG v2.6.10 GIT_SHALLOW TRUE)
FetchContent_Declare(cmsis_core
  GIT_REPOSITORY https://github.com/ARM-software/CMSIS_6.git
  GIT_TAG v6.1.0 GIT_SHALLOW TRUE)
FetchContent_Declare(freertos_kernel
  GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel.git
  GIT_TAG V11.1.0 GIT_SHALLOW TRUE)
foreach(dep cmsis_device_f4 cmsis_core freertos_kernel)
  FetchContent_GetProperties(${dep})
  if(NOT ${dep}_POPULATED)
    FetchContent_Populate(${dep})
  endif()
endforeach()

set(DEVICE_DEFS STM32F446xx)
set(DEVICE_INCLUDES
  ${cmsis_device_f4_SOURCE_DIR}/Include
  ${cmsis_core_SOURCE_DIR}/CMSIS/Core/Include)

# FreeRTOS kernel built directly from source with our config.
set(RTOS ${freertos_kernel_SOURCE_DIR})
add_library(freertos STATIC
  ${RTOS}/tasks.c ${RTOS}/queue.c ${RTOS}/list.c
  ${RTOS}/portable/GCC/ARM_CM4F/port.c
  ${RTOS}/portable/MemMang/heap_4.c)
target_include_directories(freertos PUBLIC
  include ${RTOS}/include ${RTOS}/portable/GCC/ARM_CM4F ${DEVICE_INCLUDES})
target_compile_definitions(freertos PUBLIC ${DEVICE_DEFS})

# The same portable core the host build tests, compiled for the MCU.
add_subdirectory(../core core)
target_compile_options(autopilot_core PRIVATE -Wall -Wextra -Wdouble-promotion)

add_executable(autopilot_fw
  src/main.cpp
  src/board.cpp
  src/uart.cpp
  src/icm42688.cpp
  src/runtime.cpp
  ${cmsis_device_f4_SOURCE_DIR}/Source/Templates/system_stm32f4xx.c
  ${cmsis_device_f4_SOURCE_DIR}/Source/Templates/gcc/startup_stm32f446xx.s)
target_include_directories(autopilot_fw PRIVATE src include ${DEVICE_INCLUDES})
target_compile_definitions(autopilot_fw PRIVATE ${DEVICE_DEFS})
if(AP_FW_MODE STREQUAL "BENCH")
  target_compile_definitions(autopilot_fw PRIVATE AP_FW_MODE_BENCH=1)
endif()
target_compile_options(autopilot_fw PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-Wall -Wextra -Wdouble-promotion>)
target_link_libraries(autopilot_fw PRIVATE autopilot_core freertos m)
target_link_options(autopilot_fw PRIVATE
  -T${CMAKE_CURRENT_SOURCE_DIR}/STM32F446RETx_FLASH.ld
  -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/autopilot_fw.map -Wl,--print-memory-usage)
set_target_properties(autopilot_fw PROPERTIES SUFFIX ".elf")

add_custom_command(TARGET autopilot_fw POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:autopilot_fw> autopilot_fw.bin
  COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:autopilot_fw> autopilot_fw.hex
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:autopilot_fw>
  COMMENT "objcopy -> autopilot_fw.bin / .hex")
