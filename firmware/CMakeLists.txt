# Compile the board drivers before linking the firmware application.
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

add_library(board_drivers STATIC src/board.cpp src/uart.cpp)
target_include_directories(board_drivers PRIVATE src include ${DEVICE_INCLUDES}
  ${freertos_kernel_SOURCE_DIR}/include ${freertos_kernel_SOURCE_DIR}/portable/GCC/ARM_CM4F)
target_compile_definitions(board_drivers PRIVATE ${DEVICE_DEFS})
target_compile_options(board_drivers PRIVATE -Wall -Wextra -Wdouble-promotion)
