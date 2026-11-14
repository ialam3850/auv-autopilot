# Cross-compiling for the STM32F446RE (Cortex-M4F, single-precision FPU).
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Lets you point at a toolchain that isn't on PATH:
#   cmake -DARM_TOOLCHAIN_DIR=/opt/arm-gnu-toolchain/bin ...
if(ARM_TOOLCHAIN_DIR)
  set(TC "${ARM_TOOLCHAIN_DIR}/")
endif()
set(CMAKE_C_COMPILER ${TC}arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER ${TC}arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER ${TC}arm-none-eabi-gcc)
set(CMAKE_OBJCOPY ${TC}arm-none-eabi-objcopy CACHE FILEPATH "")
set(CMAKE_SIZE ${TC}arm-none-eabi-size CACHE FILEPATH "")

# Test programs can't link without our linker script.
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(MCU_FLAGS "-mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard")
set(CMAKE_C_FLAGS_INIT "${MCU_FLAGS} -ffunction-sections -fdata-sections")
set(CMAKE_CXX_FLAGS_INIT "${MCU_FLAGS} -ffunction-sections -fdata-sections -fno-exceptions -fno-rtti -fno-threadsafe-statics")
set(CMAKE_ASM_FLAGS_INIT "${MCU_FLAGS} -x assembler-with-cpp")
# -nostartfiles: ST's startup file provides Reset_Handler; newlib's crt0 would drag in
# exit() and with it all of stdio.
set(CMAKE_EXE_LINKER_FLAGS_INIT "${MCU_FLAGS} -nostartfiles -Wl,--gc-sections -Wl,--no-warn-rwx-segments --specs=nano.specs --specs=nosys.specs")
