# Portable components, compiled on the host and for the MCU.
add_library(autopilot_core STATIC
  src/depth_controller.cpp)
target_include_directories(autopilot_core PUBLIC include)
