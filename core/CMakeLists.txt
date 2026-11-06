# Portable autopilot core. Pure C++17, no heap allocation after construction,
# no exceptions or RTTI required, so the firmware links the same sources.
add_library(autopilot_core STATIC
  src/attitude_ekf.cpp
  src/complementary_filter.cpp
  src/depth_controller.cpp
  src/protocol.cpp
  src/autopilot.cpp)
target_include_directories(autopilot_core PUBLIC include)
