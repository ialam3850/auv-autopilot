# Host build: the portable autopilot core, its unit tests, and the SIL runner.
# The STM32 firmware has its own build in firmware/ (cross-compiled).
cmake_minimum_required(VERSION 3.20)
project(auv_autopilot LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

option(AP_BUILD_TESTS "Build unit tests" ON)
option(AP_SANITIZE "Build with AddressSanitizer + UBSan" OFF)

add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wdouble-promotion)
if(AP_SANITIZE)
  add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
  add_link_options(-fsanitize=address,undefined)
endif()

add_subdirectory(core)

add_executable(sil_runner host/sil_runner.cpp)
target_link_libraries(sil_runner PRIVATE autopilot_core)

if(AP_BUILD_TESTS)
  enable_testing()
  include(FetchContent)
  FetchContent_Declare(googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.15.2.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)
  add_executable(core_tests
    core/tests/test_ekf.cpp
    core/tests/test_depth_controller.cpp
    core/tests/test_protocol.cpp
    core/tests/test_autopilot.cpp)
  target_link_libraries(core_tests PRIVATE autopilot_core GTest::gtest_main)
  include(GoogleTest)
  gtest_discover_tests(core_tests)
endif()

# Replays recorded IMU data through the EKF and the complementary filter
# (used by sim/monte_carlo.py for the estimator comparison).
add_executable(estimator_replay host/estimator_replay.cpp)
target_link_libraries(estimator_replay PRIVATE autopilot_core)
