cmake_minimum_required(VERSION 3.8)
project(arm_tendon_core LANGUAGES CXX)

add_library(arm_tendon_compensation SHARED ../src/static_tendon_compensation.cpp)
target_compile_features(arm_tendon_compensation PUBLIC cxx_std_17)
target_compile_definitions(arm_tendon_compensation PRIVATE ARM_TENDON_BUILD_DLL)
target_include_directories(arm_tendon_compensation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
  $<INSTALL_INTERFACE:include>
)
