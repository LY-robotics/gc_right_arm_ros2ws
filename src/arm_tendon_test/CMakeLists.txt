cmake_minimum_required(VERSION 3.8)
project(arm_tendon_test)

find_package(ament_cmake REQUIRED)

install(PROGRAMS scripts/static_tendon_test.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME static_tendon_test
)

ament_package()
