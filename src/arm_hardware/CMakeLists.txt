cmake_minimum_required(VERSION 3.8)
project(arm_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)

add_subdirectory(tendon_core)

add_library(${PROJECT_NAME} SHARED
  src/arm_hardware_interface.cpp
  src/socket_can.cpp
  src/motor.cpp
)

add_executable(motor_position src/motor_position_node.cpp)
target_compile_features(motor_position PUBLIC cxx_std_17)
target_link_libraries(motor_position ${PROJECT_NAME})
ament_target_dependencies(motor_position rclcpp)

add_executable(motor_position_slider src/motor_position_slider_node.cpp)
target_compile_features(motor_position_slider PUBLIC cxx_std_17)
target_link_libraries(motor_position_slider ${PROJECT_NAME})
ament_target_dependencies(motor_position_slider rclcpp sensor_msgs)

add_executable(motor_slider_command_bridge src/motor_slider_command_bridge.cpp)
target_compile_features(motor_slider_command_bridge PUBLIC cxx_std_17)
ament_target_dependencies(motor_slider_command_bridge rclcpp sensor_msgs std_msgs)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME} arm_tendon_compensation)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  urdf
)

pluginlib_export_plugin_description_file(hardware_interface arm_hardware.xml)

install(TARGETS ${PROJECT_NAME} arm_tendon_compensation
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS motor_position
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS motor_position_slider
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS motor_slider_command_bridge
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(FILES motor_slider.urdf.xacro motor_slider.ros2_control.xacro DESTINATION share/${PROJECT_NAME})
install(DIRECTORY SDK_UI/ DESTINATION share/${PROJECT_NAME}/SDK_UI)
install(FILES
  README.md
  HARDWARE_FEEDBACK_CHANGE_STEPS.md
  COUPLED_MOVEIT_IMPLEMENTATION_STEPS.md
  SYNCHRONIZED_SPEED_CHANGE_STEPS.md
  MOVEIT_TRAJECTORY_SPEED_CHANGE_STEPS.md
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  urdf
)

if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME static_tendon_compensation COMMAND
    ${CMAKE_COMMAND} -E env
    "ARM_TENDON_LIBRARY=$<TARGET_FILE:arm_tendon_compensation>"
    "ARM_TENDON_PROFILE="
    "ARM_TENDON_URDF="
    "PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/SDK_UI"
    "PYTHONDONTWRITEBYTECODE=1"
    ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/SDK_UI/tests/test_static_tendon_compensation.py
  )
  add_executable(test_synchronized_speed test/test_synchronized_speed.cpp)
  target_compile_features(test_synchronized_speed PUBLIC cxx_std_17)
  target_include_directories(test_synchronized_speed PRIVATE include)
  target_compile_options(test_synchronized_speed PRIVATE -UNDEBUG)
  add_test(NAME synchronized_speed COMMAND test_synchronized_speed)
endif()

ament_package()
