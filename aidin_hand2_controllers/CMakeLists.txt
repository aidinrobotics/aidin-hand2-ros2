cmake_minimum_required(VERSION 3.8)
project(aidin_hand2_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(aidin_hand2_msgs REQUIRED)
find_package(aidin_hand2 0.4 REQUIRED)  # 0.x 는 minor 가 breaking 경계

add_library(${PROJECT_NAME} SHARED
  src/joint_position_controller.cpp
  src/actuator_position_controller.cpp
  src/actuator_effort_controller.cpp
  src/joint_impedance_controller.cpp
  src/hand_state_broadcaster.cpp
  src/diagnostics_broadcaster.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} aidin_hand2::aidin_hand2)
ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  std_msgs
  diagnostic_msgs
  aidin_hand2_msgs
)

pluginlib_export_plugin_description_file(controller_interface plugin/aidin_hand2_controllers_plugins.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  sensor_msgs
  std_msgs
  diagnostic_msgs
  aidin_hand2_msgs
)
ament_package()
