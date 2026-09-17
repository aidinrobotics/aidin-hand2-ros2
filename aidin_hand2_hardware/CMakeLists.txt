cmake_minimum_required(VERSION 3.8)
project(aidin_hand2_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
# C++ SDK, not a ROS package. Point at it with CMAKE_PREFIX_PATH or aidin_hand2_DIR
find_package(aidin_hand2 0.5 REQUIRED)  # 0.x treats a minor bump as breaking

add_library(${PROJECT_NAME} SHARED
  src/aidin_hand2_system_interface.cpp
  src/aidin_hand2_mock_system_interface.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} aidin_hand2::aidin_hand2)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  rcl_interfaces
  std_srvs
)

pluginlib_export_plugin_description_file(hardware_interface plugin/aidin_hand2_hardware_plugin.xml)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  hardware_interface pluginlib rclcpp rclcpp_lifecycle rcl_interfaces
  std_srvs std_msgs sensor_msgs)
ament_package()
