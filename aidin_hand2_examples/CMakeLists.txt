cmake_minimum_required(VERSION 3.8)
project(aidin_hand2_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(aidin_hand2 0.5 REQUIRED)  # 0.x treats a minor bump as breaking

# One chainable upper controller skeleton per command controller, in a single plugin library
# named by plugin/aidin_hand2_examples_plugins.xml
add_library(aidin_hand2_chainable_templates SHARED
  src/upper_controllers/joint_position_upper_controller.cpp
  src/upper_controllers/joint_impedance_upper_controller.cpp
  src/upper_controllers/actuator_position_upper_controller.cpp
  src/upper_controllers/actuator_effort_upper_controller.cpp
)
target_compile_features(aidin_hand2_chainable_templates PUBLIC cxx_std_17)
target_include_directories(aidin_hand2_chainable_templates PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(aidin_hand2_chainable_templates aidin_hand2::aidin_hand2)
ament_target_dependencies(aidin_hand2_chainable_templates
  controller_interface
  pluginlib
  rclcpp
  realtime_tools
  sensor_msgs
)
install(TARGETS aidin_hand2_chainable_templates
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(
  controller_interface plugin/aidin_hand2_examples_plugins.xml)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md EXAMPLE.md DESTINATION share/${PROJECT_NAME})

ament_package()
