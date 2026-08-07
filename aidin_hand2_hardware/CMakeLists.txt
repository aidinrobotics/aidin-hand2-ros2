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
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
# C++ SDK — ROS 패키지가 아니다. CMAKE_PREFIX_PATH 또는 aidin_hand2_DIR 로 install prefix 지정.
# SDK 는 정적 라이브러리라 PRIVATE 의존(Eigen)도 소비자 링크 단계에 필요($<LINK_ONLY:> 전파)
# → aidin_hand2Config 의 find_dependency(Eigen3) 가 이 스코프에서 타겟을 찾도록 직접 find.
find_package(Eigen3 REQUIRED)
find_package(aidin_hand2 0.2 REQUIRED)  # 0.x 는 minor 가 breaking 경계

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
  std_srvs
  std_msgs
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
  hardware_interface pluginlib rclcpp rclcpp_lifecycle std_srvs std_msgs)
ament_package()
