cmake_minimum_required(VERSION 3.8)
project(aidin_hand2_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(aidin_hand2_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)

# ── basic controller 4종에 대응하는 chainable 상위 controller skeleton ──
# 성격이 같은 템플릿이라 src/upper_controllers/ 한 곳에 두고 plugin library 하나로 묶는다
# (plugin/aidin_hand2_examples_plugins.xml 의 library path). 파라미터는
# config/upper_controllers/ 에 있다.
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
ament_target_dependencies(aidin_hand2_chainable_templates
  controller_interface
  pluginlib
  rclcpp
  realtime_tools
  aidin_hand2_msgs
)
install(TARGETS aidin_hand2_chainable_templates
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# ── 텔레오퍼 예제 controller (MANUS 글러브) ──
# manus_ros2_msgs 가 워크스페이스에 있을 때만 빌드한다(MANUS SDK 미포함 환경 대비).
find_package(manus_ros2_msgs QUIET)

if(manus_ros2_msgs_FOUND)
  add_library(aidin_hand2_examples_controllers SHARED
    src/glove_teleop/glove_teleop_controller.cpp
  )
  target_compile_features(aidin_hand2_examples_controllers PUBLIC cxx_std_17)
  target_include_directories(aidin_hand2_examples_controllers PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(aidin_hand2_examples_controllers
    controller_interface
    pluginlib
    realtime_tools
    rclcpp
    manus_ros2_msgs
  )
  install(TARGETS aidin_hand2_examples_controllers
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
else()
  message(STATUS
    "aidin_hand2_examples: glove teleop controller 건너뜀 "
    "(manus_ros2_msgs 없음)")
endif()

pluginlib_export_plugin_description_file(
  controller_interface plugin/aidin_hand2_examples_plugins.xml)

# config/<성격>/ 구조를 그대로 share 에 설치한다.
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES README.md EXAMPLE.md DESTINATION share/${PROJECT_NAME})

# 글러브 캘리브 스크립트 — ros2 run aidin_hand2_examples glove_calibrate
install(PROGRAMS
  scripts/glove_teleop/glove_calibrate.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME glove_calibrate
)

ament_package()
