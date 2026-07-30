cmake_minimum_required(VERSION 3.8)
project(aidin_hand2_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HandState.msg"
  "msg/HandDiagnostics.msg"
  "msg/JointPositionCommand.msg"
  "msg/JointImpedanceCommand.msg"
  "msg/ActuatorPositionCommand.msg"
  "msg/ActuatorEffortCommand.msg"
  "msg/CommandState.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
