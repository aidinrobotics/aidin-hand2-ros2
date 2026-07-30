# ROS2 Wrapper Boundary

- SDK owns CAN-FD transport, frame codec, command sequencing, diagnostics, logging, and kinematics.
- `aidin_hand2_hardware` adapts SDK command/state to ros2_control hardware interfaces.
- `aidin_hand2_controllers` should remain thin and avoid protocol ownership.
- `aidin_hand2_bringup` owns launch/config workflows.
