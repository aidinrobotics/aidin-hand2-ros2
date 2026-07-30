# aidin_hand2_description

AIDIN Hand Gen2 의 robot description (URDF/xacro macro/mesh) — ROS2 + 시뮬레이션용.

- xacro: `urdf/aidin_hand2.urdf.xacro` (top), `urdf/aidin_hand2_{left,right}.urdf.xacro` (기하 macro),
  `ros2_control/aidin_hand2.ros2_control.xacro`.
- mesh: `meshes/{visual,collision}/*.STL`, `package://aidin_hand2_description/...` 로 참조.
- joint 21 (thumb 5 + long finger 4×4), prefix `left_`/`right_`.
