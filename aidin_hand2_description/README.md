# aidin_hand2_description

Robot description of the AIDIN Hand Gen2, for ROS 2 and simulation.

- xacro: `urdf/aidin_hand2.urdf.xacro` (top), `urdf/aidin_hand2_{left,right}.urdf.xacro`
  (geometry macro), `ros2_control/aidin_hand2.ros2_control.xacro`.
- mesh: `meshes/{visual,collision}/*.STL`, referenced as `package://aidin_hand2_description/...`.
- 21 joints (thumb 5 + long finger 4x4), prefixed `left_` or `right_`.
