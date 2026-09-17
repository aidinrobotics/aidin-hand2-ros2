# aidin_hand2_description

Robot description of the AIDIN Hand Gen2, for ROS 2 and simulation.

- xacro: `urdf/aidin_hand2.urdf.xacro` (top level), `urdf/aidin_hand2_{left,right}.urdf.xacro`
  (geometry macro per side), `ros2_control/aidin_hand2.ros2_control.xacro` (hardware component).
- mesh: `meshes/{visual,collision}/*.STL`, referenced as `package://aidin_hand2_description/...`.
- 21 joints per hand (thumb 5 + long finger 4x4) and 16 actuators (thumb 4 + long finger 3x4),
  prefixed `left_` or `right_`.
- `launch/description.launch.py` shows the URDF in RViz without hardware.

The macro parameters are in [Parameters](../docs/ko/09_parameters.md), and how to call the macros
from your own URDF is in [Integration](../docs/ko/05_integration.md).
