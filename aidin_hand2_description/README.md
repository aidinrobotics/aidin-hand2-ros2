# aidin_hand2_description

Robot description of the AIDIN Hand Gen2, for ROS 2 and simulation.

- xacro: `xacro/aidin_hand2.urdf.xacro` (top level), `xacro/aidin_hand2_{left,right}.urdf.xacro`
  (geometry macro per side), `ros2_control/aidin_hand2.ros2_control.xacro` (hardware component).
  ROS 2 builds the model from these.
- urdf: `urdf/aidin_hand2_{left,right}.urdf`, one hand each, rooted at its `*_hand_base_link` and
  naming its meshes relative to itself. Use these in tools that do not resolve `package://`.
- mesh: `meshes/{left,right}_aidin_hand2/{visual,collision}/*.STL`, referenced as
  `package://aidin_hand2_description/...`. A mesh carries the name of the link it draws.
- 21 joints per hand (thumb 5 + long finger 4x4) and 16 actuators (thumb 4 + long finger 3x4),
  prefixed `left_` or `right_`. Each `joint4` is passive, driven by the four-bar from its `joint3`.
- 17 tactile links per hand on fixed joints, three per finger and two on the palm. They carry no
  actuator and are there to give each pad a frame in TF.
- `launch/description.launch.py` shows the URDF in RViz without hardware.

The [package guide (Korean)](README.ko.md) covers file locations,
macro calls and parameters, the RViz launch arguments, and the plain URDF.
