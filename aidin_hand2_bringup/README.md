# aidin_hand2_bringup

Launch files and controller configuration that run the robot hand on its own, as a hardware check. To
put the robot hand on your own robot, call the xacro macros as [Integration](../docs/ko/05_integration.md)
describes instead of using these launch files.

- launch: `aidin_hand2.launch.py` (robot hand), `aidin_hand2_mock.launch.py` (mock),
  `aidin_hand2_isaac.launch.py` (Isaac Sim), `aidin_hand2_controllers.launch.py` (controllers
  only), `gui_bridge.launch.py` (rosbridge).
- config: `hand_bringup.yaml` (launch arguments), `controllers.yaml` / `controllers_mock.yaml` /
  `controllers_isaac.yaml` (controller_manager parameters).
- The arguments of every launch file and the precedence between the CLI and the config file are in
  [Launch files](../docs/ko/10_launch_files.md).
