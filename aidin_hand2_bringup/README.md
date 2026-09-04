# aidin_hand2_bringup

Launch files and controller config that run a hand on its own, as a hardware check. To attach the
hand to your own robot, call the xacro macro from [ros2_control setup](../docs/ko/03_setup.md)
instead of these launch files.

- launch: `aidin_hand2.launch.py` (real), `aidin_hand2_mock.launch.py` (mock),
  `aidin_hand2_isaac.launch.py` (Isaac Sim), `aidin_hand2_controllers.launch.py` (controllers
  only), `gui_bridge.launch.py` (rosbridge).
- config: `hand_bringup.yaml` (launch arguments), `controllers.yaml` / `controllers_mock.yaml` /
  `controllers_isaac.yaml` (controller_manager parameters).
- For the arguments and how the config overrides them, see
  [the bringup example](../docs/ko/05_bringup_example.md).
