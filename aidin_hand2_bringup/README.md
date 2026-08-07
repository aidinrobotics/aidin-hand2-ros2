# aidin_hand2_bringup

손을 단독 실행하는 launch와 controller config — 하드웨어 확인용 예제다. 자기 로봇에 붙일 때는
이 launch 대신 [ros2_control 설정](../docs/ko/03_setup.md)의 xacro 매크로를 쓴다.

- launch: `aidin_hand2.launch.py` (실물), `aidin_hand2_mock.launch.py` (mock),
  `aidin_hand2_controllers.launch.py` (controller만), `gui_bridge.launch.py` (rosbridge).
- config: `hand_bringup.yaml` (launch 인자), `controllers.yaml` / `controllers_mock.yaml`
  (controller_manager 파라미터).
- 인자와 config 우선순위는 [Bringup 예제](../docs/ko/05_bringup_example.md) 참조.