# aidin_hand2_examples

예제를 controller 성격별로 나눠 제공합니다.

| 성격 | source | config |
|---|---|---|
| 상위 controller skeleton (basic controller 4종 대응) | `src/upper_controllers/` | `config/upper_controllers/` |
| 글러브 텔레오퍼 controller (옵션) | `src/glove_teleop/` | `config/glove_teleop/` |

- `src/upper_controllers/` 는 basic controller 4종(JointPosition · JointImpedance ·
  ActuatorPosition · ActuatorEffort) 위에 얹는 chainable 상위 controller skeleton 입니다. 성격이
  같은 템플릿이라 한 디렉터리·한 plugin library 로 두고, 파라미터만
  `config/upper_controllers/<mode>_upper.yaml` 네 개로 나눠 두었습니다. HandState 전체를 저장하고
  값은 만들지 않습니다.
- `src/glove_teleop/` 는 MANUS 입력을 하위 자세 controller reference 로 연결하는 동작 예제이며
  `manus_ros2_msgs` 가 워크스페이스에 있을 때만 빌드됩니다. 캘리브 스크립트는
  `scripts/glove_teleop/glove_calibrate.py` 입니다.
reference 이름, 네 skeleton 의 사용법과 수정 지점은 [EXAMPLE.md](EXAMPLE.md)를 기준으로 합니다.
