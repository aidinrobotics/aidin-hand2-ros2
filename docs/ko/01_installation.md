# 설치

Ubuntu 22.04·ROS 2 Humble 환경에서 SDK를 install하고 wrapper package 6개를 build하는 절차입니다.
실물 로봇 핸드를 처음 움직이는 절차는 [첫 bringup](02_first_bringup.md)에 있습니다.

ROS 2 Humble의 Ubuntu binary package는 Ubuntu 22.04 Jammy를 대상으로 합니다. ROS 2가 아직 없다면
[공식 Humble Ubuntu 설치 절차](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)로
먼저 설치하십시오. 설치되어 있다면 아래 두 명령이 `humble`을 출력합니다.

```bash
source /opt/ros/humble/setup.bash
printenv ROS_DISTRO
```

## 1. ROS 2 dependency 설치

Compiler와 CAN 진단 도구를 포함한 SDK dependency는 2절에서 SDK 문서를 따라 설치하므로, 여기서는
wrapper에 필요한 것만 설치합니다.

```bash
sudo apt update
sudo apt install -y ros-dev-tools ros-humble-ros2controlcli
```

ROS package dependency를 설치합니다.

```bash
cd ~/your_ws
source /opt/ros/humble/setup.bash

rosdep update
rosdep install \
  --from-paths src/aidin-hand2-ros2 \
  --ignore-src \
  --rosdistro humble \
  -y
```

## 2. SDK build와 설치

Wrapper는 설치된 SDK를 `find_package`로 찾으므로 SDK를 먼저 install해야 합니다. build·install·제거
절차는 SDK 문서의
[SDK build & install](https://github.com/aidinrobotics/aidin-hand2-sdk/blob/main/docs/ko/06_sdk_build_and_install.md)에
있습니다. 그 문서의 1절부터 3절까지 완료한 뒤 돌아오십시오.

SDK는 plain CMake package이므로 colcon workspace 밖에서 따로 build합니다. 편의상 `src/` 아래에
clone해도 SDK repository root의 `COLCON_IGNORE` 때문에 colcon이 무시합니다.

> [!IMPORTANT]
> configure에서 thumb ball screw의 lead를 하드웨어에 맞게 선택해야 하고, 맞지 않는 쪽으로 빌드하면
> thumb의 actuator 두 개가 **두 배 또는 절반으로 움직입니다.** 선택 방법은 SDK 문서의
> [2. Build](https://github.com/aidinrobotics/aidin-hand2-sdk/blob/main/docs/ko/06_sdk_build_and_install.md#2-build)에
> 있습니다.

설치 결과를 확인합니다. `<prefix>`는 install한 prefix이고, 지정하지 않았다면 `/usr/local`입니다.

```bash
grep -m1 'set(PACKAGE_VERSION "' <prefix>/lib/cmake/aidin_hand2/aidin_hand2ConfigVersion.cmake
```

출력된 version이 [`aidin_hand2.repos`](../../aidin_hand2.repos)의 `version` 필드와 minor까지 같으면
됩니다. wrapper가 `SameMinorVersion` 정책으로 찾으므로 patch는 달라도 됩니다.

`/usr/local`에 install했다면 `ldconfig -p | grep aidin_hand2`에 `libaidin_hand2`와
`libaidin_hand2_kinematics` 두 항목도 나옵니다. `/usr/local`은 CMake 기본 탐색 경로이므로
`CMAKE_PREFIX_PATH` 설정이 필요 없습니다. 사용자 prefix에 install했다면 아래 wrapper build를
실행하는 shell에서 그 경로를 `CMAKE_PREFIX_PATH`에 넣습니다.

## 3. Wrapper build

Workspace root에서 ROS 2 환경을 source한 뒤 6개 package를 build합니다.

```bash
cd ~/your_ws
source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

Overlay를 적용합니다.

```bash
source install/setup.bash
ros2 pkg list | grep '^aidin_hand2_'
```

다음 6개 package가 보여야 합니다.

```text
aidin_hand2_bringup
aidin_hand2_controllers
aidin_hand2_description
aidin_hand2_examples
aidin_hand2_hardware
aidin_hand2_msgs
```

> [!NOTE]
> 이 repository에는 현재 automated ROS 2 test가 없습니다. `colcon test`가 통과하더라도 wrapper
> behavior를 검증하는 test case가 있다는 뜻은 아니므로, 실물 없이 동작을 확인하려면 다음 문서의
> mock smoke test를 수행하십시오.

다음 단계는 [첫 bringup](02_first_bringup.md)입니다.
