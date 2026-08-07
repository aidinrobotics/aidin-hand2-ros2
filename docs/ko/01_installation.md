# 설치

이 문서는 Ubuntu 22.04·ROS 2 Humble 환경에서 SDK와 wrapper를 build하고 mock을 먼저 통과한 뒤 실제 hand를 의도적으로 homing하는 절차입니다.

## 1. 사전 조건

- Ubuntu 22.04
- ROS 2 Humble desktop 또는 필요한 base package
- `ros-dev-tools`, `rosdep`, `colcon`
- AIDIN Hand Gen2 SDK source
- AIDIN Hand Gen2 ROS 2 wrapper source
- 실물 사용 시 CAN-FD adapter와 안전한 작업 공간

ROS 2 Humble의 Ubuntu binary package는 Ubuntu 22.04 Jammy를 대상으로 합니다. ROS 2가 아직 없다면 [공식 Humble Ubuntu 설치 절차](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)로 설치하십시오.

```bash
source /opt/ros/humble/setup.bash
printenv ROS_DISTRO
```

결과가 `humble`인지 확인합니다.

## 2. Workspace layout

권장 layout:

```text
~/your_ws/
├── src/
│   └── aidin-hand2-ros2/
├── build/
├── install/
└── log/
```

SDK는 이 workspace에 포함되지 않습니다. Plain CMake package라 colcon workspace 밖에서 따로 build·install하고, wrapper는 설치된 결과를 `find_package(aidin_hand2)`로 찾습니다. Build tree는 SDK 문서의 표준 위치인 `cpp/build`, install prefix는 system prefix `/usr/local`입니다. SDK repository를 편의상 `src/` 아래에 두더라도 root의 `COLCON_IGNORE` 때문에 colcon은 무시합니다.

## 3. Dependency 설치

SDK dependency:

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  libeigen3-dev \
  libspdlog-dev \
  can-utils \
  ros-dev-tools \
  ros-humble-ros2controlcli
```

ROS package dependency를 설치합니다.

```bash
cd ~/your_ws
source /opt/ros/humble/setup.bash

rosdep update
rosdep install \
  --from-paths src/aidin-hand2-ros2 \
  --ignore-src \
  --recursive \
  --rosdistro humble \
  --skip-keys "aidin_hand2 manus_ros2_msgs" \
  -y
```

`aidin_hand2`는 rosdep key가 아니라 아래에서 설치하는 CMake package입니다. `manus_ros2_msgs`는 optional glove integration이지만 `package.xml`에는 mandatory dependency로 선언돼 있어 일반 환경에서는 skip합니다.

`ros-humble-ros2controlcli`는 이 문서의 `ros2 control ...` 진단·전환 명령에 필요합니다. 현재 wrapper package metadata가 CLI 자체를 runtime dependency로 선언하지 않으므로 명시적으로 설치합니다.

## 4. SDK build와 설치

SDK repository root에서 build하고 install합니다. Wrapper는 web bridge를 쓰지 않으므로 꺼서 build 시간을 줄입니다.

```bash
cd <aidin-hand2-sdk clone 경로>

cmake -S cpp -B cpp/build \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo \
  -DAIDIN_HAND2_BUILD_WEB_BRIDGE=OFF

cmake --build cpp/build -j"$(nproc)"
ctest --test-dir cpp/build --output-on-failure
sudo cmake --install cpp/build
```

Package config를 확인합니다.

```bash
test -f /usr/local/lib/cmake/aidin_hand2/aidin_hand2Config.cmake
```

`/usr/local`은 CMake 기본 탐색 경로이므로 `CMAKE_PREFIX_PATH` 설정이 필요 없습니다.

Sudo를 쓰지 않으려면 사용자 prefix에 install하고 그 경로를 새 terminal마다 `CMAKE_PREFIX_PATH`에 넣습니다. 자동화할 때는 workspace-specific setup script에 넣고 global shell profile에 hard-code하지 않는 편이 version 관리에 안전합니다.

```bash
cmake --install cpp/build --prefix "$HOME/.local"
export CMAKE_PREFIX_PATH="$HOME/.local${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
```

## 5. Wrapper build

```bash
cd ~/your_ws
source /opt/ros/humble/setup.bash

colcon build \
  --symlink-install \
  --event-handlers console_cohesion+ \
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
> 이 repository에는 현재 automated ROS 2 test가 없습니다. `colcon test`가 통과하더라도 wrapper behavior를 검증하는 test case가 있다는 뜻은 아닙니다. 아래 mock smoke test를 별도로 수행하십시오.

다음 단계는 [첫 bringup](02_first_bringup.md)입니다.
