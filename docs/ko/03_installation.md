# Installation

Ubuntu 22.04와 ROS 2 Humble에 SDK를 install하고 wrapper package 6개를 build합니다. 먼저
[Real-time kernel setup](01_real_time_kernel_setup.md)과 [CAN-FD setup](02_can_fd_setup.md)을
마치십시오. 이 문서는 colcon workspace가 `~/your_ws`이고 wrapper 저장소가
`~/your_ws/src/aidin-hand2-ros2`에 clone되어 있다고 가정합니다.

## Contents

&nbsp;&nbsp;[**1. Check ROS 2 Humble**](#1-check-ros-2-humble)<br>
&nbsp;&nbsp;[**2. Install dependencies**](#2-install-dependencies)<br>
&nbsp;&nbsp;[**3. Install the SDK**](#3-install-the-sdk)<br>
&nbsp;&nbsp;[**4. Build the wrapper**](#4-build-the-wrapper)<br>
&nbsp;&nbsp;[**5. Verify**](#5-verify)

## 1. Check ROS 2 Humble

ROS 2 Humble의 binary package는 Ubuntu 22.04를 대상으로 합니다. ROS 2가 아직 없다면
[공식 Humble 설치 절차](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)로
먼저 설치하십시오.

설치되어 있는지 확인합니다.

```bash
source /opt/ros/humble/setup.bash
printenv ROS_DISTRO
```

출력이 `humble`이면 됩니다. 비어 있으면 ROS 2가 없거나 첫 줄을 실행하지 않은 shell입니다.

## 2. Install dependencies

wrapper build에는 colcon·vcstool과 `ros2 control` 명령, 그리고 `package.xml`이 선언한 ROS package
dependency가 필요합니다. compiler와 CMake는 3장의 SDK 문서가 함께 설치합니다.

`ros-dev-tools` package는 colcon과 vcstool을, `ros-humble-ros2controlcli` package는 `ros2 control` 명령을
제공합니다.

```bash
sudo apt update
sudo apt install -y ros-dev-tools ros-humble-ros2controlcli
```

rosdep을 처음 쓰는 컴퓨터라면 먼저 초기화합니다. 이미 했다면 `already exists`로 끝납니다.

```bash
sudo rosdep init
```

wrapper의 `package.xml`이 선언한 ROS package dependency를 설치합니다.

```bash
cd ~/your_ws
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src/aidin-hand2-ros2 --ignore-src --rosdistro humble -y
```

`#All required rosdeps installed successfully`가 출력되면 됩니다.

## 3. Install the SDK

wrapper는 install된 SDK를 `find_package(aidin_hand2 0.5 REQUIRED)`로 찾으므로 SDK를 먼저
install합니다. SDK는 plain CMake package라 colcon workspace 밖에서 따로 build하고, 저장소 root의
`COLCON_IGNORE` 때문에 `src/` 아래에 두어도 colcon이 건너뜁니다.

검증된 SDK revision은 [`aidin_hand2.repos`](../../aidin_hand2.repos)에 고정되어 있습니다. wrapper
저장소 root에서 실행하면 `~/your_ws/src/aidin-hand2-sdk`에 clone됩니다.

```bash
cd ~/your_ws/src/aidin-hand2-ros2
vcs import .. < aidin_hand2.repos
```

> [!IMPORTANT]
> **SDK를 configure할 때 로봇 핸드의 kinematics에 맞는 값을 선택합니다.** 선택지와 방법은 아래 SDK
> 문서의 2장에 있고, 틀리게 골라도 build는 성공하므로 configure 출력으로 확인하십시오. 값은 로봇 핸드
> 전달과 함께 알려드립니다.

이어서 SDK 문서의
[SDK build & install](https://github.com/aidinrobotics/aidin-hand2-sdk/blob/main/docs/ko/06_sdk_build_and_install.md)
1장부터 3장까지 수행합니다. 3장의 install 위치는 `/usr/local`(a)을 권장합니다. `/usr/local`은 CMake
기본 탐색 경로라 wrapper build에서 경로를 지정할 일이 없습니다. `sudo`를 쓸 수 없어 사용자
prefix(b)에 install했다면 4장에서 install prefix를 지정합니다.

install 결과를 확인합니다. `<prefix>`는 install한 위치이고, 지정하지 않았다면 `/usr/local`입니다.

```bash
grep -m1 'set(PACKAGE_VERSION "' <prefix>/lib/cmake/aidin_hand2/aidin_hand2ConfigVersion.cmake
ldconfig -p | grep aidin_hand2
```

첫 명령이 출력한 version이 `aidin_hand2.repos`의 `version` 값과 minor까지 같으면 됩니다. wrapper가
`SameMinorVersion` 정책으로 찾으므로 patch는 달라도 됩니다. 둘째 명령은 `/usr/local`에 install한
경우에만 출력이 있고, 출력에 `libaidin_hand2.so.0.5`가 있어야 합니다. 출력이 없으면 `sudo ldconfig`를
실행하지 않은 경우입니다.

## 4. Build the wrapper

wrapper build는 workspace root에서 colcon으로 package 6개를 한 번에 build하는 단계입니다.

SDK를 사용자 prefix에 install한 경우에만 build할 shell에서 경로를 먼저 지정합니다. `/usr/local`에
install했다면 이 단계가 없습니다.

```bash
export CMAKE_PREFIX_PATH="$HOME/.local${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
```

workspace root에서 ROS 2 환경을 source하고 package 6개를 build합니다.

```bash
cd ~/your_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

`Summary: 6 packages finished`로 끝나면 됩니다. `Could not find a package configuration file
provided by "aidin_hand2"`로 실패하면 3장의 install 확인으로 돌아가십시오.

## 5. Verify

overlay를 적용하고 package 목록을 확인합니다.

```bash
source ~/your_ws/install/setup.bash
ros2 pkg list | grep '^aidin_hand2_'
```

다음 6개가 출력되어야 합니다.

```text
aidin_hand2_bringup
aidin_hand2_controllers
aidin_hand2_description
aidin_hand2_examples
aidin_hand2_hardware
aidin_hand2_msgs
```

이어서 [Bringup](04_bringup.md)에서 mock을 먼저 실행해 로봇 핸드 없이 동작을 확인하고, 그다음 로봇
핸드를 실행하십시오.
