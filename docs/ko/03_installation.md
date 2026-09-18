# Installation

Ubuntu 22.04와 ROS 2 Humble에서 SDK와 wrapper를 설치합니다. 설치를 마치면 로봇 핸드 없이 mock을
실행할 수 있습니다. RT kernel과 CAN-FD 설정은 로봇 핸드를 실행하기 전에 준비합니다.

아래 예제의 workspace는 `~/your_ws`입니다.

## Contents

&nbsp;&nbsp;[**1. Prepare the workspace**](#1-prepare-the-workspace)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.1 Check ROS 2 Humble](#11-check-ros-2-humble)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.2 Get the source](#12-get-the-source)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.3 Install ROS dependencies](#13-install-ros-dependencies)<br>
&nbsp;&nbsp;[**2. Install the SDK**](#2-install-the-sdk)<br>
&nbsp;&nbsp;[**3. Build the wrapper**](#3-build-the-wrapper)<br>
&nbsp;&nbsp;[**4. Verify**](#4-verify)

## 1. Prepare the workspace

### 1.1 Check ROS 2 Humble

ROS 2가 없다면 [공식 Humble 설치 절차](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)로
먼저 설치하십시오. 설치된 환경을 현재 터미널에 적용합니다.

```bash
source /opt/ros/humble/setup.bash
printenv ROS_DISTRO
```

출력이 `humble`이어야 합니다. 첫 명령에서 파일을 찾지 못하면 ROS 2 설치를 확인하십시오.

### 1.2 Get the source

저장소를 받고 빌드하는 데 필요한 도구를 설치합니다.

```bash
sudo apt update
sudo apt install -y git python3-vcstool ros-dev-tools ros-humble-ros2controlcli
```

새 workspace에 wrapper를 받습니다. 이미 clone했다면 기존 저장소를 사용하고 이 명령은 건너뜁니다.

```bash
mkdir -p ~/your_ws/src
cd ~/your_ws/src
git clone https://github.com/aidinrobotics/aidin-hand2-ros2.git
```

검증된 SDK revision은 [`aidin_hand2.repos`](../../aidin_hand2.repos)에 고정되어 있습니다.
이 파일의 SDK 주소는 SSH를 사용하므로 GitHub SSH 인증과 저장소 접근 권한이 필요합니다.
다음 명령은 SDK를 `~/your_ws/src/aidin-hand2-sdk`에 받습니다.

```bash
cd ~/your_ws/src/aidin-hand2-ros2
vcs import .. < aidin_hand2.repos
```

### 1.3 Install ROS dependencies

rosdep을 처음 쓰는 컴퓨터라면 초기화합니다. 이미 초기화했다면 이 명령은 건너뜁니다.

```bash
sudo rosdep init
```

wrapper의 `package.xml`이 선언한 ROS package를 설치합니다.

```bash
cd ~/your_ws
rosdep update
rosdep install --from-paths src/aidin-hand2-ros2 --ignore-src --rosdistro humble -y
```

`#All required rosdeps installed successfully`가 출력되면 다음 단계로 진행합니다.

## 2. Install the SDK

mock도 SDK의 kinematics를 사용하므로 SDK 설치가 필요합니다. SDK 저장소로 이동합니다.

```bash
cd ~/your_ws/src/aidin-hand2-sdk
```

SDK의 [SDK build & install](https://github.com/aidinrobotics/aidin-hand2-sdk/blob/main/docs/ko/06_sdk_build_and_install.md)
1~3장에 따라 의존성 설치, hand type 선택, 빌드와 설치를 수행하십시오. SDK 문서가 저장소 root를
요구하는 명령은 위 경로에서 실행합니다. mock만 사용할 때는 SDK 문서 머리의 RT·CAN 준비와
4장의 로봇 핸드 실행을 수행할 필요가 없습니다.

설치 위치는 기본값 `/usr/local`을 사용합니다. 사용자 경로가 필요하다면 SDK 문서의 `~/.local`
설치 방법을 따르고, 아래 확인 명령의 경로도 바꾸십시오.

기본 위치에 설치한 경우 CMake 설정 파일과 라이브러리 등록을 확인합니다.

```bash
ls /usr/local/lib/cmake/aidin_hand2/aidin_hand2Config.cmake
ldconfig -p | grep aidin_hand2
```

첫 명령은 파일 경로를, 둘째 명령은 `libaidin_hand2.so`로 시작하는 줄을 출력해야 합니다.
파일이 없으면 SDK 설치 결과를 확인하고, 라이브러리 등록이 빠졌다면 `sudo ldconfig`를 실행합니다.
`~/.local` 설치에서는 설정 파일을 확인하고 다음 절의 경로 설정을 적용합니다.

## 3. Build the wrapper

SDK를 `~/.local`에 설치한 경우에만 다음 경로를 설정합니다. 다른 사용자 경로를 선택했다면
`$HOME/.local`을 해당 경로로 바꾸십시오. `/usr/local`에 설치했다면 건너뜁니다.

```bash
export CMAKE_PREFIX_PATH="$HOME/.local${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export LD_LIBRARY_PATH="$HOME/.local/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

`CMAKE_PREFIX_PATH`는 빌드할 때, `LD_LIBRARY_PATH`는 실행할 때 SDK를 찾는 경로입니다.
사용자 경로에 설치한 경우 새 실행 터미널에서도 `LD_LIBRARY_PATH` 설정이 필요합니다.

workspace root에서 wrapper package를 빌드합니다.

```bash
cd ~/your_ws
source /opt/ros/humble/setup.bash
colcon build --base-paths src/aidin-hand2-ros2 --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```

`Summary: 6 packages finished`로 끝나면 됩니다. `aidin_hand2`를 찾지 못하면
[2. Install the SDK](#2-install-the-sdk)의 설치 위치와 `CMAKE_PREFIX_PATH`를 확인하십시오.

## 4. Verify

workspace 환경을 적용하고 package 목록을 확인합니다.

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

이어서 [1. Mock](04_bringup.md#1-mock)에서 목표값을 보내 동작을 확인하십시오.
로봇 핸드를 실행할 때는 [Real-time kernel setup](01_real_time_kernel_setup.md)과
[CAN-FD setup](02_can_fd_setup.md)을 마친 뒤 [2. Robot hand](04_bringup.md#2-robot-hand)로 진행합니다.
