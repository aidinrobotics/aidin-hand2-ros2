# CAN-FD setup

SDK는 USB CAN-FD adapter를 통해 Linux SocketCAN으로 AIDIN Hand Gen2와 통신합니다(nominal
1 Mbit/s, data phase 5 Mbit/s). 이 문서는 **PEAK PCAN-USB FD**와 **CANable 2.0 Pro**(candleLight
firmware) 두 adapter를 다룹니다. 사용하는 adapter를 찾아 CAN-FD로 설정하고,
수신 frame으로 연결된 로봇 핸드의 좌우를 확인합니다. 설정 절차는 SDK 문서의
[CAN-FD setup](https://github.com/aidinrobotics/aidin-hand2-sdk/blob/main/docs/ko/05_can_fd_setup.md)과
같은 내용입니다.

## Contents

&nbsp;&nbsp;[**1. Bring up the interface**](#1-bring-up-the-interface)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.1 Check the driver](#11-check-the-driver)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.2 Find the interface](#12-find-the-interface)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.3 Bring up that interface](#13-bring-up-that-interface)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.4 Verify the link](#14-verify-the-link)<br>
&nbsp;&nbsp;[**2. Check the receive rate**](#2-check-the-receive-rate)

## 1. Bring up the interface

사용할 도구를 설치합니다. `can-utils`는 `candump`를, `ethtool`은 adapter 정보 조회 명령을 제공합니다.

```bash
sudo apt update
sudo apt install -y can-utils ethtool gawk
```

adapter의 kernel driver를 확인하고, AIDIN Hand Gen2가 연결된 interface를 찾아 CAN-FD로 올린 뒤,
들어오는 frame으로 각 interface가 어느 side인지 확인합니다.

### 1.1 Check the driver

driver module이 없으면 adapter를 연결해도 CAN network device가 등록되지 않습니다.

```bash
modinfo peak_usb | head -2       # PEAK PCAN-USB FD
modinfo gs_usb   | head -2       # CANable 2.0 Pro (candleLight)
```

사용하는 adapter의 driver에서 `filename`이 출력되면 됩니다. Ubuntu 22.04·24.04 kernel은 두
driver를 module로 포함하며 adapter 연결 시 자동으로 load하므로 별도 설치가 필요하지 않습니다.
`Module ... not found`는 해당 kernel에 driver가 없다는 뜻입니다. Ubuntu 기본 kernel이라면
`sudo apt install linux-modules-extra-$(uname -r)`로 module 패키지를 설치하고, 직접 빌드한
kernel이라면 [1.5 Configure the kernel](01_real_time_kernel_setup.md#15-configure-the-kernel)에서
`CONFIG_CAN_PEAK_USB`와 `CONFIG_CAN_GS_USB`를 활성화하십시오. 공급업체 kernel도 동일한 config가
필요하며, 재빌드 절차는 해당 공급업체 문서를 따르십시오.

### 1.2 Find the interface

CAN interface와 각 interface의 driver를 확인합니다.

```bash
ip -brief link show type can
for i in $(ip -o link show type can | awk -F': ' '{print $2}'); do
  echo "=== $i ==="; ethtool -i "$i" | grep -E '^(driver|bus-info):'
done
```

`driver`가 `peak_usb`면 PCAN-USB FD, `gs_usb`면 CANable 2.0 Pro입니다. AIDIN Hand Gen2가
왼손·오른손 둘이면 해당 항목도 둘입니다. 각 interface 이름을 확인합니다(예: `can0`·`can1`).

### 1.3 Bring up that interface

[1.2 Find the interface](#12-find-the-interface)에서 찾은 interface를 AIDIN Hand Gen2의 bitrate로, CAN-FD로
올립니다. 이 단계에서는 side를 알 필요가 없습니다. 찾은 interface를 모두 올린 뒤
[1.4 Verify the link](#14-verify-the-link)에서 판별합니다. 아래는 `can0`과 `can1` 두 개인 경우입니다.

`can0`을 CAN-FD로 설정합니다.

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can \
  bitrate 1000000 sample-point 0.875 sjw 10 \
  dbitrate 5000000 dsample-point 0.875 dsjw 2 \
  fd on restart-ms 100
sudo ip link set can0 up
sudo ip link set can0 txqueuelen 1000
```

두 번째 adapter도 사용한다면 `can1`을 설정합니다.

```bash
sudo ip link set can1 down 2>/dev/null || true
sudo ip link set can1 type can \
  bitrate 1000000 sample-point 0.875 sjw 10 \
  dbitrate 5000000 dsample-point 0.875 dsjw 2 \
  fd on restart-ms 100
sudo ip link set can1 up
sudo ip link set can1 txqueuelen 1000
```

> [!NOTE]
> `restart-ms`는 bus-off 발생 시 자동복구까지의 대기 시간(ms)입니다. 0이면 자동복구가 꺼져,
> 전원이 순간 끊길 때 컨트롤러가 bus-off로 정지한 뒤 SDK의 auto-reconnect도 회복하지 못합니다.

### 1.4 Verify the link

interface 하나씩 확인합니다. 확인할 interface를 지정합니다(`can0`부터, 이어서 `can1`).

```bash
CAN=can0                       # 확인할 interface
```

link 상태를 확인합니다.

```bash
ip -details -statistics link show "$CAN"
```

출력에서 아래 줄과 값을 확인합니다.


```text
# <FD> = CAN-FD, ERROR-ACTIVE = 정상 버스, berr-counter 0 = error 안 오름
can <FD> state ERROR-ACTIVE (berr-counter tx 0 rx 0) restart-ms 100
# nominal: bitrate 1 Mbit/s, sample-point 0.875
  bitrate 1000000 sample-point 0.875
# data phase: bitrate 5 Mbit/s, sample-point 0.875
  dbitrate 5000000 dsample-point 0.875
```

`state`가 `BUS-OFF`·`ERROR-PASSIVE`이거나 `berr-counter`가 계속 오르면 배선·termination·bitrate를
점검합니다.

AIDIN Hand Gen2가 연결·전원 On이면 state frame이 주기적으로 들어옵니다. `candump`로 frame을
확인하고 `Ctrl-C`로 종료합니다.

```bash
candump "$CAN"
```

들어오는 ID의 첫 자리로 어느 side인지 확인합니다 — **왼손은 `0x2xx`**(`221` 등), **오른손은
`0x1xx`**(`121` 등)로 시작합니다.

AIDIN Hand Gen2가 연결되지 않았거나 전원이 꺼져 있으면 frame이 없고 RX packet은 0으로 유지됩니다
— link 설정 자체는 정상입니다. 연결하고 전원을 켠 뒤 다시 확인하십시오.

> [!NOTE]
> `can0`·`can1` 같은 이름은 부팅 순서와 hotplug에 따라 정해지므로, adapter를 다시 꽂거나
> 재부팅하면 왼손·오른손과 이름의 대응이 바뀔 수 있습니다. 대응이 바뀐 뒤에는 `candump`로
> 다시 확인하십시오.

## 2. Check the receive rate

기본 수신 확인을 마친 뒤 frame별 주기를 더 확인하려면 다음 명령을 사용합니다.
앞 절에서 지정한 `CAN` 변수를 같은 터미널에서 사용합니다.

각 state frame은 500 Hz로 들어옵니다. 다음 명령으로 ID별 수신 주파수를 확인하고 `Ctrl-C`로 종료합니다.

```bash
candump -t a "$CAN" | gawk '
BEGIN { win = 1.0; refresh = 1/60 }
{
  ts = $1; gsub(/[()]/, "", ts)
  id = $3
  q[id, ++tail[id]] = ts
  if (ts - drawn < refresh) next
  printf "\033[H\033[J"
  m = asorti(tail, ids)
  for (i = 1; i <= m; i++) {
    k = ids[i]
    while (head[k] < tail[k] && ts - q[k, head[k]+1] > win) delete q[k, ++head[k]]
    printf "0x%s: %d Hz\n", k, tail[k] - head[k]
  }
  drawn = ts
}'
```

state 네 줄(왼손 `0x221`~`0x224`, 오른손 `0x121`~`0x124`)이 각각 500 Hz 안팎이면 정상입니다.

설정한 interface 이름과 확인한 좌우 구분을 기록하십시오.
wrapper 설치 전이라면 [Installation](03_installation.md)으로, 설치를 마쳤다면
[2. Robot hand](04_bringup.md#2-robot-hand)로 진행합니다.
