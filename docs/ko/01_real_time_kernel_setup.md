# Real-time kernel setup

SDK의 500 Hz 제어·통신 루프와 `controller_manager`의 update 루프를 실시간으로 스케줄링하려면
`PREEMPT_RT` kernel과, root가 아닌 user가 real-time priority(`SCHED_FIFO`)로 실행하고 메모리를 잠글
수 있는 permission이 필요합니다. 이 문서는 Ubuntu(x86_64 / arm64)에서 PREEMPT_RT kernel을 source에서
빌드하고 실행 권한을 설정합니다. mock 실행에는 이 설정이 필요하지 않습니다. SDK 문서의
[Real-time kernel setup](https://github.com/aidinrobotics/aidin-hand2-sdk/blob/main/docs/ko/04_real_time_kernel_setup.md)과
같은 내용입니다.

## Contents

&nbsp;&nbsp;[**1. Build the RT kernel**](#1-build-the-rt-kernel)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.1 Install build dependencies](#11-install-build-dependencies)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.2 Get the kernel source](#12-get-the-kernel-source)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.3 Verify signatures](#13-verify-signatures)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.4 Apply the RT patch](#14-apply-the-rt-patch)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.5 Configure the kernel](#15-configure-the-kernel)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.6 Build & install](#16-build--install)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.7 Boot into the kernel](#17-boot-into-the-kernel)<br>
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;[1.8 Verify the kernel](#18-verify-the-kernel)<br>
&nbsp;&nbsp;[**2. Real-time settings**](#2-real-time-settings)<br>
&nbsp;&nbsp;[**3. Remove the custom kernel**](#3-remove-the-custom-kernel)

## 1. Build the RT kernel

### 1.1 Install build dependencies

```bash
sudo apt update
sudo apt install -y build-essential bc bison flex gawk \
  libssl-dev libelf-dev libncurses-dev dwarves \
  fakeroot rsync cpio kmod zstd gnupg dirmngr curl xz-utils
```

> [!NOTE]
> 위 목록은 kernel 6.12 기준입니다. 빌드 의존성은 kernel version과 `.config`에 따라 달라지므로,
> 설치하려는 kernel version에서 필요한 패키지는 source 트리의 `Documentation/process/changes.rst`에서
> 확인하십시오.


### 1.2 Get the kernel source

예제는 LTS kernel 6.12(6.12.100 / rt20)를 씁니다. RT patch가 유지되는 kernel은
[PREEMPT_RT versions](https://wiki.linuxfoundation.org/realtime/preempt_rt_versions)에서
확인하고, [kernel source](https://cdn.kernel.org/pub/linux/kernel/v6.x/)에 `linux-6.12.100.tar.xz`가,
[RT patch](https://cdn.kernel.org/pub/linux/kernel/projects/rt/6.12/)에
`patch-6.12.100-rt20.patch.xz`가 있는지 확인해 아래 변수에 넣습니다.

```bash
ver=6.12.100                   # ← 직접 입력: kernel version
rt=rt20                        # ← 직접 입력: 해당 ver의 RT revision (rtN)
major="${ver%%.*}"             # 6    (source 디렉터리 v6.x)
base="${ver%.*}"               # 6.12 (RT patch 디렉터리 major.minor)
kroot=https://cdn.kernel.org/pub/linux/kernel
```

source·patch와 각각의 signature를 받습니다.

```bash
curl -LO "${kroot}/v${major}.x/linux-${ver}.tar.xz"
curl -LO "${kroot}/v${major}.x/linux-${ver}.tar.sign"
curl -LO "${kroot}/projects/rt/${base}/patch-${ver}-${rt}.patch.xz"
curl -LO "${kroot}/projects/rt/${base}/patch-${ver}-${rt}.patch.sign"
```

### 1.3 Verify signatures

먼저 key 없이 signature 검증을 시도해, 출력의 `RSA key <FINGERPRINT>`부터 확인합니다. 아직 key가
등록되지 않아 `No public key`가 나오는 게 정상입니다.

```bash
xz -cd "linux-${ver}.tar.xz" | gpg --verify "linux-${ver}.tar.sign" -
```

출력된 `<FINGERPRINT>`를 [kernel.org signatures](https://www.kernel.org/signature.html)의
fingerprint와 대조합니다 (stable=Greg KH 또는 Sasha Levin, mainline=Linus). 일치하면 해당
signer의 key를 등록합니다.

```bash
gpg --locate-keys gregkh@kernel.org       # stable — Greg KH
# gpg --locate-keys sashal@kernel.org     # stable — Sasha Levin
# gpg --locate-keys torvalds@kernel.org   # mainline — Linus
```

key를 등록한 뒤 source·patch의 signature를 다시 검증합니다. `Good signature`가 나오고 fingerprint가
위와 같으면 통과입니다 (뒤따르는 `WARNING: not certified` 는 무시 — fingerprint를 직접
대조했으므로 무방).

```bash
xz -cd "linux-${ver}.tar.xz"         | gpg --verify "linux-${ver}.tar.sign" -
xz -cd "patch-${ver}-${rt}.patch.xz" | gpg --verify "patch-${ver}-${rt}.patch.sign" -
```

### 1.4 Apply the RT patch

signature 검증을 통과했으면 source tarball을 압축 해제하고 RT patch를 적용합니다.

```bash
tar xf "linux-${ver}.tar.xz"
cd "linux-${ver}"
xzcat "../patch-${ver}-${rt}.patch.xz" | patch -p1
```

### 1.5 Configure the kernel

현재 실행 중인 kernel의 config(`/boot/config-$(uname -r)`)를 기준으로 복사합니다 — CAN driver를
포함한 현재 하드웨어 설정이 그대로 유지됩니다. 여기에 real-time preemption을 켜고, 로컬에 없는
Canonical signing certificate 경로를 비운 뒤(그대로 두면 빌드가 없는 파일을 찾다 실패),
`olddefconfig`로 새 옵션을 기본값으로 채웁니다.

> [!NOTE]
> `cp`는 **지금 부팅해서 작동 중인** kernel의 config를 복사합니다. 이미 직접 빌드한 kernel로
> 부팅한 상태라면 Ubuntu 기본 config가 아니라 그 kernel의 config를 물려받으니, 시작점이
> 의도한 config가 맞는지 확인하십시오.

```bash
cp /boot/config-"$(uname -r)" .config
scripts/config --enable  CONFIG_PREEMPT_RT
scripts/config --set-str CONFIG_SYSTEM_TRUSTED_KEYS ""      # signing certificate 경로 비우기
scripts/config --set-str CONFIG_SYSTEM_REVOCATION_KEYS ""
make olddefconfig                                          # 위 변경 반영·새 옵션 기본값 채우기
```

CAN driver가 config에 남아 있는지 확인합니다.

```bash
grep -E 'CONFIG_CAN=|CONFIG_CAN_DEV|CONFIG_CAN_RAW|CONFIG_CAN_PEAK_USB|CONFIG_CAN_GS_USB' .config
```

누락됐다면 SocketCAN 코어와 어댑터 driver를 켜고 `olddefconfig`를 다시 실행합니다. 우리가 쓰는
어댑터는 PEAK PCAN-USB FD(driver `peak_usb`)와 candleLight 기반 CANable Pro 2.0(driver
`gs_usb`)이므로 driver 2개를 함께 켭니다.

```bash
scripts/config --module CONFIG_CAN
scripts/config --module CONFIG_CAN_DEV
scripts/config --module CONFIG_CAN_RAW
scripts/config --module CONFIG_CAN_PEAK_USB   # ← PCAN-USB FD
scripts/config --module CONFIG_CAN_GS_USB     # ← CANable Pro 2.0 (candleLight)
make olddefconfig
```

다른 USB-CAN 어댑터라면 해당 driver 줄을 함께 넣으십시오 — `CONFIG_CAN_KVASER_USB`(Kvaser),
`CONFIG_CAN_8DEV_USB`(8devices), `CONFIG_CAN_ESD_USB`(esd) 등. 전체 목록은 kernel source의
`drivers/net/can/usb/Kconfig`에 있습니다. module(`--module`)로 켜면 쓰지 않는 driver는 로드되지
않으니, 확실치 않으면 여러 개를 함께 켜도 무방합니다.

### 1.6 Build & install

```bash
make -j$(nproc) bindeb-pkg
sudo IGNORE_PREEMPT_RT_PRESENCE=1 dpkg -i ../linux-image-*.deb ../linux-headers-*.deb
```

`IGNORE_PREEMPT_RT_PRESENCE=1`은 설치 중 DKMS 모듈(예: NVIDIA driver)이 RT kernel을 감지하면
빌드를 거부하는 검사를 우회합니다. DKMS 모듈이 없으면 무해합니다.

### 1.7 Boot into the kernel

방금 설치한 RT kernel은 기본 부팅 kernel이 아닐 수 있습니다. 재부팅한 뒤 GRUB 메뉴의 **Advanced
options for Ubuntu**에서 방금 빌드한 version을 선택하십시오 — 그대로 재부팅하면 기존 kernel로
부팅되어 다음 절의 검증을 통과하지 못합니다.

> [!WARNING]
> Secure Boot가 켜져 있으면 unsigned 자체 빌드 kernel은 부팅이 거부됩니다("Invalid
> Signature"). UEFI/BIOS에서 Secure Boot를 해제한 뒤 다시 부팅하십시오.

```bash
sudo reboot
```

이 kernel을 계속 쓸 것이라면, 매번 GRUB에서 고르지 않도록 기본 부팅 kernel로
고정하는 것을 권장합니다(`/etc/default/grub`의 `GRUB_DEFAULT` 설정 후 `sudo update-grub`).

### 1.8 Verify the kernel

재부팅 후 RT kernel이 실행 중인지 확인합니다.

```bash
uname -a                   # 빌드한 version과 PREEMPT_RT 표기
cat /sys/kernel/realtime   # RT kernel이면 1 (없으면 RT 아님)
```

## 2. Real-time settings

root가 아닌 user가 `SCHED_FIFO` priority와 `mlockall`을 쓰려면 permission이 필요합니다. `realtime`
group을 만들어 user를 넣고 limit을 부여합니다.

```bash
sudo groupadd -r realtime
sudo usermod -aG realtime "$USER"
sudo tee /etc/security/limits.d/99-realtime.conf >/dev/null <<'EOF'
@realtime   -   rtprio      99
@realtime   -   memlock     unlimited
EOF
```

SDK는 제어·통신 루프를 priority 90으로 올리고 `controller_manager`는 기본값 50으로 update 루프를
돌리므로 `rtprio`는 90 이상이어야 합니다. `mlockall`은 이후 할당분까지 잠그므로 `memlock`은
`unlimited`여야 합니다.

로그아웃 후 다시 로그인해 새 세션에서 확인합니다.

```bash
id -nG         # 현재 user의 group → realtime 포함 확인
ulimit -r      # rtprio  → 99
ulimit -l      # memlock → unlimited
```

## 3. Remove the custom kernel

문제가 있어 되돌릴 때만 수행합니다. 먼저 GRUB에서 기존 Ubuntu kernel을 선택해 부팅하십시오.
`uname -r`로 현재 kernel이 제거할 대상과 다른지 확인하고 설치된 package 이름을 조회합니다.

```bash
uname -r
dpkg -l 'linux-image-*' 'linux-headers-*'
```

제거할 kernel의 정확한 release를 지정합니다. 다음 값은 예시이며 위 목록의 package 이름에 맞게
바꾸십시오. 현재 실행 중인 kernel과 다시 부팅할 기존 kernel은 제거하지 않습니다.

```bash
kernel_release=6.12.100-rt20  # ← 직접 입력: 제거할 kernel release
sudo dpkg -r "linux-image-${kernel_release}" "linux-headers-${kernel_release}"
```

설정을 마쳤다면 [CAN-FD setup](02_can_fd_setup.md)에서 통신을 준비하십시오.
