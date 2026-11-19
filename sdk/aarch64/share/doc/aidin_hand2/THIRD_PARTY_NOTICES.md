# Third-Party Notices

The AIDIN Hand Gen2 SDK is licensed under the Apache License 2.0; see [LICENSE](LICENSE)
and [NOTICE](NOTICE). It also includes third-party software. This file lists each
component, the license it is provided under, and where its source can be obtained. Full
license texts are in [licenses/](licenses), and an SDK install places this file and those
texts in `share/doc/aidin_hand2`.

Components reach a user of this SDK in two different ways, and the difference decides what
this repository has to carry.

- **Compiled into a binary distributed here.** Eigen is a header-only library. The kinematics
  archives shipped as `cpp/prebuilt/<arch>/libaidin_hand2_kinematics_type_<a|b>.a` are built from
  source files that include Eigen headers, so Eigen-derived code is part of those archives and
  of the `libaidin_hand2.so` that links them.
- **Included as source and compiled into the SDK.** spdlog and the fmt library it bundles are
  in `cpp/third_party/spdlog`, header-only, and every build compiles them into
  `libaidin_hand2.so`.

## Components

| Component | Version | License | How it is used |
|---|---|---|---|
| [Eigen](https://eigen.tuxfamily.org) | 3.4.0 | MPL-2.0, with BSD-3-Clause parts | Compiled into the kinematics archives, which `libaidin_hand2.so` links |
| [spdlog](https://github.com/gabime/spdlog) | 1.17.0 | MIT | Included in `cpp/third_party/spdlog`, compiled into `libaidin_hand2.so` |
| [fmt](https://github.com/fmtlib/fmt) | 12.1.0, bundled with spdlog | MIT with an optional exception | Included with spdlog, compiled into `libaidin_hand2.so` |
| Linux userspace API headers | kernel headers of the build host | BSD-3-Clause, dual-licensed with GPL-2.0-only WITH Linux-syscall-note | Included when the SDK is compiled |

## Eigen

Eigen is licensed under the Mozilla Public License 2.0, whose text is in
[licenses/MPL-2.0.txt](licenses/MPL-2.0.txt). Eigen is not modified in any way; the headers
are used as the distribution installs them.

The kinematics archives are distributed in this repository as an Executable Form of software
covered by the MPL, and so is a `libaidin_hand2.so` built from them. Section 3.2 of the MPL requires that recipients be informed of
this and be able to obtain the Source Code Form of the covered files. The covered files are
the unmodified Eigen headers, and the source is available from the Eigen project:

- Release archives: https://eigen.tuxfamily.org
- Repository: https://gitlab.com/libeigen/eigen

The MPL applies to the Eigen files themselves and does not extend to the AIDIN ROBOTICS
source in this repository, which stays under the Apache License 2.0. Section 3.3 of the MPL
permits this combination as long as the covered files remain available under the MPL, which
the links above satisfy.

Only the `Eigen/Dense` and `Eigen/Geometry` modules are used. The Eigen files that carry
LGPL terms — `Eigen/src/SparseCholesky/SimplicialCholesky.h`,
`Eigen/src/OrderingMethods/Amd.h`, `unsupported/Eigen/src/IterativeSolvers/*` — belong to the
sparse solver modules and are not reachable from those two headers, so no LGPL obligation
arises from this SDK.

Three parts of the Eigen headers that the SDK does compile carry additional or different
terms. Their notices are reproduced in
[licenses/BSD-3-Clause-Eigen.txt](licenses/BSD-3-Clause-Eigen.txt):

| File | Terms | Copyright |
|---|---|---|
| `Eigen/src/Core/util/MKL_support.h` | BSD-3-Clause | 2011 Intel Corporation |
| `Eigen/src/Geometry/AlignedBox.h`, `transform()` only | BSD-3-Clause | 2011-2014 Willow Garage, Inc.; 2014-2015 Open Source Robotics Foundation |
| `Eigen/src/LU/arch/InverseSize4.h` | MPL-2.0, retaining an Intel attribution notice | 2001 Intel Corporation |

## spdlog

spdlog provides the logging backend used by `src/logging/`. The headers of the 1.17.0 release
are included unmodified in `cpp/third_party/spdlog/include`, and
[cpp/third_party/spdlog/VERSION](cpp/third_party/spdlog/VERSION) records the tag, commit and
source archive they come from. They are compiled header-only into `libaidin_hand2.so`. spdlog
does not appear in the SDK's public headers, and its symbols are not exported from the library.

spdlog is licensed under the MIT license, whose text is in
[licenses/MIT-spdlog.txt](licenses/MIT-spdlog.txt). Source is available from
https://github.com/gabime/spdlog.

## fmt

spdlog 1.17.0 bundles fmt 12.1.0 under `spdlog/fmt/bundled`, and the SDK uses that copy, so fmt
is compiled into `libaidin_hand2.so` along with spdlog.

fmt is licensed under the MIT license with an optional exception that waives the notice
requirement for portions embedded in object form. The full text is in
[licenses/MIT-fmt.txt](licenses/MIT-fmt.txt), and source is available from
https://github.com/fmtlib/fmt.

## Linux userspace API headers

The CAN-FD transport is built on SocketCAN and includes the kernel's userspace API headers.
These headers declare the socket and frame structures the kernel exposes; the SDK does not
link any kernel code.

| Header | SPDX identifier |
|---|---|
| `linux/can.h`, `linux/can/error.h`, `linux/can/raw.h` | `(GPL-2.0-only WITH Linux-syscall-note) OR BSD-3-Clause` |

The SDK takes the BSD-3-Clause option for these three headers, so no GPL term applies to
them. The notice is reproduced in
[licenses/BSD-3-Clause-SocketCAN.txt](licenses/BSD-3-Clause-SocketCAN.txt).

## can-utils

The [CAN-FD setup](docs/en/05_can_fd_setup.md) guide installs can-utils for inspecting a CAN
interface. The SDK never invokes it, and no code path starts an external process. can-utils
is therefore a separate program a user may choose to run, not a dependency of this software,
and its license imposes no obligation on this repository.
