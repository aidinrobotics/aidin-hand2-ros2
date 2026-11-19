#!/bin/sh
# Installs the SDK in this folder to /usr/local, where the packages of this repository find it:
#   sudo ./sdk/install.sh [--force]
# --force also replaces a newer SDK. DESTDIR=<dir> installs under <dir>/usr/local without root.
set -eu

here=$(cd "$(dirname "$0")" && pwd)
arch=$(uname -m)
src=$here/$arch
prefix=${DESTDIR:-}/usr/local

force=no
case ${1:-} in
  "") ;;
  --force) force=yes ;;
  *) echo "Usage: sudo $0 [--force]" >&2; exit 2 ;;
esac
[ -d "$src" ] || { echo "No SDK for $arch in $here, only for x86_64 and aarch64" >&2; exit 1; }
if [ -z "${DESTDIR:-}" ] && [ "$(id -u)" -ne 0 ]; then
  echo "Run with sudo: sudo $0" >&2
  exit 1
fi

new=$(sed -n 's/^sdk_version:[[:space:]]*//p' "$src/share/doc/aidin_hand2/BUILD-INFO")
old=$(sed -n 's/^set(PACKAGE_VERSION "\([0-9.]*\)")$/\1/p' \
  "$prefix/lib/cmake/aidin_hand2/aidin_hand2ConfigVersion.cmake" 2>/dev/null || true)
if [ -n "$old" ] && [ "$old" != "$new" ] && [ "$force" = no ] &&
  [ "$(printf '%s\n%s\n' "$old" "$new" | sort -V | tail -n 1)" = "$old" ]; then
  echo "SDK $old in $prefix is newer than $new. Run with --force to replace it." >&2
  exit 1
fi

# Whole folders, so that files the new SDK dropped do not stay behind
rm -rf "$prefix/include/aidin_hand2" "$prefix/lib/cmake/aidin_hand2" "$prefix/share/doc/aidin_hand2"
mkdir -p "$prefix/include" "$prefix/lib/cmake" "$prefix/share/doc"
# Other patch releases of this minor go; other minors stay for programs built against them
find "$prefix/lib" -maxdepth 1 -name "libaidin_hand2.so.${new%.*}.*" ! -name "libaidin_hand2.so.$new" -delete
cp -R "$src/include/aidin_hand2" "$prefix/include/"
cp -R "$src/lib/cmake/aidin_hand2" "$prefix/lib/cmake/"
cp -R "$src/share/doc/aidin_hand2" "$prefix/share/doc/"
cp -P --remove-destination "$src"/lib/libaidin_hand2.so* "$prefix/lib/"
[ -n "${DESTDIR:-}" ] || ldconfig

echo "Installed AIDIN Hand2 SDK $new in $prefix${old:+ (was $old)}"
