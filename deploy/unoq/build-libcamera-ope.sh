#!/bin/sh
set -eu

if [ "$#" -lt 1 ]; then
    printf 'Usage: %s MEDIA_RUNTIME_DIR [meson setup options]\n' "$0" >&2
    exit 2
fi

board=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
mkdir -p "$1"
runtime=$(CDPATH= cd -- "$1" && pwd)
shift
revision=4ee1bd781f9fb056759b755d8b90754244c0193e
checksum=d4b998ecaed90bde94eac997a47297126629f5c675f7abeb7a5a749c1ecf2c9a
work=$(mktemp -d "$runtime/.build-libcamera.XXXXXX")
trap 'rm -rf -- "$work"' EXIT

if [ -n "${LIBCAMERA_SOURCE_ARCHIVE:-}" ]; then
    cp -- "$LIBCAMERA_SOURCE_ARCHIVE" "$work/source.tar.gz"
else
    curl -fLSs "https://codeload.github.com/jwrdegoede/libcamera/tar.gz/$revision" -o "$work/source.tar.gz"
fi
printf '%s  %s\n' "$checksum" "$work/source.tar.gz" | sha256sum -c
tar -xzf "$work/source.tar.gz" -C "$work"
source="$work/libcamera-$revision"
patch -d "$source" -p1 < "$board/libcamera-ope.patch"
patch -d "$source" -p1 < "$board/libcamera-imx219-denoise.patch"
install -m 644 "$board/imx219-raw-denoise.h" "$source/src/libcamera/pipeline/camss/"

meson setup "$work/build" "$source" --prefix "$runtime/unoq-libcamera" --libdir lib \
    --buildtype release --auto-features disabled -Dpipelines=camss,simple -Dipas=softisp \
    -Dgstreamer=enabled -Dtest=false -Dwerror=false "$@"
meson compile -C "$work/build"
DESTDIR="$work/install" meson install -C "$work/build" --strip
install -m 644 "$board/imx219.yaml" \
    "$work/install$runtime/unoq-libcamera/share/libcamera/ipa/softisp/imx219.yaml"

destination="$runtime/unoq-libcamera"
if [ -e "$destination" ]; then
    mv "$destination" "$work/previous"
fi
if ! mv "$work/install$destination" "$destination"; then
    if [ -d "$work/previous" ]; then mv "$work/previous" "$destination"; fi
    exit 1
fi
