#!/bin/sh
set -eu

if [ "$#" -ne 2 ]; then
    printf 'Usage: %s install|rollback MEDIA_RUNTIME_DIR\n' "$0" >&2
    exit 2
fi

bundle=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd -P)
case "$1" in
    install) new=libcamera-denoise.so.0.7.2; old=libcamera-previous.so.0.7.2 ;;
    rollback) new=libcamera-previous.so.0.7.2; old=libcamera-denoise.so.0.7.2 ;;
    *) printf 'Select install or rollback.\n' >&2; exit 2 ;;
esac
runtime=$(CDPATH= cd -- "$2" && pwd -P)
libdir="$runtime/unoq-libcamera/lib"
target="$libdir/libcamera.so.0.7.2"
if [ "$(readlink -f -- "$target")" != "$target" ] || [ ! -f "$target" ]; then
    printf 'Private UNO Q libcamera 0.7.2 could not be found in %s\n' "$libdir" >&2
    exit 1
fi

cd "$bundle"
sha256sum --strict -c SHA256SUMS
exec 9<"$libdir"
flock -x 9
if cmp -s -- "$new" "$target"; then
    printf 'The selected camera library is already installed.\n'
    exit 0
fi
if ! cmp -s -- "$old" "$target"; then
    printf 'This package was built for a different private camera runtime.\n' >&2
    exit 1
fi

temporary=$(mktemp "$libdir/.libcamera.XXXXXXXX")
trap 'rm -f -- "$temporary"' EXIT
trap 'exit 1' HUP INT TERM
cat "$new" > "$temporary"
chown --reference="$target" -- "$temporary"
chmod --reference="$target" -- "$temporary"
mv -T -- "$temporary" "$target"
printf 'Camera library installed. It will load the next time the camera starts.\n'
