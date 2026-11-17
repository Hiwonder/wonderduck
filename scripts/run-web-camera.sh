#!/bin/sh
set -eu

source_dir=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
runtime=${MEDIA_RUNTIME_DIR:-"$source_dir/media-runtime-candidate"}

camera_available() {
  lock=$runtime/webpadd-media.sock.lock
  [ -e "$lock" ] || return 0
  if flock -n -E 77 "$lock" true; then
    return 0
  else
    status=$?
    [ "$status" -eq 77 ] && return 1
    printf 'cannot inspect camera ownership (flock exit %s)\n' "$status" >&2
    return 2
  fi
}

if [ "${1:-}" = "--check" ]; then
  if [ "$#" -ne 1 ]; then
    printf '%s\n' 'camera check takes no additional arguments' >&2
    exit 2
  fi
  [ -c /dev/video0 ] || exit 1
  if ! command -v media-ctl >/dev/null 2>&1; then
    printf '%s\n' 'media-ctl is required to inspect camera topology' >&2
    exit 2
  fi
  failed=0
  for media in /dev/media[0-9]*; do
    [ -c "$media" ] || continue
    if topology=$(timeout 2s media-ctl -d "$media" -p 2>&1); then
      if printf '%s\n' "$topology" | awk '
        /^- entity / { sensor_entity = 0 }
        /type V4L2 subdev subtype Sensor/ { sensor_entity = 1 }
        sensor_entity && /-> .*\[ENABLED\]/ { sensor = 1 }
        /^[[:space:]]*device node name \/dev\/video0$/ { capture = 1 }
        END { exit !(sensor && capture) }
      '; then
        if camera_available; then
          exit 0
        else
          status=$?
          [ "$status" -eq 1 ] && exit 3
          exit 2
        fi
      fi
    else
      printf '%s: %s\n' "$media" "$topology" >&2
      failed=1
    fi
  done
  [ "$failed" -eq 0 ] || exit 2
  exit 1
fi

exec 9>>"$runtime/camera-launch.lock"
if flock -n -E 77 9; then
  :
else
  status=$?
  if [ "$status" -eq 77 ]; then
    printf '%s\n' 'camera launcher already running' >&2
    exit 3
  fi
  printf 'cannot lock camera launcher (flock exit %s)\n' "$status" >&2
  exit 2
fi
if camera_available; then
  :
else
  status=$?
  if [ "$status" -eq 1 ]; then
    printf '%s\n' 'camera stream already running' >&2
    exit 3
  fi
  exit 2
fi

rotation_set=
for arg do
  case "$arg" in
    --rotate|--rotate=*) rotation_set=1 ;;
  esac
done
if [ -z "$rotation_set" ]; then
  set -- --rotate 0 "$@"
fi

: "${WEBRTCSINK_SIGNALLING_SERVER_LOG:=info,gst_plugin_webrtc_signalling::server=warn}"
export WEBRTCSINK_SIGNALLING_SERVER_LOG

# MPP logs every adaptive bitrate change at INFO. Keep warnings and errors, and
# allow an explicit higher level when diagnosing the encoder.
: "${mpp_log_level:=3}"
export mpp_log_level

exec "$runtime/run" mediad \
  --no-remote \
  --config /etc/robot/robotd.toml \
  --frame-socket "$runtime/webpadd-media.sock" \
  --robot-socket /dev/null/robot.sock \
  --tof-socket /dev/null/tof.sock \
  --config-socket /dev/null/config.sock \
  --pad-socket /dev/null/pad.sock \
  --updater-socket /dev/null/updater.sock \
  "$@"
