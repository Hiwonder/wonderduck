#!/bin/sh
set -eu

board=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
root=$(CDPATH= cd -- "$board/../.." && pwd)
cd "$root"
set -a
. "$board/board.env"
set +a

export PYTHONPATH="$board:$board/python${PYTHONPATH:+:$PYTHONPATH}"
export ORT_DYLIB_PATH="$root/onnxruntime-linux-aarch64-1.28.0/lib/libonnxruntime.so.1.28.0"
export WONDERDUCK_OBJECT_GPU_RUNNER="$root/target/release/object-gpu-runner"
export WONDERDUCK_UNOQ_TOF_ENABLED="${WONDERDUCK_UNOQ_TOF_ENABLED:-false}"
export WONDERDUCK_UNOQ_HEAD_IMU_ENABLED="${WONDERDUCK_UNOQ_HEAD_IMU_ENABLED:-false}"
export MEDIA_RUNTIME_DIR="$root/media-runtime-candidate"
export WONDERDUCK_ROBOT_PARAMS=${WONDERDUCK_ROBOT_PARAMS:-/etc/robot/robotd.toml}
export ROBOT_FREEZE_POLICY_IMU=0
export RUST_LOG=${RUST_LOG:-info}

exec "$root/target/release/robotd" \
    --params "$WONDERDUCK_ROBOT_PARAMS" --port "$WONDERDUCK_SERVO_PORT" \
    --enable-policies stand,ground_pick,sitstand,velocity,kick_left,kick_right \
    --head-lowpass 1 --legs-lowpass 1 --voltage-adapt false --imu-filter false \
    --max-linear 0.3 --max-lateral 0.25 --max-angular 1 \
    --imu-source id200 --imu-last --with-gamepad \
    --exit-head-30-ms 1000 --exit-head-31-ms 750,500 "$@"
