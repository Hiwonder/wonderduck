#!/bin/sh
set -eu

tr '\0' '\n' </proc/device-tree/compatible | grep -qx arduino,imola || exit 0

card_ready() {
    grep -q ArduinoImolaHPH /proc/asound/cards 2>/dev/null
}

# The Arduino boot fixup may have just started the DSP. Give its audio
# services and the dependent codec drivers time to register the sound card.
for attempt in $(seq 1 45); do
    card_ready && exit 0
    sleep 1
done
card_ready && exit 0

pinctrl=/sys/bus/platform/devices/a7c0000.pinctrl
if [ ! -d "$pinctrl" ] || [ -L "$pinctrl/driver" ]; then
    echo "ArduinoImolaHPH is missing without the LPASS pinctrl probe failure." >&2
    exit 1
fi

for dsp in /sys/class/remoteproc/*; do
    [ -r "$dsp/name" ] && [ "$(cat "$dsp/name")" = adsp ] || continue
    if [ "$(cat "$dsp/state")" != running ]; then
        echo "ArduinoImolaHPH is missing and the audio DSP did not start." >&2
        exit 1
    fi
    echo "ArduinoImolaHPH is missing and LPASS pinctrl is unbound. Restarting the audio DSP."
    printf 'stop\n' > "$dsp/state"
    printf 'start\n' > "$dsp/state"
    for attempt in $(seq 1 25); do
        if card_ready; then
            echo "ArduinoImolaHPH registered after the audio DSP restart."
            exit 0
        fi
        sleep 1
    done
    echo "ArduinoImolaHPH did not register after the audio DSP restart." >&2
    exit 1
done

echo "ArduinoImolaHPH is missing and the audio DSP was not found." >&2
exit 1
