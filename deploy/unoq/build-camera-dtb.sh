#!/bin/sh
set -eu

board_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
base_dtb=${1:-/boot/efi/dtb/qcom/qrb2210-arduino-imola.dtb}
test_dir=$(mktemp -d /tmp/wonderduck-unoq-camera.XXXXXX)
test_dtb="$test_dir/qrb2210-arduino-imola-wonderduck-csi1-test.dtb"

trap 'rm -r -- "$test_dir"' EXIT

invalid_input() {
    printf '%s\n' "$1" >&2
    exit 1
}

camss=$(fdtget -t s "$base_dtb" /__symbols__ camss)
cci=$(fdtget -t s "$base_dtb" /__symbols__ cci)
cci1=$(fdtget -t s "$base_dtb" /__symbols__ cci_i2c1)
power=/cam-pwr-csi1
if fdtget -p "$base_dtb" "$power" >/dev/null 2>&1; then
    # App Lab already built the sensor and both endpoints for the selected lane count.
    sensor="$cci1/sensor@10"
    test "$(fdtget -t s "$base_dtb" "$sensor" compatible)" = sony,imx219 ||
        invalid_input "Select Camera 1 (IMX219) in App Lab before building this candidate."
    cci0=$(fdtget -t s "$base_dtb" /__symbols__ cci_i2c0)
    expander="$cci0/gpio@26"
    test "$(fdtget -t s "$base_dtb" "$expander" compatible)" = nxp,pca9555 ||
        invalid_input "Unexpected App Lab camera GPIO controller."
    expander_handle=$(fdtget -t u "$base_dtb" "$expander" phandle)
    test "$(fdtget -t u "$base_dtb" "$power" gpio)" = "$expander_handle 3 0" &&
        test "$(fdtget -t u "$base_dtb" "$sensor" reset-gpios)" = "$expander_handle 2 0" ||
        invalid_input "Unexpected App Lab camera enable or reset wiring."
    power_handle=$(fdtget -t u "$base_dtb" "$power" phandle)
    for supply in VANA VDIG VDDL; do
        test "$(fdtget -t u "$base_dtb" "$sensor" "$supply-supply")" = "$power_handle" ||
            invalid_input "Unexpected IMX219 $supply supply."
    done

    tlmm=$(fdtget -t s "$base_dtb" /__symbols__ tlmm)
    tlmm_handle=$(fdtget -t u "$base_dtb" "$tlmm" phandle)
    cp -- "$base_dtb" "$test_dtb"
    fdtput -t u "$test_dtb" "$power" gpio "$tlmm_handle" 99 0
    # J7 has a single enable line, with no separate reset line to drive.
    fdtput -d "$test_dtb" "$sensor" reset-gpios
else
    test -z "$(fdtget -l "$base_dtb" "$cci1")" ||
        invalid_input "CCI1 already has a device. Select the App Lab Camera 1 profile."
    sensor="$cci1/imx219@10"
    overlay="$test_dir/wonderduck-exp-camera.dtbo"
    dtc -q -@ -I dts -O dtb -o "$overlay" \
        "$board_dir/overlays/wonderduck-exp-camera.dts"
    fdtoverlay -i "$base_dtb" -o "$test_dtb" "$overlay"
    rm "$overlay"
fi

# The QCM2290 CAMSS driver names these rails differently from the App Lab overlay.
# Reuse the board's existing PMIC references for PHY analog power and its PLL.
if ! fdtget -t u "$test_dtb" "$camss" vdd-csiphy-1p2-supply >/dev/null 2>&1; then
    phy_1p2=$(fdtget -t u "$test_dtb" "$camss" vdda-csiphy-1p2-supply)
    fdtput -t u "$test_dtb" "$camss" vdd-csiphy-1p2-supply "$phy_1p2"
fi
if ! fdtget -t u "$test_dtb" "$camss" vdd-csiphy-1p8-supply >/dev/null 2>&1; then
    phy_1p8=$(fdtget -t u "$test_dtb" "$camss" vdda-pll-1p8-supply)
    fdtput -t u "$test_dtb" "$camss" vdd-csiphy-1p8-supply "$phy_1p8"
fi

for node in "$camss" "$cci" "$cci1" "$sensor"; do
    fdtget -p "$test_dtb" "$node" >/dev/null
    test "$(fdtget -t s -d okay "$test_dtb" "$node" status)" = okay ||
        invalid_input "Camera node is disabled: $node"
done
test "$(fdtget -t s "$test_dtb" "$sensor" compatible)" = sony,imx219

trap - EXIT
printf '%s\n' "$test_dtb"
