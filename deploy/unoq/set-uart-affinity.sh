#!/bin/sh
set -eu

irq=$(awk '$NF == "qcom_geni_serial_uart1" { gsub(":", "", $1); print $1; exit }' /proc/interrupts)
case "$irq" in
    ''|*[!0-9]*) echo "UNO Q router UART interrupt not found" >&2; exit 1 ;;
esac
printf '%s\n' 3 > "/proc/irq/$irq/smp_affinity_list"
