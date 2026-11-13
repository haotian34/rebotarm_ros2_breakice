#!/usr/bin/env bash
# Run with bash; this configures access and never starts the robot.
set -euo pipefail
if [[ $# -ne 1 || "$1" == --help ]]; then
    printf 'Usage: bash scripts/setup_serial.sh /dev/serial/by-id/<device>\n'
    printf 'A current device path such as /dev/ttyACM0 also works.\n'
    exit 0
fi
serial_device=$(readlink -f -- "$1")
if [[ ! -c "$serial_device" ]]; then
    printf 'Serial device does not exist or is not a character device: %s\n' "$1" >&2
    exit 1
fi
case "$serial_device" in
    /dev/ttyACM*|/dev/ttyUSB*) ;;
    *) printf 'Expected a USB serial device (ttyACM or ttyUSB): %s\n' "$serial_device" >&2; exit 1 ;;
esac
serial_user=${SUDO_USER:-$(id -un)}
serial_group=$(stat -c %G -- "$serial_device")
if [[ "$serial_user" == root || "$serial_group" == root || "$serial_group" == UNKNOWN ]]; then
    printf 'Refusing to grant root-group access; run as a regular user and check device rules.\n' >&2
    exit 1
fi
if id -nG "$serial_user" | tr ' ' '\n' | grep -Fxq "$serial_group"; then
    printf '%s is already a member of %s.\n' "$serial_user" "$serial_group"
else
    printf 'Adding %s to the device group %s (requires sudo).\n' "$serial_user" "$serial_group"
    sudo usermod -aG "$serial_group" "$serial_user"
fi
printf 'Log out of the desktop and log back in to refresh group permissions.\n'
printf 'Then run: startreal channel:=%q\n' "$1"
