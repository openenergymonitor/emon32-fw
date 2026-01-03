#!/bin/bash
set -e

UF2_FILE="build/emon32.uf2"
DO_BUILD=0

echo "=== emonPi3/TX6 UF2 Flash Script ==="

#
# Parse arguments
#
while [[ $# -gt 0 ]]; do
    case "$1" in
        --build)
            DO_BUILD=1
            shift
            ;;
        *)
            echo "Unknown option: $1"
            echo "Usage: $0 [--build]"
            exit 1
            ;;
    esac
done

#
# Check picocom is installed
#
if ! command -v picocom >/dev/null 2>&1; then
    echo "ERROR: picocom is not installed."
    echo "Install it with: sudo apt install picocom"
    exit 1
fi

#
# Optional build step
#
if [[ $DO_BUILD -eq 1 ]]; then
    echo "Running build..."
    if ! make -j; then
        echo "ERROR: Build failed."
        exit 1
    fi
fi

#
# Ensure UF2 exists
#
if [ ! -f "$UF2_FILE" ]; then
    echo "ERROR: UF2 file not found at $UF2_FILE"
    exit 1
fi

echo "Checking device state..."

#
# 1. Check if device is already in BOOTLOADER MODE
#
BOOT_DEV=$(ls /dev/disk/by-id/usb-emonPi3_* 2>/dev/null || true)

if [ -n "$BOOT_DEV" ]; then
    echo "Device is already in bootloader mode."
    DEV="$BOOT_DEV"
else
    echo "Device not in bootloader mode. Checking for normal mode..."

    #
    # 2. Detect NORMAL MODE via serial device ID (*Pi3*)
    #
    SERIAL_LINK=$(ls /dev/serial/by-id/*Pi3* 2>/dev/null || true)

    if [ -z "$SERIAL_LINK" ]; then
        echo "ERROR: Device not detected in normal or bootloader mode."
        echo "Is it connected?"
        exit 1
    fi

    echo "Device found in normal mode."

    #
    # 3. Resolve serial device path
    #
    REAL_SERIAL=$(readlink -f "$SERIAL_LINK")
    echo "Using serial device: $REAL_SERIAL"

    #
    # 4. Send 'e' + CRLF, wait, then 'y' + CRLF via picocom
    #
    echo "Sending bootloader command (e + y)..."
    (
        sleep 0.2
        printf "e\r\n"
        sleep 0.2
        printf "y\r\n"
    ) | picocom -q -q --baud 115200 --omap crcrlf,crlf "$REAL_SERIAL" 2>/dev/null

    echo "Bootloader command sent. Waiting for device to re-enumerate..."

    #
    # 5. Give USB stack a moment to settle
    #
    sleep 1

    #
    # 6. Wait for bootloader device to appear
    #
    while true; do
        DEV=$(ls /dev/disk/by-id/usb-emonPi3_* 2>/dev/null || true)
        if [ -n "$DEV" ]; then
            echo "Bootloader device detected: $DEV"
            break
        fi
        echo "Waiting for bootloader device..."
        sleep 1
    done
fi

#
# 7. Check if already mounted
#
echo "Checking mount state..."
INFO=$(udisksctl info -b "$DEV")

if echo "$INFO" | grep -q "Mounted: yes"; then
    MOUNT_POINT=$(echo "$INFO" | sed -n 's/ *MountPoints: \(.*\)/\1/p')
    echo "Device is already mounted at: $MOUNT_POINT"
else
    echo "Mounting device..."
    MOUNT_OUTPUT=$(sudo udisksctl mount -b "$DEV")

    # Extract mount point robustly
    MOUNT_POINT=$(echo "$MOUNT_OUTPUT" | grep -oE "at (/[^ ]+)" | awk '{print $2}')

    if [ -z "$MOUNT_POINT" ]; then
        echo "ERROR: Could not determine mount point"
        echo "udisksctl output was:"
        echo "$MOUNT_OUTPUT"
        exit 1
    fi

    echo "Mounted at: $MOUNT_POINT"
fi

#
# 8. Copy the UF2 file
#
echo "Copying UF2 file..."
sudo cp "$UF2_FILE" "$MOUNT_POINT/"

echo "Syncing..."
sync

echo "Flash complete. Device will reboot automatically."
