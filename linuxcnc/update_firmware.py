#!/usr/bin/env python3
"""Pico CNC HMI Firmware Updater

Reboots the Pico into USB BOOTSEL mode, copies the .uf2 firmware image,
and waits for the device to come back online with the new firmware.

Usage:
    python3 update_firmware.py [path/to/firmware.uf2]

If no path is given, defaults to ../firmware/build/pico_cnc_hmi.uf2
relative to this script's directory.
"""

import glob
import os
import shutil
import struct
import sys
import time

# Try both common hid module names
try:
    import hid
except ImportError:
    print("Error: python-hid (hidapi) not installed.", file=sys.stderr)
    print("  pip install hid   or   pip install hidapi", file=sys.stderr)
    sys.exit(1)

VENDOR_ID   = 0xCAFE
CMD_BOOTSEL = 0xB0
MOUNT_GLOBS = [
    "/media/*/RPI-RP2",
    "/media/*/RP2350",
    "/media/*/*/RPI-RP2",
    "/media/*/*/RP2350",
    "/run/media/*/RPI-RP2",
    "/run/media/*/RP2350",
    "/mnt/RPI-RP2",
    "/mnt/RP2350",
]
TIMEOUT_MOUNT = 15   # seconds to wait for mass storage to appear
TIMEOUT_FLASH = 30   # seconds to wait for device to reboot after copy
TIMEOUT_HID   = 15   # seconds to wait for HID to reappear


def find_hid_device():
    """Find the HMI's Generic HID interface (usage_page 0xFF00 or interface 0)."""
    for d in hid.enumerate(VENDOR_ID):
        if d.get('usage_page', 0) == 0xFF00:
            return d
    for d in hid.enumerate(VENDOR_ID):
        if d.get('interface_number', -1) == 0:
            return d
    return None


def find_bootsel_mount():
    """Find the RPI-RP2 mass storage mount point."""
    for pattern in MOUNT_GLOBS:
        matches = glob.glob(pattern)
        if matches:
            return matches[0]
    return None


def wait_for(description, check_fn, timeout):
    """Poll check_fn() every 0.5s until it returns truthy or timeout."""
    print(f"  Waiting for {description}...", end="", flush=True)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = check_fn()
        if result:
            print(" OK")
            return result
        print(".", end="", flush=True)
        time.sleep(0.5)
    print(" TIMEOUT")
    return None


def main():
    # Resolve .uf2 path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_uf2 = os.path.join(script_dir, "..", "firmware", "build", "pico_cnc_hmi.uf2")

    uf2_path = sys.argv[1] if len(sys.argv) > 1 else default_uf2
    uf2_path = os.path.abspath(uf2_path)

    if not os.path.isfile(uf2_path):
        print(f"Error: firmware file not found: {uf2_path}", file=sys.stderr)
        sys.exit(1)

    uf2_size = os.path.getsize(uf2_path)
    print(f"Firmware: {uf2_path} ({uf2_size:,} bytes)")

    # Step 1: Find and reboot the HMI
    print("\n[1/4] Connecting to HMI...")
    info = find_hid_device()
    if info is None:
        print("Error: HMI device not found. Is it plugged in?", file=sys.stderr)
        sys.exit(1)

    dev = hid.device()
    dev.open_path(info['path'])
    product = info.get('product_string', 'HMI')
    print(f"  Found: {product}")

    print("\n[2/4] Sending BOOTSEL reboot command...")
    pkt = bytearray(64)
    pkt[0] = 0xAA        # header
    pkt[14] = CMD_BOOTSEL # reboot command
    dev.write(bytes(pkt))
    dev.close()
    print("  Command sent, device is rebooting...")

    # Step 2: Wait for mass storage to appear
    print(f"\n[3/4] Waiting for USB mass storage (up to {TIMEOUT_MOUNT}s)...")
    mount = wait_for("RPI-RP2 mount", find_bootsel_mount, TIMEOUT_MOUNT)
    if mount is None:
        print("Error: RPI-RP2 mass storage did not appear.", file=sys.stderr)
        print("  Check if your OS auto-mounts USB drives.", file=sys.stderr)
        sys.exit(1)

    # Step 3: Copy the firmware
    dest = os.path.join(mount, os.path.basename(uf2_path))
    print(f"  Copying to {dest}...")
    shutil.copy2(uf2_path, dest)
    # Sync to ensure the write is flushed to the device
    os.sync()
    print("  Copy complete. Pico is flashing...")

    # Step 4: Wait for the Pico to reboot with new firmware
    print(f"\n[4/4] Waiting for HMI to come back online (up to {TIMEOUT_HID}s)...")
    # First wait for mass storage to disappear (flash in progress)
    wait_for("mass storage to unmount",
             lambda: find_bootsel_mount() is None,
             TIMEOUT_FLASH)

    # Then wait for HID to reappear
    result = wait_for("HID device", find_hid_device, TIMEOUT_HID)
    if result:
        print(f"\n✓ Firmware update complete! Device is running.")
    else:
        print(f"\n⚠ Firmware was copied but device hasn't re-enumerated yet.")
        print("  It may still be booting. Check manually.")


if __name__ == '__main__':
    main()
