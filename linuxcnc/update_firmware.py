#!/usr/bin/env python3
"""Pico CNC HMI Firmware Updater

Reboots the Pico into USB BOOTSEL mode, copies the .uf2 firmware image,
and waits for the device to come back online with the new firmware.

Usage:
    python3 update_firmware.py [options] [path/to/firmware.uf2]

Options:
    --check   Report running and .uf2 versions without updating.
    --force   Flash even if the .uf2 is not newer than the running firmware.

If no path is given, defaults to ../firmware/build/pico_cnc_hmi.uf2
relative to this script's directory.
"""

import argparse
import glob
import os
import signal
import shutil
import struct
import subprocess
import sys
import time

try:
    import hid
except ImportError:
    print("Error: python-hid (hidapi) not installed.", file=sys.stderr)
    print("  pip install hid   or   pip install hidapi", file=sys.stderr)
    sys.exit(1)

VENDOR_ID   = 0xCAFE
CMD_BOOTSEL = 0xB0
VER_MAGIC   = 0x56455221   # "VER!" in little-endian

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
TIMEOUT_MOUNT = 15
TIMEOUT_FLASH = 30
TIMEOUT_HID   = 15


def signal_hmi_bootsel():
    """Find a running hmi.py process and send it SIGUSR2 to request
    a BOOTSEL reboot. Returns True if signal was sent."""
    try:
        result = subprocess.run(
            ['pgrep', '-f', r'python3.*hmi\.py'],
            capture_output=True, text=True)
        pids = result.stdout.strip().split()
        pids = [int(p) for p in pids if p]
    except Exception:
        pids = []

    if not pids:
        print("  No running hmi.py process found.")
        return False

    for pid in pids:
        try:
            os.kill(pid, signal.SIGUSR2)
            print(f"  Sent SIGUSR2 to hmi.py (PID {pid})")
            return True
        except ProcessLookupError:
            continue
        except PermissionError:
            print(f"  Permission denied signalling PID {pid}")
    return False


# ── Version extraction ────────────────────────────────────────────

def extract_uf2_version(uf2_path):
    """Scan a .uf2 file for the fw_version magic marker and return
    (major, minor, build, hash, dirty) or None."""
    magic_bytes = struct.pack('<I', VER_MAGIC)
    with open(uf2_path, 'rb') as f:
        data = f.read()
    idx = data.find(magic_bytes)
    if idx < 0:
        return None
    _, major, minor, build = struct.unpack('<IBBH', data[idx:idx+8])
    hash_str = data[idx+8:idx+16].split(b'\x00')[0].decode('ascii', errors='replace')
    dirty = data[idx+16]
    return (major, minor, build, hash_str, dirty)


def read_running_version(dev):
    """Query the firmware version via HID feature report.
    This is independent of the IN/OUT control protocol — works even
    when hmi.py is not running.
    Returns (major, minor, build, hash, dirty) or None."""
    try:
        # get_feature_report(report_id, max_length)
        buf = dev.get_feature_report(0, 64)
        if buf is None or len(buf) < 13:
            return None
        # Feature report layout: major(u8) minor(u8) build(u16) hash(8) dirty(u8)
        # hidapi prepends the report ID byte, so data starts at index 1
        off = 1 if len(buf) > 13 else 0
        major = buf[off]
        minor = buf[off + 1]
        build = struct.unpack('<H', bytes(buf[off+2:off+4]))[0]
        hash_str = bytes(buf[off+4:off+12]).split(b'\x00')[0].decode('ascii', errors='replace')
        dirty = buf[off + 12]
        return (major, minor, build, hash_str, dirty)
    except Exception:
        return None


def fmt_version(ver_tuple):
    """Format a version tuple for display."""
    if ver_tuple is None:
        return "unknown"
    if len(ver_tuple) == 3:
        return "v%d.%d.%d" % ver_tuple
    if len(ver_tuple) == 5:
        major, minor, build, hash_str, dirty = ver_tuple
        s = "v%d.%d.%d+%s" % (major, minor, build, hash_str)
        if dirty:
            s += "+dirty"
        return s
    return str(ver_tuple)


# ── Device discovery ──────────────────────────────────────────────

def find_hid_device():
    """Find the HMI's Generic HID interface."""
    for d in hid.enumerate(VENDOR_ID):
        if d.get('usage_page', 0) == 0xFF00:
            return d
    for d in hid.enumerate(VENDOR_ID):
        if d.get('interface_number', -1) == 0:
            return d
    return None


def find_bootsel_mount():
    """Find the RPI-RP2 / RP2350 mass storage mount point."""
    for pattern in MOUNT_GLOBS:
        matches = glob.glob(pattern)
        if matches:
            return matches[0]
    return None


def wait_for(description, check_fn, timeout):
    """Poll check_fn() every 0.5s until truthy or timeout."""
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


# ── Main ──────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Pico CNC HMI Firmware Updater")
    parser.add_argument('uf2', nargs='?', help="Path to .uf2 firmware file")
    parser.add_argument('--check', action='store_true',
                        help="Report versions without updating")
    parser.add_argument('--force', action='store_true',
                        help="Flash even if .uf2 is not newer")
    args = parser.parse_args()

    # Resolve .uf2 path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    default_uf2 = os.path.join(script_dir, "..", "firmware", "build", "pico_cnc_hmi.uf2")
    uf2_path = os.path.abspath(args.uf2 if args.uf2 else default_uf2)

    if not os.path.isfile(uf2_path):
        print(f"Error: firmware file not found: {uf2_path}", file=sys.stderr)
        sys.exit(1)

    uf2_size = os.path.getsize(uf2_path)
    uf2_ver = extract_uf2_version(uf2_path)
    print(f"UF2 file:    {os.path.basename(uf2_path)} ({uf2_size:,} bytes)")
    print(f"UF2 version: {fmt_version(uf2_ver)}")

    # Read running firmware version
    info = find_hid_device()
    running_ver = None
    if info:
        try:
            dev = hid.device()
            dev.open_path(info['path'])
            running_ver = read_running_version(dev)
            dev.close()
        except Exception as e:
            print(f"  Warning: could not read running version: {e}")
            print(f"  (Is hmi.py running? Stop it first, or check its log for the version.)")
    else:
        print("  HMI device not found (not plugged in or already in BOOTSEL)")

    print(f"Running:     {fmt_version(running_ver)}")

    # Compare versions
    if running_ver and uf2_ver:
        running_build = running_ver[2]
        uf2_build = uf2_ver[2]
        if uf2_build > running_build:
            print(f"\n→ Update available (build {running_build} → {uf2_build})")
        elif uf2_build == running_build:
            print(f"\n→ Same build ({running_build})")
        else:
            print(f"\n→ UF2 is older (build {uf2_build} < running {running_build})")

    if args.check:
        return

    # Decide whether to proceed
    if running_ver and uf2_ver and not args.force:
        if uf2_ver[2] <= running_ver[2]:
            print("Firmware is up to date. Use --force to flash anyway.")
            return

    # Check if device is already in BOOTSEL mode (mass storage mounted).
    mount = find_bootsel_mount()
    if mount:
        print(f"\n→ Device already in BOOTSEL mode ({mount})")
    else:
        if info is None:
            print("Error: HMI device not found and not in BOOTSEL mode.",
                  file=sys.stderr)
            sys.exit(1)

        # Step 1: Reboot to BOOTSEL
        print(f"\n[1/4] Sending BOOTSEL reboot command...")
        bootsel_sent = False
        try:
            dev = hid.device()
            dev.open_path(info['path'])
            pkt = bytearray(64)
            pkt[0] = 0xAA
            pkt[14] = CMD_BOOTSEL
            dev.write(bytes(pkt))
            dev.close()
            bootsel_sent = True
        except OSError:
            # Device locked by hmi.py — try signalling it via SIGUSR2.
            bootsel_sent = signal_hmi_bootsel()

        if not bootsel_sent:
            print("Error: could not send BOOTSEL command.", file=sys.stderr)
            print("  Stop hmi.py or ensure the device is accessible.", file=sys.stderr)
            sys.exit(1)
        print("  Device is rebooting...")

        # Step 2: Wait for mass storage
        print(f"\n[2/4] Waiting for USB mass storage...")
        mount = wait_for("RPI-RP2/RP2350 mount", find_bootsel_mount, TIMEOUT_MOUNT)
        if mount is None:
            print("Error: mass storage did not appear.", file=sys.stderr)
            sys.exit(1)

    # Step 3: Copy firmware
    print(f"\n[3/4] Copying firmware...")
    dest = os.path.join(mount, os.path.basename(uf2_path))
    print(f"  {uf2_path} → {dest}")
    try:
        shutil.copy2(uf2_path, dest)
    except PermissionError:
        print("  Permission denied — retrying with sudo...")
        subprocess.run(['sudo', 'cp', uf2_path, dest], check=True)
    os.sync()
    print("  Copy complete. Pico is flashing...")

    # Step 4: Wait for device to come back
    print(f"\n[4/4] Waiting for HMI to come back online...")
    wait_for("mass storage to unmount",
             lambda: find_bootsel_mount() is None,
             TIMEOUT_FLASH)
    result = wait_for("HID device", find_hid_device, TIMEOUT_HID)

    if result:
        # Read the new version
        try:
            dev = hid.device()
            dev.open_path(result['path'])
            new_ver = read_running_version(dev)
            dev.close()
            print(f"\n✓ Firmware update complete! Now running {fmt_version(new_ver)}")
        except Exception:
            print(f"\n✓ Firmware update complete!")
    else:
        print(f"\n⚠ Firmware was copied but device hasn't re-enumerated yet.")


if __name__ == '__main__':
    main()
