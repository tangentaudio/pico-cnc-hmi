#!/usr/bin/env python3
import hid
import struct
import time


def hex_dump(data, length=16):
    if isinstance(data, str):
        data = data.encode('utf-8')
    
    for i in range(0, len(data), length):
        chunk = data[i:i + length]
        hex_string = ' '.join(f'{byte:02x}' for byte in chunk)
        print(f'{hex_string:<{length*3-1}} ')

for devinfo in hid.enumerate(0xcafe):
    vid = devinfo['vendor_id']
    pid = devinfo['product_id']
    print("Found device: VID=%04x PID=%04x" % (vid, pid))
    dev = hid.Device(vid, pid)
    if dev:
        while True:
            str_out = b'\x00'
            buf = dev.read(64, 10)
            hex_dump(buf, 64)


