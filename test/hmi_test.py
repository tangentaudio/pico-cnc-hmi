#!/usr/bin/env python3
import hid
import hal
import struct
import time

HAL = hal.component("hmi_test")

def hex_dump(data, length=16):
    if isinstance(data, str):
        data = data.encode('utf-8')
    
    for i in range(0, len(data), length):
        chunk = data[i:i + length]
        hex_string = ' '.join(f'{byte:02x}' for byte in chunk)
        print(f'{hex_string:<{length*3-1}} ')

HAL.newpin('knob.0.value', hal.HAL_S32, hal.HAL_OUT)
HAL.newpin('knob.0.button', hal.HAL_BIT, hal.HAL_OUT)
HAL.newpin('knob.1.value', hal.HAL_S32, hal.HAL_OUT)
HAL.newpin('knob.1.button', hal.HAL_BIT, hal.HAL_OUT)

HAL.ready()

for devinfo in hid.enumerate(0xcafe):
    vid = devinfo['vendor_id']
    pid = devinfo['product_id']
    print("Found device: VID=%04x PID=%04x" % (vid, pid))
    dev = hid.Device(vid, pid)
    if dev:
        while True:
            str_out = b'\x00'
            buf = dev.read(64, 10)
            if len(buf) == 64:
                pkt = struct.unpack('<iiBB', buf[0:10])
                #print(buf)
                hex_dump(buf, 64)
                HAL['knob.0.value'] = pkt[0]
                HAL['knob.1.value'] = pkt[1]
                HAL['knob.0.button'] = not pkt[2]
                HAL['knob.1.button'] = not pkt[3]

