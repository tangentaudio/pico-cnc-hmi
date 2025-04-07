#!/usr/bin/env python3
import hid
import hal
import struct
import time

HAL = hal.component("hmi")

def hex_dump(data, length=16):
    if isinstance(data, str):
        data = data.encode('utf-8')
    
    for i in range(0, len(data), length):
        chunk = data[i:i + length]
        hex_string = ' '.join(f'{byte:02x}' for byte in chunk)
        print(f'{hex_string:<{length*3-1}} ')


HAL.newpin('knob.0.value', hal.HAL_U32, hal.HAL_OUT)
HAL.newpin('knob.1.value', hal.HAL_U32, hal.HAL_OUT)
HAL.newpin('knob.2.value', hal.HAL_U32, hal.HAL_OUT)
HAL.newpin('jog.axis', hal.HAL_U32, hal.HAL_OUT)
HAL.newpin('jog.step', hal.HAL_FLOAT, hal.HAL_OUT)
HAL.newpin('jog.inner.value', hal.HAL_S32, hal.HAL_OUT)
HAL.newpin('jog.outer.value', hal.HAL_FLOAT, hal.HAL_OUT)
HAL.newpin('jog.is-shuttling', hal.HAL_BIT, hal.HAL_OUT)

HAL.ready()

for devinfo in hid.enumerate(0xCAFE):
    vid = devinfo['vendor_id']
    pid = devinfo['product_id']
    print("Found device: VID=%04x PID=%04x" % (vid, pid))
    dev = hid.Device(vid, pid)

    if dev:
        print(f'Device manufacturer: {dev.manufacturer}')
        print(f'Product: {dev.product}')
        print(f'Serial Number: {dev.serial}')    

        while True:
            buf = dev.read(64, 10)
            if len(buf) == 64:
                pkt = struct.unpack('<BBBBIii', buf[0:16])
                hex_dump(buf, 64)
                HAL['knob.0.value'] = pkt[0]
                HAL['knob.1.value'] = pkt[1]
                HAL['knob.2.value'] = pkt[2]
                HAL['jog.axis'] = pkt[3]
                HAL['jog.step'] = float(pkt[4]) / 10000.0
                HAL['jog.inner.value'] = pkt[5]
                HAL['jog.outer.value'] = float(pkt[6]) / 10.0
                HAL['jog.is-shuttling'] = not (pkt[6] == 0)


