#!/usr/bin/env python3
import hid
import hal
import linuxcnc
import struct
import time
import sys

HAL = hal.component("hmi")

try:
    s = linuxcnc.stat()
    c = linuxcnc.command()
except (linuxcnc.error, detail):
    print("error", detail)
    sys.exit(1)

s.poll()

inifile = linuxcnc.ini(s.ini_filename)
configured_maxvel = float(inifile.find("TRAJ", "MAX_LINEAR_VELOCITY")) or 1.0

#-- status
#halui.mode.is-auto
#halui.mode.is-joint
#halui.mode.is-manual
#halui.mode.is-mdi
#halui.mode.is-teleop

#halui.program.is-idle
#halui.program.is-paused
#halui.program.is-running
#halui.program.block-delete.is-on
#halui.program.optional-stop.is-on

#-- control
#halui.program.block-delete.on
#halui.program.block-delete.off
#halui.program.optional-stop.on
#halui.program.optional-stop.off
#halui.program.run
#halui.program.pause
#halui.program.resume
#halui.program.stop
#halui.program.step

#halui.mode.auto
#halui.mode.joint
#halui.mode.manual
#halui.mode.mdi
#halui.mode.teleop

def hex_dump(data, length=16):
    if isinstance(data, str):
        data = data.encode('utf-8')
    
    for i in range(0, len(data), length):
        chunk = data[i:i + length]
        hex_string = ' '.join(f'{byte:02x}' for byte in chunk)
        print(f'{hex_string:<{length*3-1}} ')

def pad_bytes(data: bytes, target_size: int, padding_byte: bytes = b'\xFF') -> bytes:
    padding_length = target_size - len(data)
    if padding_length > 0:
        data += padding_byte * padding_length
    return data

status = {
    "estop": -1,
    "enabled": -1,
    "mode" : -1,
    "feedrate" : -1,
    "rapidrate" : -1,
    "maxvel" : -1
}

def poll_status():
    global status
    updated = False
    
    s.poll()
    if status['estop'] != s.estop:
        status['estop'] = s.estop
        updated = True
    if status['enabled'] != s.enabled:
        status['enabled'] = s.enabled
        updated = True
    if status['mode'] != s.task_mode:
        status['mode'] = s.task_mode
        updated = True
    if status['feedrate'] != s.feedrate:
        status['feedrate'] = s.feedrate
        updated = True
    if status['rapidrate'] != s.rapidrate:
        status['rapidrate'] = s.rapidrate
        updated = True
    if status['maxvel'] != s.max_velocity:
        status['maxvel'] = s.max_velocity
        #print(f"maxvel={s.max_velocity} {s.max_velocity / configured_maxvel}")
        updated = True
    return updated


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
            if poll_status():
                obuf = struct.pack('<BBBBBBB',
                                  0xaa,
                                  status['estop'],
                                  status['enabled'],
                                  status['mode'],
                                  int(status['feedrate'] * 100.0),
                                  int(status['rapidrate'] * 100.0),
                                  int(status['maxvel'] * 100.0 / configured_maxvel)
                                  )
                
                obuf = pad_bytes(obuf, 64)
                
                print(f"writing length={len(obuf)}: {obuf}")
                try:
                    dev.write(obuf)
                except:
                    pass

            buf = dev.read(64, 1)
            if len(buf) == 64:
                pkt = struct.unpack('<BBBBIiiB', buf[0:17])
                hex_dump(buf, 64)
                if pkt[0] != HAL['knob.0.value']:
                    HAL['knob.0.value'] = pkt[0]
                    c.feedrate( float(pkt[0]) / 14.0)
                if pkt[1] != HAL['knob.1.value']:
                    HAL['knob.1.value'] = pkt[1]
                    c.rapidrate( float(pkt[1]) / 14.0)
                if pkt[2] != HAL['knob.2.value']:
                    HAL['knob.2.value'] = pkt[2]
                    c.maxvel( float(pkt[2]) * configured_maxvel/ 14.0 ) 

                HAL['jog.axis'] = pkt[3]
                HAL['jog.step'] = float(pkt[4]) / 10000.0
                HAL['jog.inner.value'] = pkt[5]
                HAL['jog.outer.value'] = float(pkt[6]) / 10.0
                HAL['jog.is-shuttling'] = not (pkt[6] == 0)

                motion_cmd = pkt[7]
                cmd_step = motion_cmd & 0x08
                cmd_pause = motion_cmd & 0x04
                cmd_stop = motion_cmd & 0x02
                cmd_start = motion_cmd & 0x01

                if cmd_stop:
                    c.abort()
                elif cmd_start:
                    c.auto(linuxcnc.AUTO_RUN, 1)
                elif cmd_pause:
                    c.auto(linuxcnc.AUTO_PAUSE)
                elif cmd_step:
                    c.auto(linuxcnc.AUTO_STEP)



            
            




