#!/usr/bin/env python3
import os
import hid
import hal
import linuxcnc
import struct
import time
import sys
from pprint import pprint

HAL = hal.component("hmi")

try:
    s = linuxcnc.stat()
    c = linuxcnc.command()
except linuxcnc.error as detail:
    print("error", detail)
    sys.exit(1)



def dump(obj):
  for attr in dir(obj):
    print("obj.%s = %r" % (attr, getattr(obj, attr)))


s.poll()
#dump(s)

def resolve_ini_filename(timeout_s: float = 3.0, poll_s: float = 0.05) -> str:
    # During early startup stat().ini_filename can be empty for a short time.
    # Prefer the stat value once available, otherwise fall back to INI_FILE env.
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        s.poll()
        if s.ini_filename:
            return s.ini_filename
        env_ini = (
            os.environ.get("INI_FILE", "")
            or os.environ.get("INI_FILE_NAME", "")
            or os.environ.get("LINUXCNC_INI", "")
        )
        if env_ini:
            return env_ini
        time.sleep(poll_s)
    return (
        os.environ.get("INI_FILE", "")
        or os.environ.get("INI_FILE_NAME", "")
        or os.environ.get("LINUXCNC_INI", "")
    )

ini_filename = resolve_ini_filename()
configured_maxvel = 1.0
if not ini_filename:
    print("warning: unable to determine LinuxCNC INI file path, using MAX_LINEAR_VELOCITY=1.0")
else:
    print(f"opening inifile={ini_filename}")
    try:
        inifile = linuxcnc.ini(ini_filename)
        maxvel_from_ini = inifile.find("TRAJ", "MAX_LINEAR_VELOCITY")
        configured_maxvel = float(maxvel_from_ini) if maxvel_from_ini else 1.0
    except linuxcnc.error:
        print("warning: inifile.open() failed, using MAX_LINEAR_VELOCITY=1.0")




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
    "heartbeat" : 0,
    "estop": -1,
    "enabled": -1,
    "mode" : -1,
    "paused" : True,
    "feedrate" : -1,
    "rapidrate" : -1,
    "maxvel" : -1,
    "interp_state" : -1,
    "interp_errcode" : -1
}

last_poll_time = 0

def poll_status():
    global status, last_poll_time
    updated = False
    
    now = int(round(time.time() * 1000))

    if now - last_poll_time > 500:
        status['heartbeat'] = status['heartbeat'] + 1
        last_poll_time = now
        updated = True

    s.poll()
    if status['estop'] != s.estop:
        status['estop'] = s.estop
        updated = True
    if status['enabled'] != s.enabled:
        status['enabled'] = s.enabled
        updated = True
    if status['mode'] != s.task_mode:
        status['mode'] = s.task_mode
        print(f"mode={s.task_mode}")
        updated = True
    if status['paused'] != s.paused:
        status['paused'] = s.paused
        updated = True
    if status['feedrate'] != s.feedrate:
        status['feedrate'] = s.feedrate
        updated = True
    if status['rapidrate'] != s.rapidrate:
        status['rapidrate'] = s.rapidrate
        updated = True
    if status['maxvel'] != s.max_velocity:
        status['maxvel'] = s.max_velocity
        print(f"status maxvel={s.max_velocity} {s.max_velocity / configured_maxvel}")
        updated = True
    if status['interp_state'] != s.interp_state:
        status['interp_state'] = s.interp_state
        updated = True
    if status['interp_errcode'] != s.interpreter_errcode:
        status['interp_errcode'] = s.interpreter_errcode
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

def open_hid_device(devinfo):
    vid = devinfo['vendor_id']
    pid = devinfo['product_id']

    # hidapi python bindings exist in two common forms:
    # 1) hid.Device(...)
    # 2) hid.device().open(...)
    if hasattr(hid, 'Device'):
        return hid.Device(vid, pid)

    if hasattr(hid, 'device'):
        dev = hid.device()
        path = devinfo.get('path')
        if path:
            dev.open_path(path)
        else:
            dev.open(vid, pid)
        return dev

    raise RuntimeError('Unsupported hid module: missing Device/device API')

def hid_str(dev, attr_name, fallback_method):
    val = getattr(dev, attr_name, None)
    if isinstance(val, str) and val:
        return val
    if hasattr(dev, fallback_method):
        try:
            return getattr(dev, fallback_method)()
        except Exception:
            return ""
    return ""

for devinfo in hid.enumerate(0xCAFE):
    vid = devinfo['vendor_id']
    pid = devinfo['product_id']
    print("Found device: VID=%04x PID=%04x" % (vid, pid))
    dev = open_hid_device(devinfo)

    if dev:
        print(f"Device manufacturer: {hid_str(dev, 'manufacturer', 'get_manufacturer_string')}")
        print(f"Product: {hid_str(dev, 'product', 'get_product_string')}")
        print(f"Serial Number: {hid_str(dev, 'serial', 'get_serial_number_string')}")

        while True:
            if poll_status():
                obuf = struct.pack('<BBBBBBBBB',
                                  0xaa,
                                  status['heartbeat'] & 0xff,
                                  status['estop'],
                                  status['enabled'],
                                  status['mode'],
                                  status['interp_state'],
                                  int(round(status['feedrate'] * 14.0)),
                                  int(round(status['rapidrate'] * 14.0)),
                                  int(round(status['maxvel'] * 14.0 / configured_maxvel))
                                  )
                
                obuf = pad_bytes(obuf, 64)
                try:
                    dev.write(obuf)
                except:
                    pass

            buf = dev.read(64, 1)

            # hid bindings vary: read() can return bytes/bytearray or list[int].
            if isinstance(buf, list):
                buf = bytes(buf)
            elif isinstance(buf, bytearray):
                buf = bytes(buf)

            if isinstance(buf, bytes) and len(buf) >= 17:
                pkt = struct.unpack('<BBBBIiiB', buf[0:17])
                if len(buf) == 64:
                    hex_dump(buf, 64)
                if pkt[0] != HAL['knob.0.value']:
                    HAL['knob.0.value'] = pkt[0]
                    c.feedrate( float(pkt[0]) / 14.0)
                if pkt[1] != HAL['knob.1.value']:
                    HAL['knob.1.value'] = pkt[1]
                    c.rapidrate( float(pkt[1]) / 14.0)
                if pkt[2] != HAL['knob.2.value']:
                    HAL['knob.2.value'] = pkt[2]
                    new_maxvel = float(pkt[2]) * configured_maxvel / 14.0
                    c.maxvel( new_maxvel ) 

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
                    if status['paused']:
                        c.auto(linuxcnc.AUTO_RESUME)
                    elif status['interp_state'] == 1: 
                        # INTERP_IDLE
                        c.auto(linuxcnc.AUTO_RUN, 1)
                elif cmd_pause:
                    if status['paused']:
                        c.auto(linuxcnc.AUTO_RESUME)
                    else:
                        c.auto(linuxcnc.AUTO_PAUSE)
                elif cmd_step:
                    c.auto(linuxcnc.AUTO_STEP)



            
            




