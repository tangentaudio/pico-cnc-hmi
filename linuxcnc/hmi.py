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



s.poll()

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
configured_maxvel_min = 0.0
configured_max_feed_override = 1.0
configured_max_rapid_override = 1.0
# HMI_MAXVEL_CURVE: power-law exponent for the maxvel knob.
# exponent > 1 concentrates resolution at the low end of the range:
#   segment 7/14 (mid-ring) maps to (0.5 ** exponent) of the usable span.
#   e.g. exponent=2 → mid-ring = 25% of span; exponent=3 → 12.5% of span.
# exponent=1 gives a linear mapping.
# Configure via [DISPLAY] HMI_MAXVEL_CURVE in the INI file.
#
# HMI_MAXVEL_MIN: minimum velocity assigned to segment 1, expressed in the same
# units as MAX_LINEAR_VELOCITY (e.g. inches/min or mm/min — NOT a fraction).
# Segment 0 is always 0 (machine stopped).
# Example: if MAX_LINEAR_VELOCITY=60 (inches/min) and you want segment 1 = 1 IPM,
#   set HMI_MAXVEL_MIN = 1.0
# Configure via [DISPLAY] HMI_MAXVEL_MIN in the INI file.
configured_maxvel_curve = 2.0
if not ini_filename:
    print("warning: unable to determine LinuxCNC INI file path, using defaults")
else:
    print(f"opening inifile={ini_filename}")
    try:
        inifile = linuxcnc.ini(ini_filename)
        maxvel_from_ini = inifile.find("TRAJ", "MAX_LINEAR_VELOCITY")
        configured_maxvel = float(maxvel_from_ini) if maxvel_from_ini else 1.0
        max_feed_from_ini = inifile.find("DISPLAY", "MAX_FEED_OVERRIDE")
        configured_max_feed_override = float(max_feed_from_ini) if max_feed_from_ini else 1.0
        max_rapid_from_ini = inifile.find("DISPLAY", "MAX_RAPID_OVERRIDE")
        configured_max_rapid_override = float(max_rapid_from_ini) if max_rapid_from_ini else 1.0
        maxvel_curve_from_ini = inifile.find("DISPLAY", "HMI_MAXVEL_CURVE")
        configured_maxvel_curve = float(maxvel_curve_from_ini) if maxvel_curve_from_ini else 2.0
        maxvel_min_from_ini = inifile.find("DISPLAY", "HMI_MAXVEL_MIN")
        configured_maxvel_min = float(maxvel_min_from_ini) if maxvel_min_from_ini else 0.0
        print(f"  MAX_LINEAR_VELOCITY={configured_maxvel}")
        print(f"  MAX_FEED_OVERRIDE={configured_max_feed_override}")
        print(f"  MAX_RAPID_OVERRIDE={configured_max_rapid_override}")
        print(f"  HMI_MAXVEL_CURVE={configured_maxvel_curve}")
        print(f"  HMI_MAXVEL_MIN={configured_maxvel_min}")
    except linuxcnc.error:
        print("warning: inifile.open() failed, using defaults")


def maxvel_to_seg(velocity: float) -> int:
    """Convert a max-velocity value (units/s) to a ring segment index 0-14.
    Segment 0 = 0 velocity.  Segments 1-14 span [configured_maxvel_min, configured_maxvel]
    with a power-law curve (configured_maxvel_curve) for low-end resolution."""
    if velocity <= 0.0:
        return 0
    span = configured_maxvel - configured_maxvel_min
    if span <= 0:
        return 14
    fraction = max(0.0, min(1.0, (velocity - configured_maxvel_min) / span))
    seg = int(round(14.0 * (fraction ** (1.0 / configured_maxvel_curve))))
    return max(1, seg)


def seg_to_maxvel(seg: int) -> float:
    """Convert a ring segment index 0-14 back to a max-velocity value (units/s).
    Segment 0 always returns 0.  Segments 1-14 map [min, max] with the curve."""
    if seg <= 0:
        return 0.0
    frac = (min(14, seg) / 14.0) ** configured_maxvel_curve
    return configured_maxvel_min + (configured_maxvel - configured_maxvel_min) * frac


# Map raw LinuxCNC interp_state values (1,2,4,8) to the sequential firmware enum
# (INTERP_OFF=0, INTERP_IDLE=1, INTERP_READING=2, INTERP_PAUSED=3, INTERP_WAITING=4).
# LinuxCNC uses power-of-2 values; the firmware uses 0-4 sequentially.
_INTERP_STATE_MAP = {
    linuxcnc.INTERP_IDLE:    1,   # firmware INTERP_IDLE
    linuxcnc.INTERP_READING: 2,   # firmware INTERP_READING
    linuxcnc.INTERP_PAUSED:  3,   # firmware INTERP_PAUSED
    linuxcnc.INTERP_WAITING: 4,   # firmware INTERP_WAITING
}

def map_interp_state(lc_state) -> int:
    return _INTERP_STATE_MAP.get(lc_state, 0)




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
    "task_paused" : -1,
    "inpos" : -1,
    "exec_state" : -1,
    "coolant" : -1,
    "optional_stop" : -1,
    "homed" : -1,
    "feedrate" : -1,
    "rapidrate" : -1,
    "maxvel" : -1,
    "interp_state" : -1,
    "interp_errcode" : -1
}

last_poll_time = 0

def in_step_mode() -> bool:
    # Derived purely from observable LinuxCNC state — works regardless of which
    # interface initiated single-step.
    # task_paused=1 appears in both step mode and normal AUTO_PAUSE.
    # The discriminator is exec_state: normal AUTO_PAUSE uniquely shows
    # EXEC_WAITING_FOR_MOTION_AND_IO (7); step mode shows EXEC_DONE (2) for
    # non-motion steps or EXEC_WAITING_FOR_IO (5) after motion steps.
    if not status['task_paused']:
        return False
    return status['exec_state'] != linuxcnc.EXEC_WAITING_FOR_MOTION_AND_IO

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
        print(f"mode={s.task_mode} (MANUAL={linuxcnc.MODE_MANUAL} AUTO={linuxcnc.MODE_AUTO} MDI={linuxcnc.MODE_MDI})")
        updated = True
    if status['paused'] != s.paused:
        status['paused'] = s.paused
        print(f"paused={s.paused} task_paused={s.task_paused} exec_state={s.exec_state} interp_state={s.interp_state}")
        updated = True
    if status['task_paused'] != s.task_paused:
        status['task_paused'] = s.task_paused
        updated = True
    if status['exec_state'] != s.exec_state:
        status['exec_state'] = s.exec_state
        updated = True
    if status['inpos'] != s.inpos:
        status['inpos'] = s.inpos
        updated = True
    if status['feedrate'] != s.feedrate:
        status['feedrate'] = s.feedrate
        updated = True
    if status['rapidrate'] != s.rapidrate:
        status['rapidrate'] = s.rapidrate
        updated = True
    if status['maxvel'] != s.max_velocity:
        status['maxvel'] = s.max_velocity
        print(f"status maxvel={s.max_velocity} seg={maxvel_to_seg(s.max_velocity)}")
        updated = True
    if status['interp_state'] != s.interp_state:
        status['interp_state'] = s.interp_state
        print(f"interp_state={s.interp_state} "
              f"(IDLE={linuxcnc.INTERP_IDLE} READING={linuxcnc.INTERP_READING} "
              f"PAUSED={linuxcnc.INTERP_PAUSED} WAITING={linuxcnc.INTERP_WAITING}) "
              f"task_paused={s.task_paused} paused={s.paused} "
              f"exec_state={s.exec_state} "
              f"(DONE={linuxcnc.EXEC_DONE} "
              f"MOTION={linuxcnc.EXEC_WAITING_FOR_MOTION} "
              f"MOTION_QUEUE={linuxcnc.EXEC_WAITING_FOR_MOTION_QUEUE} "
              f"IO={linuxcnc.EXEC_WAITING_FOR_IO} "
              f"MOTION_AND_IO={linuxcnc.EXEC_WAITING_FOR_MOTION_AND_IO} "
              f"DELAY={linuxcnc.EXEC_WAITING_FOR_DELAY} "
              f"SYSCMD={linuxcnc.EXEC_WAITING_FOR_SYSTEM_CMD}) "
              f"mapped={map_interp_state(s.interp_state)}")
        updated = True
    if status['interp_errcode'] != s.interpreter_errcode:
        status['interp_errcode'] = s.interpreter_errcode
        updated = True
    if status['coolant'] != int(bool(s.flood)):
        status['coolant'] = int(bool(s.flood))
        updated = True
    if status['optional_stop'] != int(bool(s.optional_stop)):
        status['optional_stop'] = int(bool(s.optional_stop))
        updated = True
    all_homed = int(s.joints > 0 and s.homed.count(1) == s.joints)
    if status['homed'] != all_homed:
        status['homed'] = all_homed
        print(f"homed={all_homed} (joints={s.joints} homed_tuple={s.homed})")
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

def hmi_safe_stop():
    """Zero all motion-producing HAL outputs immediately.

    Called on USB disconnect to prevent runaway motion.
    Safe choices by output type:
      jog.outer.value   - feeds halui.axis.jog-speed directly; must be zeroed
                          or the axis keeps jogging at the last commanded speed.
      jog.is-shuttling  - gates halui.axis/joint.*.plus via AND2; clearing it
                          stops continuous jog without touching the jog-speed net.
      jog.inner.value   - feeds jog-counts (delta-based); holding the last value
                          is safe (no delta = no motion). Do NOT reset to 0: on
                          reconnect the first real encoder value would cause a
                          jog of that full magnitude.
      jog.step/axis     - sticky configuration, not motion-producing; leave alone.
      knob.* overrides  - sticky LinuxCNC values; safe to hold. Re-synced on
                          reconnect by resetting the HAL pin tracking values.
    """
    HAL['jog.outer.value'] = 0.0
    HAL['jog.is-shuttling'] = False


def find_hid_device(vid=0xCAFE):
    """Return the first devinfo for the given VID, or None if not found."""
    devlist = hid.enumerate(vid)
    return devlist[0] if devlist else None


# Outer reconnect loop — tolerates the HMI going away and coming back.
# LinuxCNC is never disturbed: HAL pins hold their last value while disconnected,
# and no c.* calls are made until HMI input is received again after reconnect.
while True:
    devinfo = find_hid_device()
    if devinfo is None:
        time.sleep(0.5)
        continue

    try:
        dev = open_hid_device(devinfo)
    except Exception as e:
        print(f"HMI: failed to open device: {e}")
        time.sleep(1)
        continue

    print("HMI: device connected VID=%04x PID=%04x" % (devinfo['vendor_id'], devinfo['product_id']))
    print(f"  manufacturer: {hid_str(dev, 'manufacturer', 'get_manufacturer_string')}")
    print(f"  product:      {hid_str(dev, 'product', 'get_product_string')}")

    # Reset status to force a full OUT packet re-sync on reconnect so the HMI
    # LEDs come up immediately correct without waiting for the next state change.
    for key in list(status.keys()):
        status[key] = -1
    status['heartbeat'] = 0
    last_poll_time = 0

    # Seed knob tracking to the current LinuxCNC segment values.
    # This way the first IN packet only triggers c.feedrate/rapidrate/maxvel
    # if the encoder actually differs from the host value (e.g. the user turned
    # a knob while the HMI was powered off).  Using 0xFFFFFFFF here would cause
    # any first IN packet — including startup-state packets where the Pico
    # rebooted and all encoders are at 0 — to call c.feedrate(0) etc.
    s.poll()
    HAL['knob.0.value'] = int(round(s.feedrate * 14.0 / configured_max_feed_override))
    HAL['knob.1.value'] = int(round(s.rapidrate * 14.0 / configured_max_rapid_override))
    HAL['knob.2.value'] = maxvel_to_seg(s.max_velocity)

    prev_motion_cmd = 0
    try:
        while True:
            if poll_status():
                obuf = struct.pack('<BBBBBBBBBBBBBB',
                                  0xaa,
                                  status['heartbeat'] & 0xff,
                                  status['estop'],
                                  status['enabled'],
                                  status['mode'],
                                  map_interp_state(status['interp_state']),
                                  int(round(status['feedrate'] * 14.0 / configured_max_feed_override)),
                                  int(round(status['rapidrate'] * 14.0 / configured_max_rapid_override)),
                                  maxvel_to_seg(status['maxvel']),
                                  int(in_step_mode()),
                                  int(bool(status['inpos'])),
                                  int(bool(status['optional_stop'])),
                                  int(bool(status['coolant'])),
                                  int(bool(status['homed']))
                                  )
                obuf = pad_bytes(obuf, 64)
                dev.write(obuf)  # raises on disconnect — caught below

            buf = dev.read(64, 1)  # 1ms timeout; raises on disconnect

            # hid bindings vary: read() can return bytes/bytearray or list[int].
            if isinstance(buf, list):
                buf = bytes(buf)
            elif isinstance(buf, bytearray):
                buf = bytes(buf)

            if isinstance(buf, bytes) and len(buf) >= 17:
                pkt = struct.unpack('<BBBBIiiB', buf[0:17])
                if pkt[0] != HAL['knob.0.value']:
                    HAL['knob.0.value'] = pkt[0]
                    c.feedrate( float(pkt[0]) * configured_max_feed_override / 14.0)
                if pkt[1] != HAL['knob.1.value']:
                    HAL['knob.1.value'] = pkt[1]
                    c.rapidrate( float(pkt[1]) * configured_max_rapid_override / 14.0)
                if pkt[2] != HAL['knob.2.value']:
                    HAL['knob.2.value'] = pkt[2]
                    new_maxvel = seg_to_maxvel(pkt[2])
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
                cmd_coolant_edge = (motion_cmd & 0x20) and not (prev_motion_cmd & 0x20)
                cmd_optional_stop_edge = (motion_cmd & 0x10) and not (prev_motion_cmd & 0x10)
                prev_motion_cmd = motion_cmd

                if cmd_stop:
                    print(f"cmd: STOP (interp={s.interp_state} step={in_step_mode()})")
                    c.abort()
                elif not status['homed']:
                    if cmd_start or cmd_pause or cmd_step:
                        print(f"cmd: blocked — not homed")
                elif cmd_start:
                    print(f"cmd: CYCLE START (interp={s.interp_state} step={in_step_mode()} paused={s.paused})")
                    if in_step_mode():
                        # Single-step mode — advance from PAUSED or WAITING.
                        if status['interp_state'] in (linuxcnc.INTERP_PAUSED, linuxcnc.INTERP_WAITING):
                            c.auto(linuxcnc.AUTO_STEP)
                    elif status['interp_state'] == linuxcnc.INTERP_PAUSED:
                        # Paused mid-run (via AUTO_PAUSE) — resume continuous run.
                        c.auto(linuxcnc.AUTO_RESUME)
                    elif status['interp_state'] == linuxcnc.INTERP_IDLE:
                        # Idle → start program from line 0 (continuous run).
                        if status['mode'] != linuxcnc.MODE_AUTO:
                            c.mode(linuxcnc.MODE_AUTO)
                            c.wait_complete()
                        c.auto(linuxcnc.AUTO_RUN, 0)
                    # else: READING/WAITING in continuous run — ignore
                elif cmd_pause:
                    print(f"cmd: PAUSE (interp={s.interp_state} step={in_step_mode()} paused={s.paused})")
                    # Only pause during continuous run (not idle, not already paused, not step mode).
                    if (status['interp_state'] in (linuxcnc.INTERP_READING, linuxcnc.INTERP_WAITING)
                            and not status['paused']
                            and not in_step_mode()):
                        c.auto(linuxcnc.AUTO_PAUSE)
                elif cmd_step:
                    print(f"cmd: STEP (interp={s.interp_state} step={in_step_mode()} paused={s.paused})")
                    # Step button: enter single-step mode from IDLE only.
                    # Once in step mode, CYCLE START advances each step.
                    if status['interp_state'] == linuxcnc.INTERP_IDLE:
                        if status['mode'] != linuxcnc.MODE_AUTO:
                            c.mode(linuxcnc.MODE_AUTO)
                            c.wait_complete()
                        c.auto(linuxcnc.AUTO_STEP)

                if cmd_coolant_edge:
                    new_state = linuxcnc.FLOOD_ON if not status['coolant'] else linuxcnc.FLOOD_OFF
                    print(f"cmd: COOLANT TOGGLE (flood={status['coolant']} -> {new_state})")
                    c.flood(new_state)

                if cmd_optional_stop_edge:
                    new_state = not status['optional_stop']
                    print(f"cmd: OPTIONAL STOP TOGGLE (optional_stop={status['optional_stop']} -> {new_state})")
                    c.set_optional_stop(new_state)

    except Exception as e:
        print(f"HMI: device disconnected: {e}")
    finally:
        # Zero motion-producing outputs before closing so a disconnected HMI
        # cannot leave a continuous jog running.
        hmi_safe_stop()
        try:
            dev.close()
        except Exception:
            pass
        print("HMI: waiting for device to reconnect...")

    time.sleep(1)



            
            




