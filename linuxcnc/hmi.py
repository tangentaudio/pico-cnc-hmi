#!/usr/bin/env python3
import os
import hid
import hal
import linuxcnc
import struct
import time
import signal
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
shuttle_speeds_from_ini = None
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
        shuttle_speeds_from_ini = inifile.find("DISPLAY", "HMI_SHUTTLE_SPEEDS")
        print(f"  MAX_LINEAR_VELOCITY={configured_maxvel}")
        print(f"  MAX_FEED_OVERRIDE={configured_max_feed_override}")
        print(f"  MAX_RAPID_OVERRIDE={configured_max_rapid_override}")
        print(f"  HMI_MAXVEL_CURVE={configured_maxvel_curve}")
        print(f"  HMI_MAXVEL_MIN={configured_maxvel_min}")
    except linuxcnc.error:
        print("warning: inifile.open() failed, using defaults")

# Shuttle speed table: maps raw shuttle positions 0-7 to jog speeds in IPM.
# Index 0 = neutral (no motion).  Indices 1-7 = shuttle positions 1-7.
# halui.axis.jog-speed takes units/min (IPM for inch machines).
# Configure via [DISPLAY] HMI_SHUTTLE_SPEEDS in INI (7 comma-separated values in IPM).
# Machine MAX_LINEAR_VELOCITY clamps the upper values automatically.
configured_shuttle_speeds = [0.0, 0.5, 1, 2, 5, 10, 25, 60]
try:
    if shuttle_speeds_from_ini:
        ipm = [float(x.strip()) for x in shuttle_speeds_from_ini.split(',')]
        if len(ipm) == 7:
            configured_shuttle_speeds = [0.0] + ipm
            print(f"  HMI_SHUTTLE_SPEEDS={ipm} IPM")
        else:
            print(f"  HMI_SHUTTLE_SPEEDS: expected 7 values, got {len(ipm)}, using defaults")
except Exception:
    pass
print(f"  shuttle speeds: {configured_shuttle_speeds[1:]} IPM")


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


def feed_override_to_seg(override: float) -> int:
    """Convert a feedrate fraction (1.0 = 100%) to ring segment 0-14.
    Piecewise linear: segs 0-7 span 0-100%, segs 7-14 span 100%-max.
    Segment 7 is always the exact 100% anchor."""
    if override <= 1.0:
        return int(round(override * 7.0))
    over_range = configured_max_feed_override - 1.0
    if over_range <= 0:
        return 7
    return min(14, int(round(7.0 + (override - 1.0) * 7.0 / over_range)))


def feed_seg_to_override(seg: int) -> float:
    """Convert ring segment 0-14 to feedrate fraction (1.0 = 100%)."""
    if seg <= 7:
        return seg / 7.0
    over_range = configured_max_feed_override - 1.0
    if over_range <= 0:
        return 1.0
    return 1.0 + (seg - 7) * over_range / 7.0


def rapid_override_to_seg(override: float) -> int:
    """Convert a rapidrate fraction (1.0 = 100%) to ring segment 0-14.
    When max rapid <= 100%: linear, full ring spans 0-100%.
    When max rapid > 100%: piecewise, seg 7 anchors at 100%."""
    if configured_max_rapid_override <= 1.0:
        return min(14, int(round(override * 14.0)))
    if override <= 1.0:
        return int(round(override * 7.0))
    over_range = configured_max_rapid_override - 1.0
    return min(14, int(round(7.0 + (override - 1.0) * 7.0 / over_range)))


def rapid_seg_to_override(seg: int) -> float:
    """Convert ring segment 0-14 to rapidrate fraction (1.0 = 100%)."""
    if configured_max_rapid_override <= 1.0:
        return seg / 14.0
    if seg <= 7:
        return seg / 7.0
    over_range = configured_max_rapid_override - 1.0
    return 1.0 + (seg - 7) * over_range / 7.0


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
    "interp_errcode" : -1,
    "position" : (0.0, 0.0, 0.0)
}

last_poll_time = 0
last_pos_update_time = 0.0

# Track step mode explicitly.  The previous heuristic (inferring from
# exec_state) was unreliable: AUTO_PAUSE produces exec_state=3
# (EXEC_WAITING_FOR_MOTION), not 7 (EXEC_WAITING_FOR_MOTION_AND_IO)
# as assumed, causing false step-mode detection after a normal pause.
# Now we simply set/clear this flag based on our own command dispatch.
# On HMI reconnect this resets to False, which is the safe default:
# CYCLE START will resume continuously rather than single-step.
hmi_step_mode = False

def in_step_mode() -> bool:
    return hmi_step_mode

def poll_status():
    global status, last_poll_time, last_pos_update_time
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

    new_pos = tuple(s.position[:3])
    cur_pos = status['position'] if isinstance(status['position'], tuple) else (0.0, 0.0, 0.0)
    now_mono = time.monotonic()
    if (any(abs(new_pos[i] - cur_pos[i]) > 0.00005 for i in range(3))
            and (now_mono - last_pos_update_time) >= 0.03):  # max ~33 Hz
        status['position'] = new_pos
        last_pos_update_time = now_mono
        updated = True

    # Force OUT packet when step mode flag changes (it's our own state,
    # not derived from LinuxCNC, so it won't trigger via stat.poll()).
    if status.get('_prev_step_mode') != hmi_step_mode:
        status['_prev_step_mode'] = hmi_step_mode
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
# Pulsed TRUE for one cycle when the STOP+CYCLE-START chord fires.
# Wire to halui.home-all in the HAL config so both AXIS and Probe Basic
# GUIs respond to it without relying solely on c.home(-1).
HAL.newpin('home-all', hal.HAL_BIT, hal.HAL_OUT)

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
    """Return the devinfo for the vendor HID interface (usage_page=0xFF00).

    The Pico exposes two HID interfaces with the same VID/PID:
      - Interface 0: Generic HID (INOUT, usage_page=0xFF00, vendor-defined)
      - Interface 1: Keyboard    (IN-only, usage_page=0x0001)

    hid.enumerate() returns both.  Picking devlist[0] is non-deterministic
    (ordering varies between hidapi versions and VM USB passthrough sessions).
    We must filter explicitly to avoid opening the keyboard interface, which
    would silently drop all writes and return malformed reads.
    """
    devlist = hid.enumerate(vid)
    # Prefer the vendor-defined usage page (0xFF00) used by GENERIC_INOUT.
    for dev in devlist:
        if dev.get('usage_page', 0) == 0xFF00:
            return dev
    # Fallback: if usage_page metadata is unavailable (some hidapi builds omit
    # it), prefer interface_number 0 (Generic HID / INOUT) explicitly.
    for dev in devlist:
        if dev.get('interface_number', -1) == 0:
            print("HMI: usage_page metadata unavailable, selected interface_number=0")
            return dev
    if devlist:
        print("HMI: WARNING — no usage_page or interface_number match, using devlist[0]")
        return devlist[0]
    return None

# SIGUSR2 handler: the firmware update script sends this to ask us to
# kick the HMI into BOOTSEL mode on its behalf (since we hold the HID lock).
_bootsel_requested = False

def _handle_bootsel_signal(signum, frame):
    global _bootsel_requested
    _bootsel_requested = True
    print("HMI: SIGUSR2 received — BOOTSEL reboot requested")

signal.signal(signal.SIGUSR2, _handle_bootsel_signal)


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

    # Query firmware version via HID feature report (independent of control protocol).
    try:
        vbuf = dev.get_feature_report(0, 64)
        if vbuf and len(vbuf) >= 14:
            # hidapi prepends report ID byte; version data starts at index 1
            off = 1
            v_major = vbuf[off]
            v_minor = vbuf[off + 1]
            v_build = struct.unpack('<H', bytes(vbuf[off+2:off+4]))[0]
            v_hash = bytes(vbuf[off+4:off+12]).split(b'\x00')[0].decode('ascii', errors='replace')
            v_dirty = vbuf[off + 12]
            v_str = "v%d.%d.%d+%s" % (v_major, v_minor, v_build, v_hash)
            if v_dirty:
                v_str += "+dirty"
            print(f"  firmware:     {v_str}")
    except Exception as e:
        print(f"  firmware:     unknown ({e})")

    # Send one-time configuration packet (header 0xAB) so the HMI firmware can
    # display real units and compute the 100%-reset segment for each knob.
    # Fields use fixed-point encoding; see usb.h usb_cfg_pkt for full description.
    # MAX_LINEAR_VELOCITY and HMI_MAXVEL_MIN are in units/second internally;
    # multiply by 60 to convert to units/minute (IPM for inch machines) for display.
    cfg_pkt = struct.pack('<BHHHHH',
        0xAB,
        int(configured_max_feed_override  * 100),        # max_feed_pct
        int(configured_max_rapid_override * 100),        # max_rapid_pct
        int(configured_maxvel     * 60 * 10),            # max_vel_x10  (units/min * 10)
        int(configured_maxvel_min * 60 * 10),            # min_vel_x10  (units/min * 10)
        int(configured_maxvel_curve * 100),              # curve_x100
    )
    cfg_pkt = pad_bytes(cfg_pkt, 64)
    dev.write(cfg_pkt)
    print(f"HMI: config sent — feed_max={configured_max_feed_override*100:.0f}% "
          f"rapid_max={configured_max_rapid_override*100:.0f}% "
          f"maxvel={configured_maxvel*60:.1f} minvel={configured_maxvel_min*60:.1f} units/min "
          f"curve={configured_maxvel_curve}")

    # Reset status to force a full OUT packet re-sync on reconnect so the HMI
    # LEDs come up immediately correct without waiting for the next state change.
    for key in list(status.keys()):
        status[key] = -1
    status['heartbeat'] = 0
    status['position'] = (0.0, 0.0, 0.0)
    last_poll_time = 0

    # Reset step mode on reconnect — safe default: CYCLE START resumes
    # continuously rather than single-stepping.  See program_control_spec.md §3.
    hmi_step_mode = False

    # Wait briefly for LinuxCNC's override values to settle before seeding
    # the knob tracking.  During LinuxCNC startup the first poll() often returns
    # feedrate=0/rapidrate=0/maxvel=0 transiently before the real defaults are
    # applied.  Seeding from those zeros would make the HMI firmware believe the
    # overrides are 0, causing the first button press to "set" the knob to its
    # already-correct value rather than toggling it.
    # 150 ms is well within the USB HID connect timeout and comfortably covers
    # the LinuxCNC initialization settling window.
    time.sleep(0.15)
    s.poll()
    HAL['knob.0.value'] = feed_override_to_seg(s.feedrate)
    HAL['knob.1.value'] = rapid_override_to_seg(s.rapidrate)
    HAL['knob.2.value'] = maxvel_to_seg(s.max_velocity)
    print(f"HMI: knob seed — feed={s.feedrate:.3f}(seg={HAL['knob.0.value']}) "
          f"rapid={s.rapidrate:.3f}(seg={HAL['knob.1.value']}) "
          f"maxvel={s.max_velocity:.3f}(seg={HAL['knob.2.value']})")

    prev_motion_cmd = 0

    prev_jog_inner = 0
    prev_jog_outer = 0
    try:
        while True:
            if poll_status():
                pos = tuple(s.position[:3])  # always fresh — s.poll() called inside poll_status()
                obuf = struct.pack('<BBBBBBBBBBBBBBBBiii',
                                  0xaa,
                                  status['heartbeat'] & 0xff,
                                  status['estop'],
                                  status['enabled'],
                                  status['mode'],
                                  map_interp_state(status['interp_state']),
                                  feed_override_to_seg(status['feedrate']),
                                  rapid_override_to_seg(status['rapidrate']),
                                  maxvel_to_seg(status['maxvel']),
                                  int(in_step_mode()),          # step_mode (byte 9)
                                  # Byte 10: LED blink/solid signal for firmware.
                                  # In step mode, s.paused stays True throughout
                                  # and s.inpos stays False after mid-motion PAUSE;
                                  # use current_vel instead: vel==0 means at rest
                                  # (blink), vel>0 means executing (solid).
                                  # Outside step mode, s.paused works correctly.
                                  int(s.current_vel < 0.0001) if in_step_mode() else int(bool(status['paused'])),
                                  int(bool(status['coolant'])),
                                  int(bool(status['optional_stop'])),
                                  int(bool(status['homed'])),
                                  0, 0,  # cmd=0 (normal), _pad — offset 14-15
                                  int(round(pos[0] * 10000)),
                                  int(round(pos[1] * 10000)),
                                  int(round(pos[2] * 10000))
                                  )
                obuf = pad_bytes(obuf, 64)
                dev.write(obuf)  # raises on disconnect — caught below

            buf = dev.read(64, 1)  # 1ms timeout; raises on disconnect

            # Check if the firmware updater has requested a BOOTSEL reboot.
            if _bootsel_requested:
                _bootsel_requested = False
                print("HMI: sending BOOTSEL reboot command...")
                bootsel_pkt = bytearray(64)
                bootsel_pkt[0] = 0xAA
                bootsel_pkt[14] = 0xB0  # CMD_BOOTSEL
                dev.write(bytes(bootsel_pkt))
                time.sleep(0.1)
                break  # device will reset — fall through to reconnect loop

            # hid bindings vary: read() can return bytes/bytearray or list[int].
            if isinstance(buf, list):
                buf = bytes(buf)
            elif isinstance(buf, bytearray):
                buf = bytes(buf)

            if isinstance(buf, bytes) and len(buf) >= 17:
                pkt = struct.unpack('<BBBBIiiB', buf[0:17])
                if pkt[0] != HAL['knob.0.value']:
                    HAL['knob.0.value'] = pkt[0]
                    c.feedrate(feed_seg_to_override(pkt[0]))
                if pkt[1] != HAL['knob.1.value']:
                    HAL['knob.1.value'] = pkt[1]
                    c.rapidrate(rapid_seg_to_override(pkt[1]))
                if pkt[2] != HAL['knob.2.value']:
                    HAL['knob.2.value'] = pkt[2]
                    new_maxvel = seg_to_maxvel(pkt[2])
                    c.maxvel( new_maxvel )

                # Detect jog activity: inner wheel delta or shuttle engagement.
                jog_inner = pkt[5]
                jog_outer = pkt[6]
                jog_active = (jog_inner != prev_jog_inner) or (jog_outer != 0)

                # Auto-switch to MANUAL on jog activity when safe.
                # Only when interp is IDLE (no running program or MDI command)
                # and not already in MANUAL mode.  This covers the common case
                # of being stuck in MDI mode after Probe Basic MDI interaction.
                jog_ok = (status['mode'] == linuxcnc.MODE_MANUAL)
                if (jog_active
                        and not jog_ok
                        and status['interp_state'] == linuxcnc.INTERP_IDLE
                        and status['enabled']
                        and not status['estop']):
                    print(f"cmd: auto-switch to MANUAL for jog "
                          f"(was mode={status['mode']})")
                    c.mode(linuxcnc.MODE_MANUAL)
                    c.wait_complete()
                    jog_ok = True   # just switched — allow this cycle's jog

                # Always track the encoder position so no delta accumulates.
                prev_jog_inner = jog_inner
                prev_jog_outer = jog_outer

                # Axis and step are configuration — always safe to update.
                HAL['jog.axis'] = pkt[3]
                HAL['jog.step'] = float(pkt[4]) / 10000.0

                # Motion-producing jog outputs: only update when in MANUAL.
                # In AUTO/MDI with active interp, suppress to prevent
                # "jogging not allowed" errors from LinuxCNC.
                if jog_ok:
                    HAL['jog.inner.value'] = jog_inner
                    # Apply configurable shuttle speed lookup.
                    # Firmware sends raw -7..+7; we map abs to speed table.
                    shuttle_abs = min(abs(jog_outer), 7)
                    shuttle_sign = 1 if jog_outer >= 0 else -1
                    HAL['jog.outer.value'] = shuttle_sign * configured_shuttle_speeds[shuttle_abs]
                    HAL['jog.is-shuttling'] = not (jog_outer == 0)
                else:
                    # Ensure shuttle is off while not in MANUAL.
                    if HAL['jog.is-shuttling']:
                        HAL['jog.outer.value'] = 0.0
                        HAL['jog.is-shuttling'] = False

                # Auto-clear the home-all pulse from the previous cycle.
                if HAL['home-all']:
                    HAL['home-all'] = False

                motion_cmd = pkt[7]
                cmd_step  = (motion_cmd & 0x08) and not (prev_motion_cmd & 0x08)
                cmd_pause = (motion_cmd & 0x04) and not (prev_motion_cmd & 0x04)
                cmd_stop  = motion_cmd & 0x02    # STOP stays level: held = continuous abort
                cmd_start = (motion_cmd & 0x01) and not (prev_motion_cmd & 0x01)
                cmd_coolant_edge     = (motion_cmd & 0x20) and not (prev_motion_cmd & 0x20)
                cmd_optional_stop_edge = (motion_cmd & 0x10) and not (prev_motion_cmd & 0x10)
                cmd_home_all_edge    = (motion_cmd & 0x40) and not (prev_motion_cmd & 0x40)
                prev_motion_cmd = motion_cmd

                if cmd_home_all_edge:
                    if not status['homed']:
                        print(f"cmd: HOME ALL (joints={s.joints})")
                        # Pulse halui.home-all — works with both AXIS and Probe Basic.
                        # c.home(-1) is also issued as a direct fallback in case
                        # halui.home-all is not wired in the HAL config.
                        HAL['home-all'] = True
                        if status['mode'] != linuxcnc.MODE_MANUAL:
                            c.mode(linuxcnc.MODE_MANUAL)
                            c.wait_complete()
                        c.home(-1)
                    else:
                        print("cmd: HOME ALL ignored — already homed")

                # ── Program control: logical state dispatcher ──
                # Compute logical state once per cycle from LinuxCNC state
                # plus our hmi_step_mode flag.  See program_control_spec.md.
                interp = status['interp_state']
                if interp == linuxcnc.INTERP_IDLE:
                    logical_state = 'IDLE'
                elif hmi_step_mode:
                    logical_state = 'STEPPING'
                elif status['paused']:
                    logical_state = 'PAUSED'
                else:
                    logical_state = 'RUNNING'

                # STOP: level-based, always active (even before homed check)
                if cmd_stop:
                    print(f"cmd: STOP (state={logical_state})")
                    hmi_step_mode = False
                    c.abort()
                # All other program commands require homed
                elif not status['homed']:
                    if cmd_start or cmd_pause or cmd_step:
                        print("cmd: blocked — not homed")
                # CYCLE START
                elif cmd_start:
                    print(f"cmd: CYCLE START (state={logical_state} paused={status['paused']})")
                    if logical_state == 'IDLE':
                        if status['mode'] != linuxcnc.MODE_AUTO:
                            c.mode(linuxcnc.MODE_AUTO)
                            c.wait_complete()
                        c.auto(linuxcnc.AUTO_RUN, 0)
                    elif logical_state == 'PAUSED':
                        c.auto(linuxcnc.AUTO_RESUME)
                    elif logical_state == 'STEPPING':
                        if s.current_vel < 0.0001:  # step stacking guard: vel==0 means at rest
                            if status['mode'] != linuxcnc.MODE_AUTO:
                                c.mode(linuxcnc.MODE_AUTO)
                                c.wait_complete()
                            c.auto(linuxcnc.AUTO_STEP)
                        else:
                            print("cmd: CYCLE START ignored — step still executing (vel=%.4f)" % s.current_vel)
                    # RUNNING: no effect
                # PAUSE
                elif cmd_pause:
                    print(f"cmd: PAUSE (state={logical_state} paused={status['paused']})")
                    if logical_state == 'RUNNING':
                        c.auto(linuxcnc.AUTO_PAUSE)
                    elif logical_state == 'STEPPING':
                        # s.paused stays True throughout AUTO_STEP execution,
                        # so we can't gate on it.  AUTO_PAUSE is a safe no-op
                        # when already paused.
                        c.auto(linuxcnc.AUTO_PAUSE)
                    # PAUSED/IDLE: no effect
                # SINGLE STEP
                elif cmd_step:
                    print(f"cmd: STEP (state={logical_state} paused={status['paused']})")
                    if logical_state == 'STEPPING':
                        # Toggle off — program stays paused, CYCLE START resumes.
                        hmi_step_mode = False
                        print("cmd: exited step mode (paused, CYCLE START will resume)")
                    elif logical_state == 'IDLE':
                        hmi_step_mode = True
                        if status['mode'] != linuxcnc.MODE_AUTO:
                            c.mode(linuxcnc.MODE_AUTO)
                            c.wait_complete()
                        c.auto(linuxcnc.AUTO_STEP)
                    elif logical_state == 'PAUSED':
                        hmi_step_mode = True
                        print("cmd: entered step mode from paused state")
                    # RUNNING: no effect (must PAUSE first)

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
