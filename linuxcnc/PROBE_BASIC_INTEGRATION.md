# Pico CNC HMI — Probe Basic Integration Guide

This guide describes how to add the Pico CNC HMI to an **existing Probe Basic machine
configuration**. It uses the template config at
`~/dev/probe_basic/configs/probe_basic/` as the reference starting point, which is what
most installations are based on.

> [!NOTE]
> All paths starting with `<your-config>/` refer to your machine's LinuxCNC config directory
> (e.g. `~/linuxcnc/configs/my_machine/`). Paths starting with `<hmi-repo>/` refer to the
> root of this repository (`pico-cnc-hmi/`).

---

## Step 1 — Install the Python dependency

The bridge script requires `python3-hid` (hidapi Python bindings):

```bash
sudo apt install python3-hid
# or, if running from a venv:
pip install hidapi
```

Verify:

```bash
python3 -c "import hid; print(hid.__version__)"
```

---

## Step 2 — USB permissions

The bridge runs as a normal user and needs read/write access to the HID device.
Create a udev rule so the device is accessible without `sudo`:

```bash
sudo tee /etc/udev/rules.d/99-pico-cnc-hmi.rules << 'EOF'
# Pico CNC HMI (TangentAudio, VID=0xCAFE PID=0x4008)
SUBSYSTEM=="hidraw", ATTRS{idVendor}=="cafe", ATTRS{idProduct}=="4008", \
    MODE="0660", GROUP="plugdev"
EOF
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Add your user to the `plugdev` group if not already a member:

```bash
sudo usermod -aG plugdev $USER   # log out and back in afterwards
```

---

## Step 3 — Copy files to your config directory

Copy the following two files from `<hmi-repo>/linuxcnc/` into your config directory:

```
<hmi-repo>/linuxcnc/hmi.py                  →  <your-config>/hmi.py
<hmi-repo>/linuxcnc/pb_sim/hmi_wiring.hal   →  <your-config>/hmi_wiring.hal
```

> [!IMPORTANT]
> Use `pb_sim/hmi_wiring.hal` (not `hmi_sim/hmi_wiring.hal`).  The Probe Basic
> version loads both `not` RT module instances in a single `loadrt` call to avoid a
> conflict with `probe_basic_postgui.hal` on POSIX non-realtime kernels.  On a real
> RT kernel this is usually fine either way, but the merged call is safe in both cases.

---

## Step 4 — Create `hmi_bridge.hal`

Create `<your-config>/hmi_bridge.hal` with this content:

```hal
# Pico CNC HMI bridge — loads the Python userspace component.
loadusr -Wn hmi python3 /home/YOUR_USER/linuxcnc/configs/my_machine/hmi.py
```

Use the **absolute path** to `hmi.py` in your config directory.

---

## Step 5 — INI changes

Open your machine INI file and make the following additions.

### `[DISPLAY]` section

Add these keys. Adjust values to match your machine:

```ini
# --- Pico CNC HMI settings ---

# Maximum rapid override for the rapid-override knob ring.
# Must match or be <= what your machine can actually do.
MAX_RAPID_OVERRIDE = 1.0        # 1.0 = 100%

# Power-law exponent for the maxvel knob.
# > 1 concentrates resolution at the low end.  3.0 is a good starting point.
# Segment 7/14 (mid-ring) = (0.5 ** exponent) of the usable range.
# exponent=1 gives a linear mapping.
HMI_MAXVEL_CURVE = 3.0

# Minimum velocity at segment 1 of the maxvel knob, in the same units as
# [TRAJ] MAX_LINEAR_VELOCITY (inches/min or mm/min — NOT a fraction).
# Segment 0 is always 0 (machine stopped).
HMI_MAXVEL_MIN = 1.0
```

> [!NOTE]
> `MAX_FEED_OVERRIDE` and `MAX_LINEAR_VELOCITY` are already in most Probe Basic INIs.
> The HMI reads them automatically — no change needed for those.

### `[HAL]` section

Add the two new HALFILE entries **before** `HALUI` and `POSTGUI_HALFILE`:

```ini
[HAL]
HALFILE = hallib/core_sim.hal           # your existing machine HAL (name varies)
# ... other existing HALFILE entries ...

# --- Add these two lines ---
HALFILE = hmi_bridge.hal                # starts the Python HMI bridge
HALFILE = hmi_wiring.hal                # wires HMI pins to motion/jog/halui

HALUI = halui
POSTGUI_HALFILE = probe_basic_postgui.hal
TWOPASS = on
```

> [!IMPORTANT]
> `hmi_bridge.hal` must come **before** `hmi_wiring.hal` since the wiring file
> references HAL pins that the bridge creates.  Both must come before `POSTGUI_HALFILE`.

---

## Step 6 — Modify `probe_basic_postgui.hal`

The stock postgui does `loadrt not` to create a `not` gate for the cycle timer.
`hmi_wiring.hal` also loads the `not` module (as `hmi-not`).  On POSIX non-realtime
kernels **you cannot `loadrt` the same module twice**, even with different instance names.

Make the following three-line change in your `probe_basic_postgui.hal`:

```diff
-loadrt not
+loadrt not names=pb-not

-addf not.0 servo-thread
+addf pb-not servo-thread

-net prog-running not.0.in <= halui.program.is-idle
-net cycle-timer time.0.start <= not.0.out
+net prog-running pb-not.in <= halui.program.is-idle
+net cycle-timer time.0.start <= pb-not.out
```

Everything else in the postgui file stays the same.

> [!NOTE]
> On a real realtime kernel (`RTAI`, `Xenomai`, or `PREEMPT_RT`) this conflict usually
> does not occur because kernel modules are shareable objects.  The rename is still
> recommended so your config works in simulation mode too.

---

## Step 7 — `halui.home-all` wiring

`hmi_wiring.hal` already includes:

```hal
net hmi-home-all hmi.home-all => halui.home-all
```

This is what makes the **STOP + CYCLE START chord** (see below) trigger Probe Basic's
REF ALL home sequence.  No additional action is needed here — it is already in the file
you copied in Step 3.

---

## Step 8 — Adapt `hmi_wiring.hal` for your axis count

The provided file is written for a **3-axis XYZ machine**.  If your machine has more
joints (e.g. XYZAC — 5 joints), extend the file as follows:

1. **`demux personality`** — change `personality=4` to `personality=<N+1>` where N is
   your total joint count (+1 for the "no axis" state at index 0).

2. **`and2` instances** — add one `hmi-and-shuttle-<axis>` and one `hmi-and-jog-<axis>`
   for each additional axis:
   ```hal
   loadrt and2 names=hmi-and-shuttle-x,...,hmi-and-shuttle-a,hmi-and-jog-x,...,hmi-and-jog-a
   ```

3. **`addf` lines** — add `addf` for each new `and2` instance.

4. **`net` lines** — for each additional axis wire:
   ```hal
   net sel-shuttle-axis-a hmi-and-shuttle-a.out => halui.joint.3.plus halui.axis.a.plus
   net sel-jog-axis-a     hmi-and-jog-a.out     => axis.a.jog-enable joint.3.jog-enable
   net hmi.jog-increment  => axis.a.jog-scale joint.3.jog-scale
   ```
   Also add `setp halui.joint.3.minus 0` (and `.axis.a.minus 0`) to prevent runaway jog.

5. **Demux output** — wire the new demux output to both shuttle and jog and2 inputs:
   ```hal
   net aa-in1 hmi-and-shuttle-a.in1 <= hmi-axis.demux.out-04 => hmi-and-jog-a.in1
   ```

---

## INI key reference

| Key | Section | Default | Description |
|---|---|---|---|
| `MAX_LINEAR_VELOCITY` | `[TRAJ]` | — | Upper end of maxvel knob range (required) |
| `MAX_FEED_OVERRIDE` | `[DISPLAY]` | — | Upper end of feed-override knob (required) |
| `MAX_RAPID_OVERRIDE` | `[DISPLAY]` | 1.0 | Upper end of rapid-override knob |
| `HMI_MAXVEL_CURVE` | `[DISPLAY]` | 2.0 | Power-law exponent (1 = linear, 3 = aggressive low-end) |
| `HMI_MAXVEL_MIN` | `[DISPLAY]` | 0.0 | Min velocity at segment 1 (same units as MAX_LINEAR_VELOCITY) |

---

## HAL pin reference

All pins are on the `hmi` component, created by `hmi.py`.

| Pin | Type | Dir | Description |
|---|---|---|---|
| `hmi.jog.axis` | U32 | OUT | Selected axis (0=none, 1=X, 2=Y, 3=Z, …) |
| `hmi.jog.step` | FLOAT | OUT | Jog increment (inches or mm) |
| `hmi.jog.inner.value` | S32 | OUT | Incremental jog encoder counts |
| `hmi.jog.outer.value` | FLOAT | OUT | Shuttle ring velocity (units/s; 0 when centered) |
| `hmi.jog.is-shuttling` | BIT | OUT | TRUE while shuttle ring is displaced |
| `hmi.home-all` | BIT | OUT | Pulsed TRUE for one cycle when STOP+START chord fires |
| `hmi.knob.0.value` | U32 | OUT | Feed-override ring segment (0–14) |
| `hmi.knob.1.value` | U32 | OUT | Rapid-override ring segment (0–14) |
| `hmi.knob.2.value` | U32 | OUT | Maxvel ring segment (0–14) |

---

## STOP + CYCLE START chord (Home All Axes)

When the machine is **powered, out of e-stop, and not yet homed**:

1. Press and hold **STOP**
2. While holding STOP, press and hold **CYCLE START**
3. Hold both for **2 seconds**

**Firmware feedback during countdown:** STOP LED fast-blinks (4 Hz).  Releasing either
button before 2 seconds cancels the chord.

**On fire:** both STOP and RUN LEDs flash full brightness; `hmi.home-all` is pulsed to
`halui.home-all`; `c.home(-1)` is also called directly.  The machine begins homing all
joints simultaneously.

**Guard:** the chord is silently ignored if the machine is in e-stop, not enabled, or
already homed.  Once homed, STOP and CYCLE START return to their normal functions.
