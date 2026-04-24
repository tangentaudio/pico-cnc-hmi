# Pico CNC HMI — Program Control Specification

## 1. Overview

This document specifies the desired behavior of the four program control buttons
on the HMI: **STOP**, **PAUSE**, **CYCLE START**, and **SINGLE STEP**. It also
covers their interaction with LinuxCNC's mode and interpreter state.

> [!IMPORTANT]
> **Design Principle:** The HMI must never initiate unexpected machine motion.
> Only CYCLE START may start or resume motion. All other transitions leave the
> machine stationary.

---

## 2. LinuxCNC State Primer

### Modes (task_mode)
| Value | Name | Notes |
|---|---|---|
| 1 | MANUAL | Jog, home |
| 2 | AUTO | Program execution, step, pause, resume |
| 3 | MDI | Manual Data Input (Probe Basic uses this frequently) |

### Task State (s.state / RCS status)
| Constant | Meaning |
|---|---|
| `RCS_DONE` | Task is idle (not executing anything) |
| `RCS_EXEC` | Task is executing (program running, stepping, or paused mid-program) |
| `RCS_ERROR` | Task encountered an error |

> [!IMPORTANT]
> `s.state == RCS_EXEC` remains true even when a program is **paused**.
> This is how QtPyVCP detects "paused mid-program" vs "truly idle":
> `RCS_EXEC and s.paused`.

### Interpreter States (interp_state)
| Value | Name | Meaning |
|---|---|---|
| 1 | IDLE | No program activity |
| 2 | READING | Interpreter is reading/executing G-code lines |
| 3 | PAUSED | Program paused (by AUTO_PAUSE or between steps) |
| 4 | WAITING | Waiting for motion/IO to complete |

### Pause-Related Fields
| Field | Type | Meaning |
|---|---|---|
| `s.paused` | bool | Motion trajectory paused (`motion.traj.paused`) |
| `s.task_paused` | int | Task-level paused flag (`task.task_paused`) |

> [!WARNING]
> `interp_state == INTERP_PAUSED` is NOT the only way LinuxCNC can be "paused."
> After an `AUTO_STEP`, the interpreter may settle in `INTERP_READING` or
> `INTERP_WAITING` with `task_paused=1` and `paused=True`. Our code must use
> `s.paused` (or `s.state == RCS_EXEC and s.paused`) as the canonical "is it
> paused?" check, NOT `interp_state == INTERP_PAUSED`.

### Key LinuxCNC Commands
| Command | Effect |
|---|---|
| `c.auto(AUTO_RUN, 0)` | Start program from line 0 (continuous) |
| `c.auto(AUTO_STEP)` | Execute one block then pause |
| `c.auto(AUTO_PAUSE)` | Pause a running program (immediate deceleration) |
| `c.auto(AUTO_RESUME)` | Resume from pause (continuous) |
| `c.abort()` | Abort program execution |
| `c.mode(MODE_AUTO)` | Switch to AUTO mode (required before program commands) |

---

## 3. HMI State Model

### The Step-Mode Question

LinuxCNC does **not** expose a "step mode" flag. From LinuxCNC's perspective,
`AUTO_STEP` is a one-shot command: "execute one block, then pause." There is no
persistent "step mode" state in LinuxCNC.

**QtPyVCP's approach (Probe Basic's GUI):** Each button press is a simple,
stateless command:
- **Step button** → `CMD.auto(AUTO_STEP)` (no flag, no toggle)
- **Run button** → if `paused`: `AUTO_RESUME`, else: `AUTO_RUN`
- **Pause button** → `AUTO_PAUSE`

Probe Basic does NOT track a step-mode flag. Each step press just advances one
block. The "run" button always resumes continuously. This is simple and robust.

**Our HMI constraint:** We have a dedicated STEP button that we want to act as
a modal toggle (enter/exit step mode). This is a UX choice that goes beyond what
LinuxCNC natively models. We need `hmi_step_mode` because:
1. We want CYCLE START to advance one block when "in step mode" vs resume
   continuously when "not in step mode"
2. We want distinct LED states for step mode vs paused

**Accepted trade-off:** `hmi_step_mode` is HMI-local state that can desync from
LinuxCNC (e.g., on USB reconnect, or if Probe Basic's GUI is used concurrently).
It resets to `False` on reconnect (safe default: resume continuously).

### Logical State Derivation

Rather than mixing raw LinuxCNC fields throughout the command handlers, compute
a single logical state once per cycle:

```python
def compute_logical_state():
    if s.interp_state == INTERP_IDLE:
        return 'IDLE'
    elif hmi_step_mode:
        return 'STEPPING'
    elif s.paused:       # covers interp PAUSED, READING+paused, WAITING+paused
        return 'PAUSED'
    else:
        return 'RUNNING'
```

This collapses the combinatorial explosion into 4 states that map cleanly to
button actions and LED states.

### State Transition Diagram

```
                       STEP from IDLE
                       (sets step_mode=true,
                        issues AUTO_STEP →
┌──────────────┐        executes 1st block)
│     IDLE     │─────────────────────────────┐
│  (STOP lit)  │                             │
└──────┬───────┘                             ▼
       │                              ┌──────────────┐
       │ CYCLE START                  │  STEPPING    │◄──┐
       │ (AUTO_RUN)                   │(STEP+START   │   │
       ▼                              │  blink)      │   │
┌──────────────┐                      └──┬───┬───────┘   │
│   RUNNING    │                        │   │            │
│  (START lit) │          CYCLE START   │   │            │
└──┬───────────┘          (AUTO_STEP) ──┘   │            │
   │                                        │            │
   │ PAUSE                    STEP          │            │
   │ (AUTO_PAUSE)             (toggle off → │            │
   ▼                           step_mode=   │            │
┌──────────────┐               false)       │            │
│    PAUSED    │◄───────────────────────────┘            │
│ (PAUSE blink)│                                         │
└──┬───────────┘                                         │
   │                                                     │
   │ STEP from PAUSED                                    │
   │ (sets step_mode=true, NO command issued,            │
   │  awaits CYCLE START to advance)                     │
   └─────────────────────────────────────────────────────┘

   STOP from any state → IDLE  (c.abort, step_mode=false)
```

**Key distinction:**
- **STEP from IDLE** → enters step mode AND auto-starts first block (`AUTO_STEP`)
- **STEP from PAUSED** → enters step mode only (no motion until CYCLE START)

---

## 4. Button Behavior Specification

### 4.1 STOP

| Aspect | Specification |
|---|---|
| **Detection** | Level-based (held = continuous abort) — safety |
| **Pre-condition** | Always active (even during e-stop for re-abort) |
| **Action** | `c.abort()`, clear `hmi_step_mode` |
| **Post-state** | IDLE |
| **Notes** | Most authoritative button. Overrides all other states. |

### 4.2 CYCLE START

| Aspect | Specification |
|---|---|
| **Detection** | Rising-edge only |
| **Pre-condition** | Machine homed |

**Behavior by logical state:**

| Logical State | Action | Result |
|---|---|---|
| IDLE | Ensure MODE_AUTO, `AUTO_RUN(0)` | Start program continuously |
| RUNNING | No effect | Already running |
| PAUSED | `AUTO_RESUME` | Resume continuous run |
| STEPPING | Guard: only if `s.current_vel == 0`; ensure MODE_AUTO, `AUTO_STEP` | Advance one block |

> [!WARNING]
> **Step stacking guard:** In STEPPING state, CYCLE START is ignored when
> `s.current_vel > 0` (machine is still executing the previous block). This
> prevents hammering the button from queueing multiple steps.
> 
> **Note:** `s.paused` cannot be used here — it stays `True` throughout step
> mode execution. `s.inpos` cannot be used either — it stays `False` after
> mid-motion PAUSE. `s.current_vel` reliably indicates actual machine motion.

> [!IMPORTANT]
> CYCLE START is the **only** button that initiates machine motion.
> In PAUSED state, it **always** resumes continuously — the user must press
> STEP first to enter step mode if they want single-stepping.

### 4.3 PAUSE

| Aspect | Specification |
|---|---|
| **Detection** | Rising-edge only |
| **Pre-condition** | Machine homed |

| Logical State | Action | Result |
|---|---|---|
| RUNNING | `AUTO_PAUSE` | Pause program |
| PAUSED | No effect | Already paused |
| STEPPING | `AUTO_PAUSE` if `not s.paused` | Halt mid-block immediately |
| IDLE | No effect | Nothing to pause |

> [!IMPORTANT]
> PAUSE must **always** work as an immediate motion halt. Operators learn this
> as a safety reflex — it must not silently fail in step mode or any other mode.

### 4.4 SINGLE STEP

| Aspect | Specification |
|---|---|
| **Detection** | Rising-edge only |
| **Pre-condition** | Machine homed |

| Logical State | Action | Result |
|---|---|---|
| IDLE | Set step_mode=true, ensure MODE_AUTO, `AUTO_STEP` | Enter step mode from idle |
| PAUSED | Set step_mode=true | Enter step mode (next CYCLE START steps) |
| RUNNING | No effect | Must PAUSE first |
| STEPPING | Set step_mode=false | Exit step mode → PAUSED |

> [!NOTE]
> When exiting step mode (toggling off), the program remains paused. The user
> must press CYCLE START to resume continuous execution. This prevents accidental
> motion on a toggle.

### 4.5 HOME ALL CHORD (STOP + CYCLE START)

| Aspect | Specification |
|---|---|
| **Detection** | Firmware-level: both STOP and CYCLE START held simultaneously for 2 seconds |
| **Pre-condition** | Machine enabled, out of e-stop, NOT homed |
| **Action** | `hmi.home-all` HAL pin pulsed, `c.home(-1)` called directly |
| **Post-state** | Homing sequence begins on all joints |

**Firmware feedback during countdown:** STOP LED fast-blinks (4 Hz). Releasing
either button before 2 seconds cancels the chord.

**On fire:** Both STOP and RUN LEDs flash full brightness.

**Guard:** The chord is silently ignored if the machine is in e-stop, not enabled,
or already homed. Once homed, STOP and CYCLE START return to their normal
functions as defined above.

> [!NOTE]
> This is implemented entirely in the firmware (chord detection + LED countdown).
> hmi.py reads the `home_all` bit from the HID IN packet and issues the LinuxCNC
> commands. The chord uses bits 0x40 in `motion_cmd`.

---

## 5. LED Specification

### LED Assignments
| LED Index | Label | Button |
|---|---|---|
| 8 | STEP | Single Step |
| 9 | PAUSE | Pause |
| 10 | STOP | Stop |
| 11 | RUN | Cycle Start |

### LED States by Logical State

| Logical State | STEP (8) | PAUSE (9) | STOP (10) | RUN (11) |
|---|---|---|---|---|
| **Not homed** | Off | Off | Slow blink | Off |
| **IDLE** | Off | Off | Solid | Off |
| **RUNNING** | Off | Off | Off | Solid |
| **PAUSED** | Off | Blink | Off | Off |
| **STEPPING (waiting)** | Blink | Off | Off | Blink |
| **STEPPING (executing)** | Solid | Off | Off | Solid |
| **E-stop / Disabled** | Off | Off | Off | Off |

> [!NOTE]
> "Stepping (waiting)" vs "Stepping (executing)" is driven by
> **`s.current_vel`**: when velocity is zero the machine is at rest (blink),
> when velocity is non-zero a block is executing (solid).
>
> **Rejected alternatives (tested 2026-04-24):**
> - `s.paused`: stays `True` throughout step mode execution — completely
>   unusable for LED distinction.
> - `s.inpos`: works for normal step cycles but stays `False` after a
>   mid-motion PAUSE, causing incorrect "executing" indication when stopped.

---

## 6. Mode Switching

### Auto-Switch to AUTO
Program control commands (CYCLE START, STEP from IDLE) require `MODE_AUTO`.
The HMI transparently switches to AUTO before issuing the command.

### Auto-Switch to MANUAL
When the jog wheel moves and `interp_state == IDLE`, the HMI auto-switches
to `MODE_MANUAL`. This covers the common case of being stuck in MDI/AUTO
after a program ends.

### Jog Controls Gating
- **Jog LEDs** (axis/increment): visible in MANUAL, MDI, or AUTO+IDLE
- **Jog motion**: only sent to HAL when in MANUAL mode
- **Jog config** (axis/increment selection): always forwarded (config, not motion)

---

## 7. Edge Detection Rules

| Button | Detection | Rationale |
|---|---|---|
| STOP | Level (held) | Safety — continuous abort while held |
| CYCLE START | Rising edge | Prevent repeated run/step on hold |
| PAUSE | Rising edge | Prevent repeated pause commands |
| STEP | Rising edge | Prevent repeated toggle |
| COOLANT | Rising edge | Toggle |
| OPT STOP | Rising edge | Toggle |
| HOME CHORD | Rising edge (2s hold) | Special: chord timing in firmware |

---

## 8. Analysis of Current Implementation vs This Spec

### 8.1 Bug: Step→Resume Broken (CYCLE START dead after exiting step mode)

**Symptom:** After exiting step mode (STEP toggle off), CYCLE START has no effect.

**Root cause:** When `hmi_step_mode` is toggled off, LinuxCNC is "paused" but
`interp_state` may be `INTERP_READING` (2) — not `INTERP_PAUSED` (3).
The CYCLE START resume path only checks `interp_state == INTERP_PAUSED`,
missing the READING+paused case entirely.

**Fix per this spec:** The `compute_logical_state()` function uses `s.paused`
(not `interp_state == INTERP_PAUSED`) to detect PAUSED. The CYCLE START handler
for PAUSED state issues `AUTO_RESUME` regardless of `interp_state`. This matches
how QtPyVCP does it:
```python
# QtPyVCP program_actions.py line 129:
elif STAT.state == linuxcnc.RCS_EXEC and (STAT.paused or interp_paused):
    CMD.auto(linuxcnc.AUTO_RESUME)
```

### 8.2 Architectural Comparison with QtPyVCP

QtPyVCP's approach is simpler because it has **separate buttons** for Step and
Run. Each button is a stateless action: Step always steps, Run always
runs/resumes. No toggle state needed.

Our HMI has a single CYCLE START button that must do different things depending
on context (step vs resume). This forces us to maintain `hmi_step_mode`. The
trade-off is acceptable — we just need to keep the dispatch clean.

### 8.3 Key Lessons from QtPyVCP

1. **Use `s.paused` not `interp_state`** for pause detection
2. **Use `s.state == RCS_EXEC`** to know if a program is active
3. **`AUTO_STEP` works from any paused state** — no special interp_state checks
4. **`AUTO_RESUME` works from any paused state** — same
5. **Step is stateless in LinuxCNC** — our modal toggle is an HMI-layer concept

---

## 9. Resolved Design Decisions

**Q1: PAUSE during step mode?**
**Decision: Allowed.** PAUSE must always work as an immediate motion halt —
operators learn this as a safety reflex and it must not silently fail in any
mode. LinuxCNC's `emcTrajPause()` immediately decelerates mid-block.
`AUTO_PAUSE` is issued unconditionally in STEPPING state — it is a safe
no-op when the machine is already at rest. After pausing mid-step, the LEDs
blink (vel=0). The next CYCLE START issues `AUTO_STEP` which continues from
where it stopped — LinuxCNC handles this natively.

**Q2: STEP during RUNNING (continuous)?**
**Decision: No effect (must PAUSE first).** Keeps the interaction model simple.

**Q3: Step-mode LED and guard signal?**
**Decision: Resolved — use `s.current_vel`.** Tested 2026-04-24:
- `s.paused` stays `True` throughout step mode execution (completely unusable).
- `s.inpos` works for normal step cycles but stays `False` after mid-motion
  PAUSE, preventing further stepping and showing incorrect LED state.
- `s.current_vel` reliably transitions between 0 (at rest) and >0 (executing)
  in all tested scenarios: step-from-idle, pause→step, pause-mid-step→resume.
  Used for both the LED blink/solid distinction and the step stacking guard.
