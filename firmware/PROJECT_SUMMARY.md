# Pico CNC HMI Firmware - Project Summary

Date: 2026-04-20 (updated same day after active debug session)
Scope: Source-based review of firmware plus LinuxCNC host bridge script.

Current maturity (confirmed by user):
- Early bring-up / debug / proof-of-concept state.
- Project has been dormant for about a year and this document is intended as a memory-refresh baseline.
- Current implementation details are provisional and may change substantially as requirements and hardware usage are refined.

## 1. What This Project Appears To Do

This firmware targets Raspberry Pi Pico 2 and implements a CNC human-machine interface (HMI).

Primary functions observed in code:
- Read hardware controls: keypad matrix, GPIO key events, jog wheel/shuttle, and rotary encoders.
- Drive outputs: simple LEDs (TLC59116), RGB ring LEDs (WS2812), optional OLED display (SH1122 via LVGL).
- Exchange runtime state with host CNC software over USB HID.
- Emulate keyboard key events over a second USB HID keyboard interface.

## 1.1 Physical Device Layout (User-Described, In Progress)

Overall form factor:
- Roughly the footprint of a standard 101-key keyboard.
- Uses keyboard-style keys plus rotary and jog/shuttle controls.
- Includes monochrome display, several single-color key LEDs, and RGB LED rings around rotary knobs.

Section-by-section description (left to right, currently captured):
- Section A (left): key-only area, 4 columns x 5 rows.
- Section A exception: lower-right position uses one double-width horizontal key replacing two standard keys.
- Intended use for Section A: future G-code / MDI keypad functions.
- Section A key labels/functions: currently TBD.

- Physical gap between sections.

- Section B (next): key-only area, 4 columns x 5 rows.
- Section B exception: lower-right area combines two vertically stacked keys into one extra-tall key.
- Intended use for Section B: roughly numeric keypad + math operators + previous/next + enter, aligned with current key map in code.

- Section C (to the right of B, with a larger gap): 5 rows tall, mostly dedicated machine-control keys with LED status.
- Section C top row: two buttons, M1 Stop and Coolant, each with LED toggle/status indication.
- Section C row 2: Single Step with LED.
- Section C row 3: Pause with LED.
- Section C row 4: Stop with LED.
- Section C row 5: Cycle Start with LED.

- Section D (to the right of C, small gap): three continuous rotary encoders arranged vertically.
- Each Section D encoder has a surrounding RGB LED ring covering nearly 360 degrees with a small wedge gap at the bottom.
- Section D value model: conceptually 0-100%, with the wedge gap representing the discontinuity point.

- Section E (rightmost, final section): display at the top, with a grid of keys below, each key having LED feedback.
- Section E top key row: jog increment selection with values 0.010 inch, 0.001 inch, and 0.0001 inch.
- Section E next key row: axis selection keys X, Y, and Z.
- Section E lower control: special jog/shuttle knob with two concentric rings.
- Section E outer ring behavior: spring-return to center, supports continuous jogging with variable degree/speed.
- Section E inner ring behavior: traditional incremental encoder wheel for stepped jog moves.
- Both concentric controls operate on the currently selected movement axis.

Notes:
- This layout documentation is intentionally evolving and may be revised as ergonomics and workflows are refined.
- Additional sections to be appended as provided.

## 1.2 Operator Flow Map (Current Behavior + Planned Intent)

This map is intended to connect physical controls to operational workflows.
Items are marked as either current (observed in code) or planned/TBD.

Flow A: Manual jog workflow (current core behavior)
1. Operator selects jog increment in Section E (0.010 / 0.001 / 0.0001 inch).
2. Operator selects axis in Section E (X / Y / Z).
3. Operator moves jog controls in Section E:
- Outer spring-return ring drives continuous jog intent (variable level).
- Inner encoder drives incremental jog intent.
4. Firmware packages axis + step + jog/shuttle data into HID input report.
5. LinuxCNC host bridge applies values via HAL and command interfaces.

Flow B: Program execution control workflow (current core behavior)
1. Operator presses Section C controls (Cycle Start / Stop / Pause / Single Step / M1 Optional Stop / Coolant).
2. Firmware maps these to motion command bits and sends them to host.
3. Host bridge translates command bits to LinuxCNC commands:
- Stop -> abort
- Cycle Start -> AUTO_RUN (from IDLE), AUTO_RESUME (from normal pause), AUTO_STEP (in step mode)
- Pause -> AUTO_PAUSE (gates: READING/WAITING, not step mode, not already paused)
- Single Step -> AUTO_STEP from IDLE (enters step mode; Cycle Start then advances each step)
- M1 Optional Stop -> toggle c.set_optional_stop() on press edge
- Coolant -> toggle c.flood(FLOOD_ON/OFF) on press edge
4. Host state update returns over HID output report.
5. Firmware updates Section C LED feedback according to machine/interpreter state.

Flow C: Override tuning workflow (current core behavior)
1. Operator adjusts three Section D rotary encoders.
2. Firmware normalizes encoder segment values and sends knob1/2/3 in HID input report.
3. Host bridge maps values to feedrate, rapidrate, and max velocity overrides.
4. Firmware updates encoder RGB ring displays to indicate current values/state.

Flow D: Keyboard/MDI-style input workflow (hybrid: current + planned)
1. Section B numeric/operator keys are mapped to keyboard/HID behavior in current code.
2. Section A is intended for G-code/MDI-centric entry but function map remains TBD.
3. Host-side integration path for finalized MDI workflow is pending requirements definition.

Flow E: Display-assisted operation workflow (bring-up only)
1. Display currently serves diagnostics/bring-up feedback.
2. Final operator UI pages, alert model, and steady-state information architecture are TBD.

Design note:
- Because this is early-stage development, all flows are expected to evolve.
- Use this map as a working model for alignment, not as a fixed contract.

## 1.3 State and LED Behavior Snapshot (As Left In Last Effort)

This section captures observed LED/ring behavior from current firmware as a historical baseline.
It is intentionally not treated as final product behavior.

Machine availability behavior:
- When machine link is not alive, firmware turns off selection LEDs, interp LEDs, and all three RGB rings.
- On receipt of host traffic, machine-alive state resumes and normal state-driven LED behavior continues.

Section E selection LEDs (manual mode only):
- Increment LEDs (indices 0,1,2): one active at low brightness based on selected increment.
	- Increment 1 -> LED 0 on
	- Increment 2 -> LED 1 on
	- Increment 3 -> LED 2 on
- Axis LEDs (indices 3,4,5): one active at low brightness based on selected axis.
	- Axis 1 -> LED 3 on
	- Axis 2 -> LED 4 on
	- Axis 3 -> LED 5 on
- Outside manual mode or when estop/disabled, these selection LEDs are forced off.

Section C program-control LEDs (interp state presentation, 2026-04-20 milestone):
- LED index to physical key mapping:
	- LED 6 = M1 Optional Stop (0x3d): solid on = optional stop enabled, off = disabled
	- LED 7 = Coolant/Flood (0x3e): solid on = flood on, off = flood off
	- LED 8 = Single Step (0x33)
	- LED 9 = Pause (0x34)
	- LED 10 = Stop (0x35)
	- LED 11 = Cycle Start (0x36)
- Interp state presentation (LEDs 8-11, gated on enabled and not estop):
	- INTERP_IDLE: LED 10 (Stop) solid
	- INTERP_READING, normal run: LED 11 (Cycle Start) solid
	- INTERP_READING, step mode: LED 8 (Step) + LED 11 (Cycle Start) solid if inpos, blink if not
	- INTERP_PAUSED, normal run (AUTO_PAUSE): LED 9 (Pause) blink
	- INTERP_PAUSED, step mode: LED 8 + LED 11 blink (waiting for Cycle Start) or solid (machine moving)
	- INTERP_WAITING, normal run: LED 11 solid
	- INTERP_WAITING, step mode: LED 8 + LED 11 blink or solid (same as PAUSED step logic)
	- estop or disabled: all off
- M1 and Coolant LEDs (6 and 7) always follow host state regardless of estop/enabled.

Section D RGB encoder rings:
- Ring 0 follows encoder 1 value and uses color urgb(0x3f, 0x1f, 0x00).
- Ring 1 follows encoder 2 value and uses color urgb(0x00, 0x3f, 0x3f).
- Ring 2 follows encoder 3 value and uses color urgb(0x3f, 0x00, 0x3f).
- Rings update on encoder change events and are cleared when machine link is not alive.

Presentation caveat:
- LED index to physical legend mapping should be treated as provisional until front-panel legend/function lock is completed.

## 1.4 Control Index Cross-Reference (Current Snapshot)

Purpose:
- Avoid ambiguity like "LED 10" by tying matrix code, LED index, physical control intent, and host action together.
- This table reflects current code and should be treated as a baseline snapshot, not a locked final design.

Key/LED mapping backbone:
- key_to_led_map order in firmware is:
	- [0]=0x2f, [1]=0x30, [2]=0x37, [3]=0x38, [4]=0x39, [5]=0x3a,
	- [6]=0x3d, [7]=0x3e, [8]=0x33, [9]=0x34, [10]=0x35, [11]=0x36.

Section E and Section C controls (mapped):

| Physical Intent | Matrix Code | LED Index | Firmware Meaning | Host/HID Meaning |
|---|---:|---:|---|---|
| Jog increment 0.010 in | 0x2f | 0 | selected_increment = 1 | pkt.step = 100 (-> 0.010 in in hmi.py) |
| Jog increment 0.001 in | 0x30 | 1 | selected_increment = 2 | pkt.step = 10 (-> 0.001 in in hmi.py) |
| Jog increment 0.0001 in | 0x37 | 2 | selected_increment = 3 | pkt.step = 1 (-> 0.0001 in in hmi.py) |
| Axis X select | 0x38 | 3 | selected_axis = 1 | pkt.axis = 1 |
| Axis Y select | 0x39 | 4 | selected_axis = 2 | pkt.axis = 2 |
| Axis Z select | 0x3a | 5 | selected_axis = 3 | pkt.axis = 3 |
| M1 Optional Stop | 0x3d | 6 | motion_cmd bit 0x10 (edge-detected) | toggle c.set_optional_stop(); LED = s.optional_stop |
| Coolant/Flood | 0x3e | 7 | motion_cmd bit 0x20 (edge-detected) | toggle c.flood(ON/OFF); LED = bool(s.flood) |
| Single Step | 0x33 | 8 | motion_cmd bit 0x08 | enters step mode from IDLE; Cycle Start advances steps |
| Pause | 0x34 | 9 | motion_cmd bit 0x04 | AUTO_PAUSE (gated: READING/WAITING, not step mode) |
| Stop | 0x35 | 10 | motion_cmd bit 0x02 | always c.abort() |
| Cycle Start | 0x36 | 11 | motion_cmd bit 0x01 | AUTO_RUN/RESUME/STEP depending on state |

Section D encoder and ring mapping:

| Physical Intent | Source | Visual Channel | Firmware Field | Host Action |
|---|---|---|---|---|
| Override knob 1 | encoder 1 | RGB ring 0 | pkt.knob1 | hmi.py feedrate override |
| Override knob 2 | encoder 2 | RGB ring 1 | pkt.knob2 | hmi.py rapid override |
| Override knob 3 | encoder 3 | RGB ring 2 | pkt.knob3 | hmi.py max velocity override |

Special jog/shuttle mapping (Section E concentric control):

| Physical Control | Source Field | Host Pin/Use |
|---|---|---|
| Inner jog wheel (incremental) | pkt.jog | HAL jog.inner.value |
| Outer shuttle ring (spring return) | pkt.shuttle | HAL jog.outer.value and jog.is-shuttling |

Open labeling note:
- All Section C keys (0x3d through 0x36) are now fully implemented and locked in. No open physical-to-code mapping ambiguity remains for Section C.

## 1.5 Deployment Context and Primary Use Cases (Confirmed)

System context:
- This HMI is not intended to run standalone.
- It is intended to operate alongside a LinuxCNC GUI on a touchscreen system.

Primary GUI targets:
- Probe Basic (QtPyVCP).
- A custom QtVCP GUI derived in part from QtDragon.

Operational goal:
- Combined HMI + touchscreen interaction should replace keyboard and mouse for most machine operation.
- Target coverage is roughly 95 percent of typical usage.
- Likely exception area: editing complex G-code programs, where full keyboard/mouse workflows may still be needed.

Design implication for future work:
- Control mapping, display content, and host integration should prioritize fast operational workflows and minimize dependency on desktop-style input devices.

## 2. Build and Feature Flags

Top-level CMake options:
- ENABLE_DISPLAY: enables LVGL + SH1122 OLED path.
- ENABLE_USB: enables TinyUSB HID interfaces and USB task.

Key dependencies used directly:
- Pico SDK
- FreeRTOS kernel
- TinyUSB
- LVGL (optional)

## 2.1 2026 SDK Landscape Audit (For This Project)

Current project baseline:
- Project is configured for SDK 2.1.1 in CMake/tooling metadata.
- Current TinyUSB baseline via SDK 2.1.1 is TinyUSB 0.18.0.

Upstream changes since this project was last active:
- pico-sdk 2.2.0 is available (release date 2025-07-29) with additional bug fixes and documentation improvements.
- TinyUSB has continued advancing (latest visible release: 0.20.0), including multiple stack and controller-driver fixes over 0.18.0.

What appears most relevant to this firmware:
- USB stack evolution: TinyUSB API and behavior have continued to mature (device/host internals, endpoint handling, stability fixes).
- FreeRTOS + RP2350 alignment: SDK notes emphasize keeping FreeRTOS import/port files current for RP2350-era integration.
- Runtime and IRQ quality improvements in newer SDK versions reduce edge-case risk in interrupt-heavy and RTOS-heavy projects.

Assessment for your USB concern:
- Yes, the landscape has improved versus a year ago, especially around TinyUSB maturity and RP2350 ecosystem integration.
- Your current code already uses the newer `tusb_init(rhport, init_struct)` style, which aligns with modern TinyUSB direction.

Project-specific modernization opportunities (no behavior changes yet):
1. SDK bump trial branch
- Evaluate upgrade from 2.1.1 to 2.2.x in a dedicated branch and recreate the build directory.

2. USB descriptor hygiene
- Review USB configuration descriptor attributes and power declaration for strict spec conformance.

3. HID robustness
- Add explicit handling/logging for full USB OUT queue conditions in `tud_hid_set_report_cb`.
- Check return status from generic HID IN report send path and track dropped frames.

4. Protocol observability
- Add counters for rx packets, tx packets, queue overflows, parse rejects, and heartbeat timeouts.

5. FreeRTOS import freshness
- Re-validate `FreeRTOS_Kernel_import.cmake` and port path assumptions against current upstream guidance for RP2350 builds.

Suggested execution order:
- Step 1: SDK/toolchain upgrade branch + clean rebuild.
- Step 2: USB smoke tests (enumeration, reconnect, suspend/resume, long-run HID traffic).
- Step 3: Add observability and queue-drop instrumentation.
- Step 4: Only then consider protocol/interface refactors.

## 3. Runtime Architecture

Main initialization sequence (high level):
1. Initialize UART stdout.
2. Optional USB stack init.
3. Pulse peripheral reset pin (GPIO22).
4. Optional display init.
5. Init shared I2C bus.
6. Init task objects (encoder, matrix, LED, optional display).
7. Create FreeRTOS tasks and start scheduler.

Main tasks observed:
- MATRIX_TASK: polls TCA8418 event FIFO and pushes key/gpio events.
- ENCODER_TASK: polls quadrature PIO counters and shuttle GPIO code, pushes value events.
- LED_TASK: processes LED commands for TLC59116 and WS2812.
- DISPLAY_TASK_TIMER / DISPLAY_TASK_HANDLER / DISPLAY_GUI_TASK (optional): LVGL scheduling and UI updates.
- USB_TASK (optional): runs tinyusb tud_task periodically.
- MAIN_TASK: central state machine, consumes matrix/encoder/USB queues and updates LEDs + outbound HID reports.

Inter-task communication:
- FreeRTOS queues are the primary IPC mechanism.
- MAIN_TASK is the orchestrator that translates device events to host HID packets and visual state.

## 4. USB Protocol (Observed)

Two HID interfaces are exposed:
- Generic HID IN/OUT (64-byte report) for CNC state exchange.
- Keyboard HID interface for keycode-style commands.

Observed outbound-to-host fields (input report):
- knob1/2/3, axis, step, jog, shuttle, motion_cmd.

Observed host-to-device fields (output report):
- heartbeat, estop, enabled, mode, interp_state, feedrate_override, rapidrate_override, maxvel_override.

Behavioral notes:
- Firmware gates many actions on machine alive + not estop + enabled + mode.
- LED state reflects mode/interpreter state and selected axis/increment.
- Encoder presses from matrix GPIO map can trigger smart reset behavior for knob values.

## 4.1 LinuxCNC Host Bridge (Confirmed)

Host side implementation exists in ../linuxcnc/hmi.py and uses:
- python-hid for USB HID communication.
- linuxcnc Python API (stat + command).
- HAL component pins for observability/bridging.

Observed host -> device packet handling:
- Polls LinuxCNC status, increments heartbeat every 500 ms, and sends a 64-byte HID output report when status changes or heartbeat advances.
- Packed payload format: <BBBBBBBBB
	- header (0xAA), heartbeat, estop, enabled, mode, interp_state,
	- feedrate*100, rapidrate*100,
	- maxvel*100/configured_maxvel.

Observed device -> host packet handling:
- Reads 64-byte HID input reports and unpacks first 17 bytes with format <BBBBIiiB.
- Mapping in hmi.py:
	- knob0 -> c.feedrate(knob0/14.0)
	- knob1 -> c.rapidrate(knob1/14.0)
	- knob2 -> c.maxvel(knob2*configured_maxvel/14.0)
	- axis, jog step, jog inner, jog outer are exported to HAL pins.
	- motion_cmd bitmask drives LinuxCNC actions:
		- 0x02 stop -> c.abort()
		- 0x01 start -> AUTO_RESUME if paused else AUTO_RUN from line 1 if idle
		- 0x04 pause -> toggle pause/resume
		- 0x08 step -> AUTO_STEP

Integration implication:
- The 0..14 encoder segmentation is not just a UI choice; it is currently part of host control scaling in hmi.py.

## 5. Hardware Mapping (High-Level)

From code comments and pin constants:
- UART0: GPIO0/1
- SPI OLED: GPIO2/3/4/5
- Jog quadrature: GPIO6/7
- Shuttle code bits: GPIO8-11
- Additional encoders: GPIO12-17
- WS2812: GPIO18
- Key interrupt: GPIO19
- I2C bus: GPIO20/21
- Peripheral reset: GPIO22

## 6. Review Findings (Code Risks)

1. High: encoder set_value scaling appears incorrect and can overflow limits.
- In drivers/encoder.cc, set_value compares and assigns using already-scaled internal limits and multiplies by divisor again, then multiplies once more when storing.
- This likely causes values outside intended bounds for non-unit divisors.

2. High: task creation assert may pass on USB task failure.
- In main.cc, usb_task_status is checked as truthy instead of compared to pdPASS.
- If xTaskCreate returns a non-zero error code, assert can still pass unexpectedly.

3. Medium: LED task appears to spin without delay, potentially wasting CPU and affecting scheduling.
- task_led.cc loops continuously and toggles default LED each pass.
- vTaskDelay is commented out.

4. Medium: machine heartbeat timeout may not arm on first packet if heartbeat value does not change from zero baseline.
- main task marks machine alive on first packet, but timer reset is only tied to heartbeat value changes.
- This could keep machine_alive true longer than intended after link loss in some startup sequences.

## 7. Confirmed Decisions (2026-04-20)

1. Mode semantics
- MODE_AUTO and MODE_TELEOP are intentionally sharing the same motion-command behavior at this stage.

2. Packet hardening
- Header validation (0xAA) is considered nice-to-have, but not required yet during current bring-up.

3. HID discovery strategy
- No additional product/interface filtering is needed yet; current VID-based discovery is acceptable for now.

4. LED task cadence
- LED task timing behavior is acknowledged as likely to change soon, but no code changes are requested at this time.

5. Display scope
- Display is currently true bring-up only; final UI scope and usage are not defined yet.

## 8. Suggested Next Validation Steps

- Add a short protocol note (single source of truth) for both packet structs and scaling constants shared by firmware and hmi.py.
- Add small unit/integration checks around encoder scaling and packet conversion.
- Validate FreeRTOS task stack sizes under display + USB load.
- Confirm intended heartbeat timeout and startup behavior, given host heartbeat period of 500 ms.
- Revisit header validation, HID filtering, and LED scheduler behavior when transitioning from bring-up to stabilization.

## 9. Improvement Opportunities (Design Is Intentionally Flexible)

These are suggestions, not required immediate changes.

1. Define a versioned HID protocol contract
- Add protocol version and capability bits in packet header.
- Reserve bytes for future fields and compatibility.
- Keep one canonical spec file and generate shared constants where practical.

2. Separate transport, domain state, and UI policy
- Keep USB packet encode/decode logic isolated from machine-state logic.
- Keep LED/display policy in a dedicated layer to simplify feature iteration.
- This makes future UI/UX changes less risky for transport/control behavior.

3. Harden task timing and CPU budgeting before feature growth
- Add explicit cadence for LED/background tasks.
- Confirm priority and stack sizing with trace/telemetry snapshots.
- This reduces surprise regressions as display and interaction logic expand.

4. Improve observability for bring-up
- Add structured debug counters: packet RX/TX rate, queue depth high-water marks, timeout counts.
- Add optional lightweight status dump command/report mode for field debugging.

5. Clarify state-machine contracts
- Document allowed transitions for estop/enabled/mode/interp combinations.
- Define precedence when host state and local input conflict.
- This helps keep behavior consistent during future refactors.

6. Make host bridge evolution safer
- Consider explicit device selection (serial/product/interface) when moving past single-device bring-up.
- Add basic exception/backoff strategy in host loop for long-run stability.

7. Prepare for display roadmap uncertainty
- Treat current display as diagnostics-first.
- Define small, testable display data model independent of final screen layout.

## 10. USB Code Debt Audit: Hacks and Workarounds to Undo

This section catalogues specific things in the USB code that were put in place because the stack was
immature, the right pattern was unclear, or template code was never cleaned up. Each item notes what
was done, why it is suboptimal, and what the correct resolution is. Items are ordered from highest to
lowest impact.

---

### HACK-1: CFG_TUSB_OS=OPT_OS_NONE + manual polling USB_TASK
**Files**: `port/tinyusb/tusb_config.h`, `main.cc`, `drivers/usb.c`
**Category**: architectural workaround

What was done:
- TinyUSB is configured with `OPT_OS_NONE`, meaning it has no knowledge of FreeRTOS.
- A dedicated `USB_TASK` calls `usb_periodic()` → `tud_task()` every 1ms via `vTaskDelay(1)`.
- The USB task runs at priority 1 (the lowest in the system), so it can be starved by other tasks.

Why it is a hack:
- TinyUSB has supported FreeRTOS OS integration (`OPT_OS_FREERTOS`) since TinyUSB 0.13 and it is
  explicitly supported and tested on RP2040/RP2350 via the pico-sdk.
- With `OPT_OS_NONE`, all USB traffic processing happens only when `tud_task()` is explicitly called.
  USB interrupts still fire (the IRQ handler just sets a flag), but data is not processed until the
  poll runs. This introduces up to 1ms latency on every USB event.
- With `OPT_OS_FREERTOS`, the USB IRQ posts to a semaphore and TinyUSB wakes a dedicated task only
  when there is actual work to do. Latency drops to the IRQ → task wake path (typically <100µs).
  No continuous polling needed.

How to fix:
1. Change `CFG_TUSB_OS` from `OPT_OS_NONE` to `OPT_OS_FREERTOS` in `tusb_config.h`.
2. The pico-sdk FreeRTOS port supplies the required `osal_freertos.h` glue automatically; no custom
   port code is needed.
3. The `USB_TASK` loop changes from polling to a blocking `tud_task()` that sleeps on the semaphore:
   ```c
   void usb_task(void *unused) {
     while (true) tud_task();  // blocks internally on FreeRTOS semaphore
   }
   ```
4. `vTaskDelay(1)` in the USB task loop is removed.
5. USB task priority can be raised (to 2 or 3) since it will sleep when idle rather than burning a
   scheduling slot every tick.

Reference: pico-examples `dev_hid_freertos` demonstrates this exact pattern.
Risk: medium — requires confirming the correct FreeRTOS include path in tusb_config.h; straightforward once SDK version is confirmed post-upgrade.

---

### HACK-2: usb_hid_periodic() — dead template code never removed
**Files**: `drivers/usb.c`, `drivers/usb.h`
**Category**: leftover example code

What was done:
- `usb_hid_periodic()` was copied from a TinyUSB example. It reads `board_button_read()` and sends
  HID key 'A' when the button is held. It also has a `#ifdef REMOTE_WAKEUP` block.
- The function is declared in `usb.h` and defined in `usb.c` but is **never called anywhere** in the
  actual firmware.

Why it is a hack:
- It is dead code. The keyboard interface is managed in `main_task()` via `tud_hid_n_keyboard_report()`.
  The board button has no hardware wiring on this PCB.
- Its presence is confusing because it contains a keyboard report send path that appears to duplicate
  the one in main.cc.

How to fix:
- Delete `usb_hid_periodic()` from `drivers/usb.c`.
- Remove the declaration from `drivers/usb.h`.
- Remove the `#ifdef REMOTE_WAKEUP` block along with it.

Risk: none — function is unreachable.

---

### HACK-3: Commented-out old API call
**File**: `drivers/usb.c` line 18
**Category**: stale comment

What was done:
- `//tud_init(BOARD_TUD_RHPORT);` is left as a comment showing the old single-argument API.
- The new call directly below it uses the correct `tusb_init(rhport, &dev_init)` form.

How to fix:
- Delete the commented line.

Risk: none.

---

### HACK-4: Generic HID IN report sent without readiness check
**File**: `main.cc`
**Category**: inconsistent defensive coding

What was done:
- The keyboard send path correctly checks `tud_hid_n_ready(ITF_KEYBOARD)` before calling
  `tud_hid_n_keyboard_report()`.
- The generic HID input report send (`tud_hid_report(0, &pkt, sizeof(pkt))`) has no equivalent
  readiness check and its `bool` return value is discarded.

Why it is a hack:
- If the generic HID endpoint is not ready (e.g. host not polling, endpoint stalled, or enumeration
  in progress) the packet is silently lost with no log or counter. The inconsistency between the
  two interfaces suggests the readiness check was added to one path when it caused problems, but the
  fix was not applied uniformly.

How to fix:
```c
if (tud_hid_n_ready(ITF_GENERIC_HID)) {
  if (!tud_hid_report(0, &pkt, sizeof(pkt))) {
    printf("hid_report: send failed\n");  // or increment counter
  }
} else {
  printf("hid_report: endpoint not ready\n");
}
```

Risk: low — purely additive defensive check.

---

### HACK-5: Silent drop in tud_hid_set_report_cb
**File**: `drivers/usb.c`
**Category**: unobservable failure mode

What was done:
- `xQueueSend(usb_out_queue, &pkt, 0)` uses a zero timeout, so if the queue is full the packet is
  dropped silently.
- Using zero timeout is correct here (cannot block in a USB callback), but the drop itself is
  unlogged.

Why it is a hack:
- Dropped host→device packets mean the firmware misses machine state updates. During initial bring-up
  this was probably masked by the host resending state frequently (500ms heartbeat), but it is
  invisible without explicit logging.

How to fix:
```c
if (xQueueSend(usb_out_queue, &pkt, 0) != pdTRUE) {
  // Cannot printf here safely; use a volatile counter instead
  usb_out_queue_drops++;
}
```
Declare `volatile uint32_t usb_out_queue_drops = 0;` and log it periodically from `main_task()`.

Risk: none — purely additive.

---

### HACK-6: usb_in_pkt sends uninitialized padding bytes
**File**: `main.cc`
**Category**: information leak / undefined behavior

What was done:
- `usb_in_pkt` is a union of a packed struct and a 64-byte buffer. The struct occupies 17 bytes
  (knob1, knob2, knob3, axis=4 × uint8, step=uint32, jog=int, shuttle=int, motion_cmd=uint8).
  The remaining 47 bytes are not initialized.
- `tud_hid_report(0, &pkt, sizeof(pkt))` sends all 64 bytes. The uninitialized 47 bytes contain
  whatever was on the stack at that point.

Why it is a hack:
- Sending arbitrary stack contents is both a minor information leak and a source of spurious data if
  the host side ever tries to interpret those bytes in the future.
- By contrast, the receive-side `last_out_pkt` is correctly zeroed with `bzero()` on initialization.

How to fix:
- Add `bzero(&pkt, sizeof(pkt));` immediately before populating the struct fields in `main_task()`.

Risk: none.

---

### HACK-7: g_machine_alive is not volatile
**File**: `main.cc`
**Category**: concurrency correctness

What was done:
- `bool g_machine_alive = false;` is a plain global.
- It is read in `main_task()` (one FreeRTOS task) and written from the heartbeat timer callback
  (which executes in the FreeRTOS timer service task, a different execution context).

Why it is a hack:
- Without `volatile`, the compiler is permitted to cache the value in a register and not re-read it
  from memory on each loop iteration in `main_task()`. In practice GCC at -O0/-O1 usually re-reads
  globals, but at -O2 or with LTO this can bite.
- Writing from a timer callback and reading from a task without any FreeRTOS synchronization
  primitive is technically a data race. For a single bool it is safe in practice on RP2040 (aligned
  single-byte write is atomic), but it should be declared `volatile` to make the intent explicit and
  prevent compiler optimization issues.

How to fix:
- Change declaration to `volatile bool g_machine_alive = false;`

Risk: none.

---

### HACK-8: Duplicate #include in usb.c
**File**: `drivers/usb.c` lines 6 and 8
**Category**: copy-paste artifact

What was done:
- `#include <queue.h>` appears twice in the include block.

How to fix:
- Remove one copy.

Risk: none.

---

### HACK-9: Power declaration of 500mA in configuration descriptor
**File**: `port/tinyusb/usb_descriptors.c`
**Category**: inherited example template value

What was done:
- `TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 500)` declares 500mA as the
  maximum bus power draw. This was copied from a TinyUSB example template.

Why it is a hack:
- A USB HID device drawing 500mA is unusual and some hosts will flag it. Realistic draw for this
  device (RP2040/2350 + HID only, no motors or heavy peripherals on USB power) is well under 200mA.
- Advertising 500mA may prevent enumeration on hosts with strict power budgeting (e.g., powered
  USB hubs with per-port current limits).

How to fix:
- Change to a realistic value, e.g. 200 (200mA) or 100 (100mA).
- Note: attribute byte `0x00` means bus-powered, no remote wakeup. If remote wakeup is ever wanted,
  bit 5 of the attribute byte must also be set.

Risk: low — change only affects what the descriptor advertises, not actual current draw.

---

### Summary Table

| # | Issue | File(s) | Impact | Effort |
|---|---|---|---|---|
| HACK-1 | OPT_OS_NONE polling instead of FreeRTOS integration | tusb_config.h, main.cc, usb.c | High — USB latency, CPU waste | Medium |
| HACK-2 | Dead usb_hid_periodic() template code | usb.c, usb.h | Low — confusion, dead code | Trivial |
| HACK-3 | Commented-out old API call | usb.c | Cosmetic | Trivial |
| HACK-4 | No readiness check on generic HID IN send | main.cc | Medium — silent packet loss | Low |
| HACK-5 | Silent drop in tud_hid_set_report_cb | usb.c | Medium — unobservable failure | Low |
| HACK-6 | Uninitialized padding bytes sent in HID report | main.cc | Low/medium — stack leak | Trivial |
| HACK-7 | g_machine_alive not volatile | main.cc | Low-medium — concurrency risk | Trivial |
| HACK-8 | Duplicate #include | usb.c | Cosmetic | Trivial |
| HACK-9 | 500mA power declaration | usb_descriptors.c | Low — descriptor inaccuracy | Trivial |

Recommended cleanup order:
1. HACK-2, 3, 6, 7, 8, 9 — all trivial; do in one pass before any other USB work.
2. HACK-4 and HACK-5 — add readiness guard and drop counter; do before testing USB stability.
3. HACK-1 — do after SDK upgrade (Section 2.1 Step 1) is confirmed working; this is the main
   architectural improvement and should be tested in isolation.

---

## 11. Active Debug Session Log (2026-04-20)

This section records the specific bugs found, root causes diagnosed, and fixes applied during the
first active debugging session after project reactivation. Intended as a reference for future work.

---

### 11.1 INI MAX_LINEAR_VELOCITY Mismatch (Scale Bug)

**Symptom**: LinuxCNC AXIS showed 60 IPM at 100% but the HMI maxvel ring showed the 4th LED (~25%).

**Root cause**: `hmi_sim.ini` had `[TRAJ] MAX_LINEAR_VELOCITY = 4` while all axis `MAX_VELOCITY`
entries were 1.0 unit/sec. `hmi.py` reads `[TRAJ] MAX_LINEAR_VELOCITY` at startup into
`configured_maxvel`. Because `configured_maxvel = 4.0` but actual ceiling was 1.0:

    OUT packet: int(round(1.0 * 14.0 / 4.0)) = 3  →  LED 3 (not 14)

The previous fix attempt had changed the wrong instance — there were two `MAX_LINEAR_VELOCITY` lines
(one in `[DISPLAY]` or similar, one in `[TRAJ]`). The `[TRAJ]` entry was still `= 4`.

**Fix**: Changed `[TRAJ] MAX_LINEAR_VELOCITY = 4` → `= 1` in `hmi_sim.ini`.

**Lesson**: When `hmi.py` and LinuxCNC disagree on the machine velocity ceiling, all three overrides
will display with wrong scale. Always verify the `[TRAJ]` section specifically; other INI sections
may also have `MAX_LINEAR_VELOCITY` entries that are not read by `hmi.py`.

---

### 11.2 Encoder Divisor Regression (4 Steps per Detent)

**Symptom**: Each physical detent of the override knobs jumped the value by 4 instead of 1.

**Root cause**: During a previous experimental session, `set_limits(1/2/3, 0, 14, 1)` had been
set with `div=1`. The physical encoders produce 4 quadrature counts per detent. With `div=1` the
internal counter and logical value are the same, so each detent incremented the logical value by 4.

**Encoder divisor mechanics** (`drivers/encoder.cc`):
- `set_limits(num, min, max, div)` stores internal limits as `min*div` and `max*div`.
- `value(num)` returns `m_cur_values[num] / div`.
- With `div=4`, 4 raw counts → 1 logical step. With `div=1`, 4 raw counts → 4 logical steps.

**Fix**: Restored `set_limits(1, 0, 14, 4)`, `set_limits(2, 0, 14, 4)`, `set_limits(3, 0, 14, 4)`
in `task_encoder.cc`.

**Compatibility note**: `set_encoder_value(enc, val)` (used for startup sync) calls `set_value()`
which stores `val * divisor` internally, so it remains correct with `div=4`.

---

### 11.3 USB Disconnect: LEDs and Display Not Reset

**Symptom**: When USB disconnected, the rings/LEDs held their last state instead of going dark,
and the display held the last encoder/key text instead of returning to the scrolling banner.

**Fix**: On the transition to "not alive" (`!leds_cleared`), in addition to clearing all LEDs,
a `DISPLAY_CMD_RESET` command is now sent to the display task:
- Added `DISPLAY_CMD_RESET` to `TaskDisplay::cmds` enum in `task_display.hh`.
- Handler in `task_display.cc::gui_task` restores `LV_LABEL_LONG_SCROLL_CIRCULAR` and resets
  label text to the boot banner string.
- Encoder/key update handlers now also set `LV_LABEL_LONG_CLIP` before updating text (previously
  the label stayed in scroll mode when the first encoder event arrived, which looked odd).
- `main.cc` disconnect handler sends the `DISPLAY_CMD_RESET` message under `#ifdef ENABLE_DISPLAY`.

---

### 11.4 WS2812 Ring LED Blip / Wrong Color on Wrong Ring

This was the most complex bug in the session. Multiple root causes were investigated.

#### 11.4.1 Batching Fix (Partial Improvement)

**Symptom**: When all three rings updated together (e.g. `machine_state_changed`), three separate
`update()` calls with intermediate states briefly visible.

**Root cause**: Each `set_ring_led(..., update_now=true)` triggered a full 45-pixel frame write.
Three sequential writes meant ring 0 could be seen with rings 1/2 still showing old state.

**Fix**: Added `LED_CMD_FLUSH` to `TaskLED::cmds`. All three ring commands now use
`update_now=false`, followed by a single `LED_CMD_FLUSH` queued after all three. The LED task
dequeues them in order: three buffer-only updates, then one `update()` call.

#### 11.4.2 WS2812 Frame Alignment (Root Cause)

**Symptom**: Magenta/cyan dots appearing briefly on the yellow ring (ring 0) when rapidly turning
knob 3 (ring 2). Knob 1 (ring 0) caused no artifacts on others. Pattern was asymmetric and
speed-dependent.

**Root cause diagnosis**: `pio_sm_put_blocking` is a **CPU-side busy-wait loop** on FIFO space:

    while (pio_sm_is_tx_fifo_full(pio, sm)) tight_loop_contents();
    pio->txf[sm] = value;

If FreeRTOS preempts the LED task (priority 2) mid-loop in favour of the encoder/matrix tasks
(priority 4), the CPU stops feeding the FIFO. The PIO runs independently and drains the remaining
FIFO entries (~4 × 30µs = 120µs max). Once the FIFO empties, the PIO **stalls** on
`out x, 1 side 0`, which holds the data pin **LOW**. If the stall exceeds ~50µs, the WS2812
strip latches the partial frame. When the LED task resumes, subsequent pixels are received by
the strip as a new frame starting from LED 0 — the remaining ring 2 pixels overwrite ring 0
and ring 1.

The asymmetry follows from send order: ring 0 is transmitted first, so it is most exposed to
preemption mid-frame when ring 2 knob turns arrive. Turning ring 0's knob never affects
others because rings 1/2 haven't started transmitting yet when preemption hits.

**Attempted fixes (did not fully resolve)**:
1. FIFO drain loop + `sleep_us(60)` after `update()` — 60µs was too short (OSR drain ~30µs
   + WS2812 reset >50µs = need 80µs+).
2. Increased to `sleep_us(100)` — still not robust; drain timing depends on where in the PIO
   cycle the FIFO check fires, and OSR overlap is not accounted for.
3. Absolute inter-frame timer (`busy_wait_until next_update_at`) — enforces minimum 1450µs
   between frame starts; correct but doesn't prevent mid-frame preemption.

**Actual fix**: `taskENTER_CRITICAL()` / `taskEXIT_CRITICAL()` wrapping the pixel write loop
in `ws2812.cc::update()`. This raises the interrupt priority mask, preventing FreeRTOS from
scheduling any task during the ~1.35ms pixel write. The CPU continuously feeds the FIFO with
no gaps, so the PIO never stalls.

```cpp
taskENTER_CRITICAL();
next_update_at = make_timeout_time_us(1450);
for (uint8_t i = 0; i < WS2812_NUM_LEDS; i++) {
    put_pixel(m_pio, m_sm, m_leds[i]);
}
taskEXIT_CRITICAL();
```

`busy_wait_until(next_update_at)` remains outside the critical section to avoid spinning with
preemption disabled longer than needed.

**Known limitation**: 1.35ms with preemption disabled is long for a FreeRTOS critical section.
At very high encoder spin speeds, encoder events could theoretically be missed during this window.
In practice, the encoder task (priority 4) queues events; it is the queue consumer (main_task)
that is delayed, not the event producer. No missed inputs observed in testing.

**Preferred future fix**: DMA transfer — see Section 12 (Pending Work).

---

### 11.5 Summary of Files Changed This Session

| File | Change |
|---|---|
| `linuxcnc/hmi_sim/hmi_sim.ini` | `[TRAJ] MAX_LINEAR_VELOCITY` corrected: `4` → `1` |
| `firmware/task_encoder.cc` | Encoder divisor restored: `div=1` → `div=4` for encoders 1-3 |
| `firmware/task_display.hh` | Added `DISPLAY_CMD_RESET` to `cmds` enum |
| `firmware/task_display.cc` | Handle `DISPLAY_CMD_RESET`; set `LONG_CLIP` before encoder/key text |
| `firmware/task_led.hh` | Added `LED_CMD_FLUSH` to `cmds` enum |
| `firmware/task_led.cc` | Handle `LED_CMD_FLUSH` → calls `m_rgbleds.update()` |
| `firmware/main.cc` | Send `DISPLAY_CMD_RESET` on disconnect; batch ring updates with single flush |
| `firmware/drivers/ws2812.cc` | Added `pico/time.h`, `FreeRTOS.h`, `task.h`; `update()` uses critical section + absolute inter-frame timer |

---

## 12. Pending Work

### 12.1 WS2812 DMA Refactor

The current `taskENTER_CRITICAL()` fix works but disables preemption for ~1.35ms. The correct
long-term fix is DMA:

- Allocate a 45-element `uint32_t` DMA buffer (pixels pre-shifted `<< 8` for PIO format).
- `dma_channel_configure()` with PIO TX FIFO as write address, `dreq = pio_get_dreq(pio, sm, true)`,
  32-bit transfers, read-increment.
- Fire with `dma_channel_start()` — CPU immediately free, PIO fed from DMA with no gaps.
- On next `update()`, wait for completion: `dma_channel_wait_for_finish_blocking()` or
  IRQ + semaphore for non-blocking.
- Keep `busy_wait_until(next_update_at)` for inter-frame gap enforcement.
- No critical section needed; fully preemption-safe.

### 12.2 hmi.py USB Reconnection

`hmi.py` has no reconnect loop. If the HMI USB device disconnects, the script will crash or hang.
Requirements for a safe implementation:
- Detect HID device loss via read/write exception.
- Poll `hid.enumerate()` until the device reappears; re-open when found.
- Do NOT call any LinuxCNC HAL/command APIs during the disconnected period (HAL pins hold last value).
- Do NOT restart LinuxCNC or the HAL component.
- On reconnect, force a full OUT packet re-sync to the HMI so all LEDs come up correct immediately.

