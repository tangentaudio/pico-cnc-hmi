- Project: Pico 2 CNC HMI firmware + LinuxCNC host bridge.
- Maturity: early bring-up/debug/POC; dormant ~1 year as of 2026-04-20.
- Host bridge: ../linuxcnc/hmi.py (python-hid + linuxcnc stat/command + HAL pins).
- HID: vendor 0xCAFE, dual interface (generic in/out + keyboard).
- Host heartbeat increments every 500 ms in hmi.py.
- Encoder scaling in host: knob values interpreted as 0..14 for feed/rapid/maxvel overrides.
- Outstanding risks: encoder set_value scaling logic in firmware, USB task assert check, LED task tight loop.
- Confirmed decisions (2026-04-20): MODE_AUTO and MODE_TELEOP share motion behavior for now; no header validation yet; no stricter HID filtering yet; LED task changes deferred; display remains bring-up-only with scope TBD.
- Design guidance preference: treat current implementations as provisional in early stage; include improvement suggestions and redesign options rather than assuming present structure is fixed.
- Physical layout (in-progress): device roughly 101-key keyboard size; monochrome display; some single-color key LEDs; rotary knobs with RGB rings.
- Section A: 4x5 keys, lower-right has one horizontal 2u key; intended future G-code/MDI area; labels/functions TBD.
- Section B: 4x5 keys, lower-right has one vertical 2u key; intended numeric/operator + prev/next + enter area aligned with current key map.
- Section C: to right of B with larger gap; 5 rows machine-control keys: top row M1 Stop + Coolant (both LED status, implemented), then Single Step (LED, implemented), Pause (LED, implemented), Stop (LED, implemented), Cycle Start (LED, implemented).
- Section C key codes: M1 Stop=0x3d/LED6, Coolant=0x3e/LED7, Step=0x33/LED8, Pause=0x34/LED9, Stop=0x35/LED10, Cycle Start=0x36/LED11.
- Section D: to right of C with small gap; 3 vertical continuous rotary encoders, each with RGB ring (near-360 with bottom wedge gap), conceptual 0-100% with wedge marking discontinuity.
- Section E (rightmost/final): display at top; below is LED-backed key grid.
- Section E key rows: top row jog increment select (0.010 in, 0.001 in, 0.0001 in); next row axis select (X/Y/Z).
- Section E jog control: concentric jog/shuttle knob; outer ring spring-centers for continuous variable jogging; inner ring is incremental encoder wheel; both act on selected axis.
- Operator flow map (working): Manual jog flow via Section E (increment+axis+concentric jog), program control flow via Section C motion keys, override tuning flow via Section D encoders, keyboard/MDI hybrid via Sections B/A (A still TBD), display flow currently diagnostics-only.
- LED/state snapshot captured in summary: manual-mode increment/axis LEDs use indices 0-5, interp presentation uses LEDs 8-11, encoder rings 0-2 map to knobs 1-3 with fixed colors; all cleared when machine link not alive.
- Documentation preference: refer to controls by physical function names (e.g., Cycle Start, Axis X) alongside matrix code, not by LED index alone.
- Deployment model: HMI is not standalone; always paired with LinuxCNC touchscreen GUI.
- Primary GUI targets: Probe Basic (QtPyVCP) and custom QtVCP derived from QtDragon.
- Product goal: HMI + touchscreen should replace keyboard/mouse for ~95% of use; likely exception is complex G-code editing.
- 2026 SDK audit: project baseline is pico-sdk 2.1.1 (TinyUSB 0.18.0). Upstream has moved to pico-sdk 2.2.0 and newer TinyUSB releases; recommend branch-based SDK bump trial, USB robustness instrumentation, and FreeRTOS import freshness check before larger refactors.









