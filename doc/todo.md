# Pending Work

## WS2812 DMA refactor (LED task)
Current fix: `taskENTER_CRITICAL()` wraps the 45-pixel `put_pixel` loop in `ws2812.cc::update()`.
This works but holds preemption disabled for ~1.35ms (45 pixels × 30µs at 800kHz).

**Why the fix was needed:**
- `pio_sm_put_blocking` is a CPU-side busy-wait on FIFO space, not a DMA transfer
- If FreeRTOS preempts the LED task mid-loop (encoder/matrix tasks run at priority 4 vs LED at 2),
  the CPU stops feeding the FIFO
- The PIO keeps running independently and drains the remaining FIFO entries (~120µs for 4 deep)
- Once FIFO empty, PIO stalls on `out x, 1 side 0` which holds the data pin **LOW**
- >50µs of LOW = WS2812 reset/latch — strip latches the partial frame at whatever pixel it reached
- Next pixels after resume land shifted — cyan/magenta appear on the yellow ring etc.
- The effect was asymmetric: turning knob 3 (ring 2, sent last) corrupted rings 0 and 1
  because those were still in-flight when preemption hit; knob 1 (ring 0, sent first)
  never affected the others because rings 1 and 2 were not yet started

**Preferred long-term fix: DMA**
- Allocate a 45-element uint32 buffer (shifted left 8 for PIO format, same as `put_pixel`)
- Use `dma_channel_configure()` with the PIO TX FIFO as the write address,
  dreq = `pio_get_dreq(pio, sm, true)`, 32-bit transfers, read-increment
- Fire with `dma_channel_start()` — CPU immediately free, PIO fed from DMA with no gaps
- On next `update()` call, wait for DMA completion before re-arming:
  `dma_channel_wait_for_finish_blocking()` (or use an IRQ + semaphore for non-blocking)
- Still need the inter-frame gap (>50µs after last pixel) — track with `make_timeout_time_us`
  same as current `next_update_at` logic
- No critical section needed; fully preemption-safe

## Jog overlay (transient display) — DISABLED, TODO revisit

**What it does:** When a jog encoder event fires, show a transient overlay on the OLED
displaying "Jog Continuous X +0.3316" (or Incremental) for 2000ms, then revert to the
status screen.

**Current state:** `DISPLAY_CMD_JOG` send is disabled in `main.cc` (both the encoder-event
path and the pos-feedback path are wrapped in `if (false)`). The display task `gui_task`
still has the JOG case handler intact.

**Root cause of failure (best understanding):**
The display queue (depth 10) fills up faster than `gui_task` can drain it. This happens
because `gui_task` calls `lv_timer_handler_run_in_period(5)` on every loop iteration, which
triggers LVGL rendering → calls `lv_sh1122_flush_cb()` → calls `spi_write_blocking()` for a
full 256×64 frame at 24MHz SPI. The frame flush takes ~7ms (256×64÷2 bytes ÷ 24Mbit/s ≈
341µs minimum, but with SPI overhead and OLED addressing commands closer to 5-10ms). During
this time `gui_task` cannot service the queue. Meanwhile the encoder ISR + `main_task` keep
filling it at rates up to ~50 events/sec (shuttle + pos-feedback JOGs). Once full, every
subsequent JOG cmd is dropped, `transient_until` stops advancing, and the display freezes on
the last value shown.

**Approaches to try when revisiting:**
- **Option A: Dirty-flag coalescing** — instead of queuing every JOG event, `main_task`
  writes to a shared `volatile` struct (axis, pos, continuous flag) and sets a dirty flag.
  `gui_task` reads the latest value on each loop iteration — no queue needed, always shows
  the most recent position, cannot overflow.
- **Option B: Separate overlay task** — dedicated task with its own timer, only wakes on
  semaphore signal from encoder event; handles the overlay without touching the main
  status-rendering path.
- **Option C: Reduce LVGL render frequency** — only call `lv_timer_handler_run_in_period`
  every N iterations, or gate it to only run when LVGL has pending dirty regions.
- **Option D: Partial display updates** — configure LVGL for partial-region rendering so
  only the overlay region is flushed when the overlay changes, not the full 256×64 frame.
  The SH1122 supports column/row addressing so this is hardware-feasible.

**Files involved:**
- `firmware/main.cc` — encoder event handler (search `// DISPLAY_CMD_JOG disabled`)
- `firmware/main.cc` — pos-feedback path (search `// DISPLAY_CMD_JOG disabled`)
- `firmware/task_display.cc` — `DISPLAY_CMD_JOG` case in `gui_task` (intact, ~line 373)

---

## COMPLETED

### USB BOOTSEL reboot + firmware updater (2026-04-24)
Magic command byte `cmd=0xB0` in OUT packet offset 14 triggers `reset_usb_boot(0, 0)`.
`linuxcnc/update_firmware.py` automates the full cycle: BOOTSEL reboot → wait for
RP2350/RPI-RP2 mass storage mount → copy .uf2 → wait for device to come back online.
Display shows "USB Boot" message before reset (display clears when bootloader resets GPIOs).

### Step-mode LED and step stacking: velocity-based signals (2026-04-24)
Replaced `inpos`-based logic with `s.current_vel` for both the LED blink/solid indicator
and the step stacking guard. `inpos` stays False after mid-motion PAUSE; `s.paused` stays
True throughout step execution. `current_vel` reliably transitions 0↔>0 in all tested
scenarios: step-from-idle, pause→step, pause-mid-step→resume. Wire protocol byte 10
renamed `inpos`→`paused` (carries `s.paused`), byte 9 renamed `task_paused`→`step_mode`.

### Section C program control (2026-04-20)
All six Section C keys implemented: Stop, Cycle Start, Pause, Single Step, M1 Optional Stop,
Coolant/Flood. motion_cmd bits: 0x01=start, 0x02=stop, 0x04=pause, 0x08=step,
0x10=opt_stop edge, 0x20=coolant edge.

### Homed state gating (2026-04-21)
`homed` byte in OUT packet. Program control buttons gated when not homed.
STOP LED (LED10) slow-blinks at 1s period when enabled but not homed.

### HACK cleanup session (2026-04-21)
HACK-2 through HACK-9 all resolved. See git history for details.

### LED queue depth bug (2026-04-21)
LED task cmd_queue raised from depth 10 → 32. Fixed LED 7 flush flag.

### hmi.py USB reconnection (prior)
Outer reconnect loop. Safe stop on disconnect. Status dict reset forces full re-sync on reconnect.
HAL knob seeded to current LinuxCNC values (not 0xFFFF) so first IN packet doesn't spuriously set overrides.

### Override knob startup sync — full rework (2026-04-22)
Replaced async `ENCODER_CMD_SYNC_ALL` with synchronous `sync_all_immediate()` /
`sync_value_immediate()` API on `TaskEncoder`. Extended mutex protection to shadow values.
Added `enc_user_moved[]` flags: encoders re-sync to host on every override change until the
user physically touches that knob. Added 150ms settle delay in `hmi.py` before seeding
`HAL['knob.*']`. See `doc/knob_startup_bug.md` for full root-cause analysis.

### HID interface selection fix (2026-04-22)
`hmi.py` `find_hid_device()` now filters by `interface_number==0` (Generic HID) then
`usage_page==0xFF00` as fallback, instead of non-deterministic `devlist[0]`. Eliminates
silent no-sync when the keyboard interface was picked. Added `cfg_pkt` write retry ×3.
Added `max_velocity` float rounding to suppress jitter-driven spurious status updates.

### STOP+CYCLE START chord: home all axes (2026-04-22)
Hold STOP then CYCLE START for 2 seconds (only when enabled and not homed) to trigger
`c.home(-1)` and pulse `hmi.home-all` → `halui.home-all`. STOP LED fast-blinks (4 Hz)
during countdown; both LEDs flash full brightness on fire. Bit 0x40 added to `motion_cmd`
byte. Guard: chord only active when `!estop && enabled && !homed`.

### Probe Basic (QtPyVCP) sim config (2026-04-22)
`linuxcnc/pb_sim/` — standalone config alongside `hmi_sim/` (AXIS).
- 3-axis XYZ, inch, same HMI bridge and wiring as hmi_sim
- `DISPLAY = probe_basic`; venv (`~/dev/venv/bin/`) prepended to PATH in `run_pb_sim.sh`
- HAL: `core_sim_3.hal` + `simulated_home_probe_basic_3.hal` + `spindle_sim.hal`
- Local `hallib/probe_basic_postgui.hal` loads `not names=pb-not` to avoid conflict
  with `hmi_wiring.hal`'s `names=hmi-not` (POSIX sim can't `loadrt` same module twice)
- `subroutines/`, `user_buttons/`, `user_dro_display/`, `custom_config.yml`,
  `user_atc_buttons/` all symlinked from `~/dev/probe_basic/configs/probe_basic/`

### hmi.home-all HAL pin (2026-04-22)
`hmi.home-all` BIT OUT pin added to hmi.py. Pulsed TRUE for one cycle on the
STOP+CYCLE-START chord (bit 0x40). Wired to `halui.home-all` in both
`hmi_sim/hmi_wiring.hal` and `pb_sim/hmi_wiring.hal`. This is the same
path Probe Basic's "REF ALL" button uses, so the UI tracks homing correctly.
