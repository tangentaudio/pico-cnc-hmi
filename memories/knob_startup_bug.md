# Knob Startup Race Condition — FULLY RESOLVED (2026-04-22)

## Original Symptom
On fresh startup after USB connect, the first press of a knob toggle button
(rapidrate/maxvel/feedrate) did nothing.  The second press worked correctly.

## Root Cause (final)
A two-layer startup sequencing problem:

1. **LinuxCNC transitional packets (hmi.py side):** `s.feedrate/rapidrate/max_velocity`
   return 0.0 transiently during LinuxCNC initialization before settling to their real
   defaults.  The Python bridge was seeding `HAL['knob.*']` immediately on connect,
   capturing these zeros.  The first OUT packet therefore carried `feedrate=0, rapidrate=0,
   maxvel=0`.

2. **Firmware only synced encoders once (main.cc side):** The firmware performed an
   initial encoder sync on the *first* OUT packet only (`if (!initial_feedrate)`).
   With the transient zeros in that first packet, the encoder was committed to 0.
   Subsequent packets changed the values to their real defaults and updated the ring
   LEDs, but never re-synced the encoder — so `get_value()` still returned 0.
   The first button press then computed `cur=0, target=reset_seg` and "set" the
   encoder to its already-correct value, wasting the press.

## Resolution (2026-04-22 Round 4)

### Fix 1: hmi.py — 150ms stabilization delay
Before seeding `HAL['knob.*']`, sleep 150ms then re-poll so that LinuxCNC's
override values have settled to their real defaults.  This ensures the first OUT
packet carries correct segment values in the common case.

File: `linuxcnc/hmi.py` — reconnect block, before knob seed lines.

### Fix 2: firmware — per-encoder `enc_user_moved[]` flag + `sync_value_immediate()`
Belt-and-suspenders for cases where the Python delay is insufficient (e.g. very
slow LinuxCNC startup).

- `enc_user_moved[4]` in `main_task`: reset to false at every reconnect.
- Encoder event handler: sets `enc_user_moved[encoder]` when any enc 1–3 event
  fires (physical turn or button toggle).
- OUT packet handler: calls `task_encoder->sync_value_immediate(enc, value)` on
  every `feedrate_changed / rapidrate_changed / maxvel_changed` event *while*
  `enc_user_moved[enc]` is false.  Once the user touches a knob, that encoder is
  no longer chased.

### Fix 3: firmware — `sync_value_immediate()` API on TaskEncoder
Single-encoder synchronous mutex-protected setter added alongside the existing
`sync_all_immediate()`.  Bypasses the async cmd queue so `get_value()` returns
the correct value immediately (no 10ms wait for the encoder task loop).

### Architectural improvements (same session)
- `last_value[5]` promoted from a `task()` local to `m_last_value[5]` member of
  `TaskEncoder`, enabling `sync_*_immediate()` to update both `m_cur_values` and
  the change-detection shadow atomically under the mutex.
- Mutex coverage extended to protect `m_last_value` in all write paths
  (SET_VALUE, SET_VALUE_NOTIFY, SMART_SET_VALUE, hardware poll) — previously
  only `get_value()` reads were guarded.
- `ENCODER_CMD_SYNC_ALL` removed: replaced entirely by `sync_all_immediate()`
  (called at reconnect) and `sync_value_immediate()` (called on each override
  change while user hasn't moved the encoder).
- `cmd_t` simplified: union/sync struct removed now that SYNC_ALL is gone.

## Key Files
- `linuxcnc/hmi.py` — 150ms settle delay before knob seed
- `firmware/main.cc` — `enc_user_moved[]`, reconnect reset, OUT packet re-sync logic
- `firmware/task_encoder.hh` — `sync_value_immediate()` declaration, `m_last_value` member
- `firmware/task_encoder.cc` — `sync_value_immediate()` implementation, extended mutex coverage

## Prior Fix Rounds (context)
- **Round 1 (2026-04-21):** `m_last_values{0,0,0,0}` in encoder.cc; first OUT packet
  fully processed; IN packet gated on initial sync flags.
- **Round 2 (2026-04-22):** `set_value()` clamping double-scaling bug fixed.
- **Round 3 (2026-04-22):** `ENCODER_CMD_SYNC_ALL` for atomic 3-encoder batch sync
  (replaced by Round 4's synchronous API).
