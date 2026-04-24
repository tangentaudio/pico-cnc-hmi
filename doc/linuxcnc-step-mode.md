# LinuxCNC single-step mode discrimination

## Key finding
`task_paused=1` appears in BOTH step mode AND normal AUTO_PAUSE.
The correct discriminator is `exec_state`:

| Scenario | task_paused | exec_state | in_step_mode() |
|---|---|---|---|
| Step mode, non-motion step waiting | 1 | 2 (EXEC_DONE) | True |
| Step mode, after motion step waiting | 1 | 5 (EXEC_WAITING_FOR_IO) | True |
| Normal run AUTO_PAUSE | 1 | 7 (EXEC_WAITING_FOR_MOTION_AND_IO) | False |
| Normal run / idle | 0 | any | False |

## in_step_mode() implementation
```python
def in_step_mode() -> bool:
    if not status['task_paused']:
        return False
    return status['exec_state'] != linuxcnc.EXEC_WAITING_FOR_MOTION_AND_IO
```

## interp_state constants on LinuxCNC 2.9.8
INTERP_IDLE=1, INTERP_READING=2, INTERP_PAUSED=3, INTERP_WAITING=4 (sequential, NOT power-of-2)

## OUT packet layout (hmi.py → firmware)
Byte 0: 0xaa header
Byte 1: heartbeat
Byte 2: estop
Byte 3: enabled
Byte 4: mode
Byte 5: interp_state (mapped sequential)
Byte 6: feedrate_override (0-14 segments)
Byte 7: rapidrate_override (0-14 segments)
Byte 8: maxvel_override (0-14 segments)
Byte 9: in_step_mode() (0/1)
Byte 10: inpos (0/1)
Byte 11: optional_stop (0/1)
Byte 12: coolant/flood (0/1)
Total: 13 structured bytes + pad to 64. Format string: '<BBBBBBBBBBBBB'

## Single-step LED behaviour (firmware)
- PAUSED/WAITING + task_paused=1, inpos=True  → step(8)+run(11) BLINK (waiting for Cycle Start)
- PAUSED/WAITING + task_paused=1, inpos=False → step(8)+run(11) SOLID (machine moving)
- READING + task_paused=1                      → step(8)+run(11) SOLID (executing)
- PAUSED + task_paused=0                       → pause(9) BLINK (normal AUTO_PAUSE)
- READING/WAITING + task_paused=0              → run(11) SOLID (normal run)
- IDLE                                          → stop(10) SOLID

## Button gating (hmi.py)
- CYCLE START from step PAUSED/WAITING → AUTO_STEP
- CYCLE START from normal PAUSED → AUTO_RESUME
- CYCLE START from IDLE → AUTO_RUN
- STEP from IDLE only → enters step mode via AUTO_STEP
- PAUSE only fires when READING/WAITING and not in_step_mode()
- STOP always aborts
