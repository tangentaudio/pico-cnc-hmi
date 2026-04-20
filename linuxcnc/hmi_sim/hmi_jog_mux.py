#!/usr/bin/env python3
import time
import hal

COMP = hal.component("hmi_jog_mux")

# Inputs from hmi.py HAL component
COMP.newpin("axis", hal.HAL_U32, hal.HAL_IN)
COMP.newpin("step", hal.HAL_FLOAT, hal.HAL_IN)
COMP.newpin("inner_value", hal.HAL_S32, hal.HAL_IN)
COMP.newpin("outer_value", hal.HAL_FLOAT, hal.HAL_IN)
COMP.newpin("is_shuttling", hal.HAL_BIT, hal.HAL_IN)

# Jog outputs to LinuxCNC AXIS/JOINT jog pins
COMP.newpin("x_counts", hal.HAL_S32, hal.HAL_OUT)
COMP.newpin("y_counts", hal.HAL_S32, hal.HAL_OUT)
COMP.newpin("z_counts", hal.HAL_S32, hal.HAL_OUT)
COMP.newpin("enable", hal.HAL_BIT, hal.HAL_OUT)
COMP.newpin("vel_mode", hal.HAL_BIT, hal.HAL_OUT)
COMP.newpin("scale", hal.HAL_FLOAT, hal.HAL_OUT)

COMP.ready()

last_inner = int(COMP["inner_value"])
x_counts = 0
y_counts = 0
z_counts = 0

while True:
    axis = int(COMP["axis"])
    step = float(COMP["step"])
    inner = int(COMP["inner_value"])

    # Convert the absolute wheel value into a count delta, then apply it to the selected axis.
    delta = inner - last_inner
    last_inner = inner

    if delta != 0:
        if axis == 1:
            x_counts += delta
        elif axis == 2:
            y_counts += delta
        elif axis == 3:
            z_counts += delta

    COMP["x_counts"] = x_counts
    COMP["y_counts"] = y_counts
    COMP["z_counts"] = z_counts

    COMP["enable"] = True
    COMP["vel_mode"] = bool(COMP["is_shuttling"])

    # Keep scale positive and sane; default to a conservative jog step if input is invalid.
    if step <= 0.0:
        step = 0.001
    COMP["scale"] = step

    # HAL userspace loop cadence. 1ms keeps jogging responsive in simulation.
    time.sleep(0.001)
