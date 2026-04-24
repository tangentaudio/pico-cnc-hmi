#ifndef HMI_KEYMAP_H
#define HMI_KEYMAP_H

// ── TCA8418 matrix key codes (Section B & C) ─────────────────────
// These are the raw codes returned by the TCA8418 key scanner.

// Section B: jog increment and axis selectors
#define KEY_INCR_LG       0x2f   // 0.010"
#define KEY_INCR_MD       0x30   // 0.001"
#define KEY_INCR_SM       0x37   // 0.0001"
#define KEY_AXIS_X        0x38
#define KEY_AXIS_Y        0x39
#define KEY_AXIS_Z        0x3a

// Section C: program control + toggle buttons
#define KEY_SINGLE_STEP   0x33
#define KEY_PAUSE         0x34
#define KEY_STOP          0x35
#define KEY_CYCLE_START   0x36
#define KEY_OPT_STOP      0x3d   // M1 Optional Stop
#define KEY_COOLANT       0x3e   // Coolant/Flood


// ── TLC59116 LED indices ─────────────────────────────────────────
// The TLC59116 drives individual LEDs; these are their channel numbers.

// Section B: jog increment and axis indicator LEDs
#define LED_INCR_LG        0    // 0.010"
#define LED_INCR_MD        1    // 0.001"
#define LED_INCR_SM        2    // 0.0001"
#define LED_AXIS_X         3
#define LED_AXIS_Y         4
#define LED_AXIS_Z         5

// Section C: toggle and program control LEDs
#define LED_OPT_STOP       6    // M1 Optional Stop
#define LED_COOLANT        7    // Coolant/Flood
#define LED_SINGLE_STEP    8
#define LED_PAUSE          9
#define LED_STOP          10
#define LED_CYCLE_START   11

#endif // HMI_KEYMAP_H
