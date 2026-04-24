#ifndef __TASK_DISPLAY_HH__
#define __TASK_DISPLAY_HH__

#include <FreeRTOS.h>
#include "queue.h"
#include "lvgl/lvgl.h"
#include "oled_sh1122.hh"
#include "spi.hh"
#include "drivers/usb.h"   // for interp_t, mode_t, hmi_config_t

#if LV_COLOR_DEPTH == 1
#define LV_BUFSIZE ((SH1122_HOR_RES * SH1122_VER_RES / 8) + 8)
#define LV_COLOR_FORMAT LV_COLOR_FORMAT_I1
#elif LV_COLOR_DEPTH == 8
#define LV_BUFSIZE (SH1122_HOR_RES * SH1122_VER_RES)
#define LV_COLOR_FORMAT LV_COLOR_FORMAT_L8
#else
#pragma message "Unsupported color depth"
#endif

#define LVGL_LOCK(x) xSemaphoreTake(x, portMAX_DELAY);
#define LVGL_UNLOCK(x) xSemaphoreGive(x)

// How long a transient overlay stays on screen before returning to status (ms).
#define DISPLAY_TRANSIENT_MS 1000

// Uncomment to show a transient overlay with the raw keycode on every key press.
// Useful during bringup; leave disabled for normal use.
//#define DISPLAY_DEBUG_KEYS

class TaskDisplay
{
public:
    TaskDisplay();
    ~TaskDisplay();

    void init();
    static void timer_task(void *param);
    static void task_handler_task(void *param);
    static void gui_task(void *param);

    SemaphoreHandle_t mutex;

    // ---------------------------------------------------------------
    // Commands sent to the display from main_task / other tasks.
    // ---------------------------------------------------------------
    typedef enum
    {
        // Disconnected — no USB host present.
        DISPLAY_CMD_DISCONNECTED,

        // Full machine-state update.  Sent on every OUT packet change.
        DISPLAY_CMD_MACHINE_STATE,

        // E-stop transition — takeover screen.
        DISPLAY_CMD_ESTOP,

        // Knob (encoder override) turned — transient overlay.
        DISPLAY_CMD_KNOB,

        // Jog encoder moved — transient overlay.
        DISPLAY_CMD_JOG,

        // Increment or axis selection button pressed — transient overlay.
        DISPLAY_CMD_JOG_SELECT,

        // Key pressed — transient overlay.
        DISPLAY_CMD_KEY,

        // About to reboot into BOOTSEL — show message until MCU resets.
        DISPLAY_CMD_BOOTSEL,
    } cmds;

    // Machine state snapshot packed into one command.
    typedef struct {
        bool     estop;
        bool     enabled;
        bool     homed;
        bool     coolant;
        mode_t   mode;
        interp_t interp_state;
        bool     step_mode;           // hmi_step_mode from host (was task_paused)
        bool     paused;          // s.paused from LinuxCNC (was inpos)
        // Override segment values (0-14) as last seen from OUT packet.
        uint8_t  feed_seg;
        uint8_t  rapid_seg;
        uint8_t  maxvel_seg;
    } machine_state_t;

    typedef struct cmd
    {
        cmds cmd;

        // DISPLAY_CMD_MACHINE_STATE / DISPLAY_CMD_ESTOP
        machine_state_t state;

        // DISPLAY_CMD_KNOB: which knob (0=feed,1=rapid,2=maxvel), segment value
        uint8_t knob_index;
        uint8_t knob_seg;

        // DISPLAY_CMD_JOG: current axis positions and selected axis
        float   jog_pos[3];    // [0]=X [1]=Y [2]=Z, machine coordinates
        uint8_t jog_axis;      // selected axis (1=X, 2=Y, 3=Z)
        bool    jog_continuous; // true = outer ring / continuous, false = inner wheel / incremental

        // DISPLAY_CMD_JOG_SELECT: new axis or increment selection
        // jog_select_is_axis: true = axis changed, false = increment changed
        // jog_axis: 1=X 2=Y 3=Z (when is_axis)
        // jog_increment: 1=0.010 2=0.001 3=0.0001 (when !is_axis)
        bool    jog_select_is_axis;
        uint8_t jog_increment;  // 1/2/3

        // DISPLAY_CMD_KEY: keycode and press/release
        uint8_t key_code;
        bool    key_press;

        // DISPLAY_CMD_BOOTSEL: text to show before reboot
        char bootsel_text[32];
    } cmd_t;

    QueueHandle_t cmd_queue;

    // ---------------------------------------------------------------
    // Jog overlay — shared dirty-flag struct (no queue, no overflow).
    // main_task writes directly; gui_task polls each loop iteration.
    // ---------------------------------------------------------------
    struct jog_overlay_t {
        volatile bool dirty;        // set by main_task, cleared by gui_task
        uint8_t axis;               // selected axis 1=X 2=Y 3=Z
        bool    continuous;         // true = shuttle, false = mpg wheel
        // Pre-formatted by main_task to avoid float snprintf on gui_task stack.
        char    top_text[32];       // e.g. "Jog Continuous X"
        char    pos_text[24];       // e.g. "+1.2345"
    };
    jog_overlay_t jog_overlay;

protected:
    static void align_area(lv_event_t *e);

private:
    SPI m_spi;
    OLED m_oled;
    lv_display_t *m_display;
    uint8_t m_disp_buf1[LV_BUFSIZE];
};

#endif
