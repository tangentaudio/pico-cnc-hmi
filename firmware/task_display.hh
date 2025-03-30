#ifndef __TASK_DISPLAY_HH__
#define __TASK_DISPLAY_HH__

#include <FreeRTOS.h>
#include "queue.h"
#include "lvgl/lvgl.h"
#include "oled_sh1122.hh"
#include "spi.hh"

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

    typedef enum
    {
        DISPLAY_CMD_UPDATE_ENCODER
    } cmds;

    typedef struct cmd
    {
        cmds cmd;
        uint8_t encoder;
        int8_t value;
    } cmd_t;

    QueueHandle_t cmd_queue;

protected:
    static void align_area(lv_event_t *e);

private:
    SPI m_spi;
    OLED m_oled;
    lv_display_t *m_display;
    uint8_t m_disp_buf1[LV_BUFSIZE];
};

#endif
