#include <stdio.h>
#include <FreeRTOS.h>
#include "semphr.h"
#include <task.h>
#include "spi.hh"
#include "oled_sh1122.hh"
#include "lvgl.h"
#include "task_display.hh"

extern lv_font_t roboto_64;

TaskDisplay::TaskDisplay()
    : m_spi(),
      m_oled(m_spi),
      m_display(nullptr)
{

}

TaskDisplay::~TaskDisplay()
{
}

void TaskDisplay::init()
{
    mutex = xSemaphoreCreateMutex();    

    LVGL_LOCK(mutex);

    m_spi.init();
    m_oled.init();

    lv_init();

    m_display = lv_display_create(SH1122_HOR_RES, SH1122_VER_RES);

    lv_display_set_color_format(m_display, LV_COLOR_FORMAT);

    lv_display_set_user_data(m_display, &m_oled);

    lv_display_set_buffers(m_display, m_disp_buf1, NULL, sizeof(m_disp_buf1), LV_DISPLAY_RENDER_MODE_FULL);

    lv_display_set_flush_cb(m_display, [](lv_display_t *display, const lv_area_t *area, uint8_t *px_map)
                            {
                                OLED *oled = static_cast<OLED *>(lv_display_get_user_data(display));
                                oled->lv_sh1122_flush_cb(display, area, px_map);
                            });


    #if LV_COLOR_DEPTH == 8
    lv_display_add_event_cb(m_display, align_area, LV_EVENT_INVALIDATE_AREA, nullptr);
    lv_display_set_antialiasing(m_display, true);
    #endif

    lv_tick_set_cb(xTaskGetTickCount);

    lv_display_set_default(m_display);

    LVGL_UNLOCK(mutex);
}

#if LV_COLOR_DEPTH == 8
void TaskDisplay::align_area(lv_event_t *e)
{
  auto *area = (lv_area_t *)lv_event_get_param(e);

  printf("align_area before %d %d %d %d  ", area->x1, area->y1, area->x2, area->y2);

  area->x1 &= ~3;
  area->x2 = ((area->x2 + 4) & ~3) - 1;
  printf("after %d %d %d %d\n", area->x1, area->y1, area->x2, area->y2);
}
#endif

void TaskDisplay::timer_task(void *param)
{
    TaskDisplay *inst = static_cast<TaskDisplay *>(param);

    printf("display timer task\n");

    while (true)
    {
        LVGL_LOCK(inst->mutex);
        lv_timer_handler_run_in_period(5);
        LVGL_UNLOCK(inst->mutex);

        vTaskDelay(5 / portTICK_PERIOD_MS);

    }
}


void TaskDisplay::task_handler_task(void *param)
{
    TaskDisplay *inst = static_cast<TaskDisplay *>(param);

    printf("display task_handler task\n");

    while (true)
    {
      LVGL_LOCK(inst->mutex);
      lv_task_handler();
      LVGL_UNLOCK(inst->mutex);
      vTaskDelay(100 / portTICK_PERIOD_MS);

    }
}

void TaskDisplay::gui_task(void *param)
{
    TaskDisplay *inst = static_cast<TaskDisplay *>(param);

    LVGL_LOCK(inst->mutex);

    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_text_font(lv_screen_active(), &roboto_64, LV_PART_MAIN);
    lv_obj_set_style_text_color(lv_screen_active(), lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_line_color(lv_screen_active(), lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_line_width(lv_screen_active(), 1, LV_PART_MAIN);

    lv_obj_t *label = lv_label_create(lv_screen_active());
    lv_label_set_text(label, "~~ LVGLv9 and FreeRTOS on Raspberry Pi Pico2 ~~");
    lv_obj_set_pos(label, 0, 0);
    lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_width(label, 256);
    
    LVGL_UNLOCK(inst->mutex);

    while (true)
    {
      LVGL_LOCK(inst->mutex);
      lv_label_set_text_fmt(label, "%u", xTaskGetTickCount() / portTICK_PERIOD_MS);
      LVGL_UNLOCK(inst->mutex);
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

/*

lv_obj_set_style_bg_color(lv_screen_active(), lv_color_black(), LV_PART_MAIN);
lv_obj_set_style_text_font(lv_screen_active(), &lv_font_montserrat_28, LV_PART_MAIN);
lv_obj_set_style_line_color(lv_screen_active(), lv_color_white(), LV_PART_MAIN);
lv_obj_set_style_line_width(lv_screen_active(), 1, LV_PART_MAIN);

lv_obj_t *labeljog = lv_label_create(lv_screen_active());
lv_label_set_text(labeljog, "");
lv_obj_set_pos(labeljog, 64, 0);

lv_obj_t *labelkey = lv_label_create(lv_screen_active());
lv_label_set_text(labelkey, "");
lv_obj_set_pos(labelkey, 0, 32);

static lv_style_t style_bg;
static lv_style_t style_indic;

lv_style_init(&style_bg);
lv_style_set_border_color(&style_bg, lv_color_white());
lv_style_set_border_width(&style_bg, 2);
lv_style_set_pad_all(&style_bg, 4);
lv_style_set_radius(&style_bg, 2);
lv_style_set_anim_duration(&style_bg, 100);

lv_style_init(&style_indic);
lv_style_set_bg_opa(&style_indic, LV_OPA_COVER);
lv_style_set_bg_color(&style_indic, lv_color_white());
lv_style_set_radius(&style_indic, 1);

lv_obj_t *barshuttle = lv_bar_create(lv_screen_active());
lv_bar_set_orientation(barshuttle, LV_BAR_ORIENTATION_HORIZONTAL);
lv_bar_set_range(barshuttle, -7, 7);
lv_bar_set_mode(barshuttle, LV_BAR_MODE_SYMMETRICAL);
lv_obj_remove_style_all(barshuttle);
lv_obj_add_style(barshuttle, &style_bg, 0);
lv_obj_add_style(barshuttle, &style_indic, LV_PART_INDICATOR);
lv_bar_set_value(barshuttle, 0, LV_ANIM_OFF);
lv_obj_set_size(barshuttle, 128, 24);
lv_obj_set_pos(barshuttle, 128, 0);

lv_obj_t *barencoder[3];
for (uint8_t i = 0; i < 3; i++)
{
  lv_obj_t *barenc = lv_bar_create(lv_screen_active());
  lv_bar_set_orientation(barenc, LV_BAR_ORIENTATION_HORIZONTAL);
  lv_bar_set_range(barenc, 0, 14);
  lv_bar_set_mode(barenc, LV_BAR_MODE_NORMAL);
  lv_obj_remove_style_all(barenc);
  lv_obj_add_style(barenc, &style_bg, 0);
  lv_obj_add_style(barenc, &style_indic, LV_PART_INDICATOR);
  lv_bar_set_value(barenc, 0, LV_ANIM_OFF);
  lv_obj_set_size(barenc, 36, 20);
  lv_obj_set_pos(barenc, 128 + (38 * i), 32);

  barencoder[i] = barenc;
}

uint32_t time_till_next = lv_timer_handler();

static absolute_time_t last_time = get_absolute_time();
absolute_time_t since_last = last_time;
absolute_time_t now = last_time;
static int t = 0;



if (pressed)
  lv_label_set_text_fmt(labelkey, LV_SYMBOL_DOWN "%2.2X", key);
else
  lv_label_set_text_fmt(labelkey, LV_SYMBOL_UP "%2.2X", key);



lv_label_set_text_fmt(labeljog, "%5d", jog);
lv_bar_set_value(barshuttle, shuttle, LV_ANIM_OFF);
lv_bar_set_value(barencoder[0], v1, LV_ANIM_OFF);
lv_bar_set_value(barencoder[1], v2, LV_ANIM_OFF);
lv_bar_set_value(barencoder[2], v3, LV_ANIM_OFF);


*/