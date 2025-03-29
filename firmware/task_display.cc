

#ifdef ENABLE_DISPLAY
#include "spi.hh"
#include "oled_sh1122.hh"
#include "lvgl.h"
#endif

#ifdef ENABLE_DISPLAY
SPI spi;
OLED oled(spi);
#endif

#ifdef ENABLE_DISPLAY
void align_area(lv_event_t *e)
{
  auto *area = (lv_area_t *)lv_event_get_param(e);

  printf("align_area before %d %d %d %d  ", area->x1, area->y1, area->x2, area->y2);

  area->x1 &= ~3;
  area->x2 = ((area->x2 + 4) & ~3) - 1;
  printf("after %d %d %d %d\n", area->x1, area->y1, area->x2, area->y2);
}
#endif

#ifdef ENABLE_DISPLAY
printf("SPI init...");
spi.init();
printf("OK\n");

printf("OLED init...");
oled.init();
printf("OK\n");

printf("LVGL init...");

lv_init();

#if LV_COLOR_DEPTH == 1
#define LV_BUFSIZE ((SH1122_HOR_RES * SH1122_VER_RES / 8) + 8)
#define LV_COLOR_FORMAT LV_COLOR_FORMAT_I1
#elif LV_COLOR_DEPTH == 8
#define LV_BUFSIZE (SH1122_HOR_RES * SH1122_VER_RES)
#define LV_COLOR_FORMAT LV_COLOR_FORMAT_L8
#else
#pragma message "Unsupported color depth"
#endif

static uint8_t disp_buf1[LV_BUFSIZE];
lv_display_t *display = lv_display_create(SH1122_HOR_RES, SH1122_VER_RES);
lv_display_set_color_format(display, LV_COLOR_FORMAT);
lv_display_set_user_data(display, &oled);

lv_display_set_buffers(display, disp_buf1, NULL, sizeof(disp_buf1), LV_DISPLAY_RENDER_MODE_FULL);
lv_display_set_flush_cb(display, [](lv_display_t *display, const lv_area_t *area, uint8_t *px_map)
                        {
      OLED* oled = static_cast<OLED*>(lv_display_get_user_data(display));
      oled->lv_sh1122_flush_cb(display, area, px_map); });

// lv_display_add_event_cb(display, align_area, LV_EVENT_INVALIDATE_AREA, nullptr);

lv_display_set_default(display);
// lv_display_set_antialiasing(display, true);

printf("OK\n");
#endif

#ifdef ENABLE_DISPLAY
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
#endif

#ifdef ENABLE_DISPLAY
if (pressed)
  lv_label_set_text_fmt(labelkey, LV_SYMBOL_DOWN "%2.2X", key);
else
  lv_label_set_text_fmt(labelkey, LV_SYMBOL_UP "%2.2X", key);
#endif

#ifdef ENABLE_DISPLAY
lv_label_set_text_fmt(labeljog, "%5d", jog);
lv_bar_set_value(barshuttle, shuttle, LV_ANIM_OFF);
lv_bar_set_value(barencoder[0], v1, LV_ANIM_OFF);
lv_bar_set_value(barencoder[1], v2, LV_ANIM_OFF);
lv_bar_set_value(barencoder[2], v3, LV_ANIM_OFF);
#endif

#ifdef ENABLE_DISPLAY
absolute_time_t now = get_absolute_time();

int64_t elapsed_time_us = absolute_time_diff_us(last_time, now);
int64_t elapsed_time_ms = elapsed_time_us / 1000;

int64_t since_last_us = absolute_time_diff_us(since_last, now);
int64_t since_last_ms = since_last_us / 1000;

if (since_last_ms >= 33)
{
  time_till_next = lv_timer_handler();

  lv_task_handler();
  lv_tick_inc(since_last_ms);

  since_last = now;
}

last_time = now;
#endif
