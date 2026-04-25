#include <stdio.h>
#include <FreeRTOS.h>
#include "semphr.h"
#include <task.h>
#include "spi.hh"
#include "oled_sh1122.hh"
#include "lvgl.h"
#include "task_display.hh"
#include "version_gen.h"

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
  cmd_queue = xQueueCreate(10, sizeof(cmd_t));
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
                                oled->lv_sh1122_flush_cb(display, area, px_map); });

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

  area->x1 &= ~3;
  area->x2 = ((area->x2 + 4) & ~3) - 1;
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

  // ---------------------------------------------------------------
  // Screen geometry constants
  // ---------------------------------------------------------------
  static constexpr int W  = SH1122_HOR_RES;   // 256
  static constexpr int H  = SH1122_VER_RES;    // 64
  static constexpr int MID = H / 2;            // 32 — row boundary

  // ---------------------------------------------------------------
  // Base style: black background, white text/lines
  // ---------------------------------------------------------------
  LVGL_LOCK(inst->mutex);

  lv_obj_t *scr = lv_screen_active();
  lv_obj_set_style_bg_color(scr, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(scr,  LV_OPA_COVER,      LV_PART_MAIN);
  lv_obj_set_style_text_color(scr, lv_color_white(), LV_PART_MAIN);

  // Horizontal divider between rows (y = 31)
  static lv_point_precise_t div_pts[2] = {{0, MID - 1}, {W - 1, MID - 1}};
  lv_obj_t *divider = lv_line_create(scr);
  lv_line_set_points(divider, div_pts, 2);
  lv_obj_set_style_line_color(divider, lv_color_make(0x30, 0x30, 0x30), LV_PART_MAIN);
  lv_obj_set_style_line_width(divider, 1, LV_PART_MAIN);
  lv_obj_set_style_line_opa(divider, LV_OPA_TRANSP, LV_PART_MAIN); // hidden until connected

  // ---------------------------------------------------------------
  // Row 1 labels  (y=0..31)
  // ---------------------------------------------------------------
  // Extern declarations for generated fonts
  // Fallback from Roboto → FontAwesome is baked into the generated .c files at build time.
  extern const lv_font_t roboto_14;
  extern const lv_font_t roboto_18;
  extern const lv_font_t roboto_20;
  extern const lv_font_t roboto_28;

  // Power / machine-enabled indicator (left)
  lv_obj_t *lbl_power = lv_label_create(scr);
  lv_obj_set_style_text_font(lbl_power, &roboto_18, LV_PART_MAIN);
  lv_obj_set_pos(lbl_power, 2, 5);
  lv_label_set_text(lbl_power, "");

  // Mode (MAN / AUTO / MDI)
  lv_obj_t *lbl_mode = lv_label_create(scr);
  lv_obj_set_style_text_font(lbl_mode, &roboto_14, LV_PART_MAIN);
  lv_obj_align(lbl_mode, LV_ALIGN_TOP_LEFT, 3, 8);
  lv_label_set_text(lbl_mode, "");

  // Interp state (IDLE / RUN / PAUSE / STEP) — fixed-width centred in row 1
  lv_obj_t *lbl_interp = lv_label_create(scr);
  lv_obj_set_style_text_font(lbl_interp, &roboto_18, LV_PART_MAIN);
  lv_obj_set_style_text_align(lbl_interp, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_set_width(lbl_interp, 90);
  lv_obj_align(lbl_interp, LV_ALIGN_TOP_MID, 0, 5);
  lv_label_set_text(lbl_interp, "");

  // Homed indicator — LV_SYMBOL_HOME icon, right side of row 1
  lv_obj_t *lbl_homed = lv_label_create(scr);
  lv_obj_set_style_text_font(lbl_homed, &roboto_20, LV_PART_MAIN);
  lv_obj_align(lbl_homed, LV_ALIGN_TOP_RIGHT, -3, 5);
  lv_label_set_text(lbl_homed, "");

  // Slash drawn over the home icon when not homed.
  // Coordinates approximate the icon bounding box at font_20, TOP_RIGHT (-3, 5).
  static lv_point_precise_t slash_pts[2] = {{233, 28}, {254, 4}};
  lv_obj_t *slash_homed = lv_line_create(scr);
  lv_line_set_points(slash_homed, slash_pts, 2);
  lv_obj_set_style_line_color(slash_homed, lv_color_white(), LV_PART_MAIN);
  lv_obj_set_style_line_width(slash_homed, 2, LV_PART_MAIN);
  lv_obj_add_flag(slash_homed, LV_OBJ_FLAG_HIDDEN);

  // ---------------------------------------------------------------
  // Row 2 labels  (y=32..63)
  // ---------------------------------------------------------------
  extern const lv_font_t jetbrainsmono_20_4bpp;
  lv_obj_t *lbl_feed = lv_label_create(scr);
  lv_obj_set_style_text_font(lbl_feed, &jetbrainsmono_20_4bpp, LV_PART_MAIN);
  lv_obj_set_pos(lbl_feed, 2, MID + 6);
  lv_label_set_text(lbl_feed, "");

  lv_obj_t *lbl_rapid = lv_label_create(scr);
  lv_obj_set_style_text_font(lbl_rapid, &jetbrainsmono_20_4bpp, LV_PART_MAIN);
  lv_obj_set_pos(lbl_rapid, 84, MID + 6);
  lv_label_set_text(lbl_rapid, "");

  lv_obj_t *lbl_vel = lv_label_create(scr);
  lv_obj_set_style_text_font(lbl_vel, &jetbrainsmono_20_4bpp, LV_PART_MAIN);
  lv_obj_set_pos(lbl_vel, 162, MID + 6);
  lv_label_set_text(lbl_vel, "");

  // ---------------------------------------------------------------
  // Overlay panels — one per overlay type, each with fixed layout.
  // A helper lambda creates the common full-screen black backdrop.
  // ---------------------------------------------------------------
  extern const lv_font_t jetbrainsmono_48;

  auto make_overlay_panel = [&]() -> lv_obj_t* {
    lv_obj_t *p = lv_obj_create(scr);
    lv_obj_set_size(p, W, H);
    lv_obj_set_pos(p, 0, 0);
    lv_obj_set_style_bg_color(p, lv_color_black(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(p, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(p, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(p, 0, LV_PART_MAIN);
    lv_obj_add_flag(p, LV_OBJ_FLAG_HIDDEN);
    return p;
  };

  // --- Jog overlay: dim axis letter (left) + position (right) + speed bar (bottom) ---
  lv_obj_t *ov_jog = make_overlay_panel();
  lv_obj_t *lbl_jog_axis = lv_label_create(ov_jog);
  lv_obj_set_style_text_font(lbl_jog_axis, &jetbrainsmono_48, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl_jog_axis, lv_color_make(0x60, 0x60, 0x60), LV_PART_MAIN);
  lv_obj_align(lbl_jog_axis, LV_ALIGN_LEFT_MID, 4, -5);
  lv_label_set_text(lbl_jog_axis, "");

  lv_obj_t *lbl_jog_pos = lv_label_create(ov_jog);
  lv_obj_set_style_text_font(lbl_jog_pos, &jetbrainsmono_48, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl_jog_pos, lv_color_white(), LV_PART_MAIN);
  lv_obj_align(lbl_jog_pos, LV_ALIGN_LEFT_MID, 40, -5);
  lv_label_set_text(lbl_jog_pos, "");

  // Segmented speed bar: 7 segments left (reverse) + 7 right (forward).
  // Each segment is a small filled rectangle at the bottom of the overlay.
  // Brightness ramps from dim (level 1) to bright (level 7).
  static constexpr int SEG_COUNT = 7;         // segments per direction
  static constexpr int SEG_W     = 14;        // segment width in pixels
  static constexpr int SEG_H     = 5;         // segment height in pixels
  static constexpr int SEG_GAP   = 2;         // gap between segments
  static constexpr int SEG_Y     = H - SEG_H - 2;  // y position (2px bottom margin)
  static constexpr int CENTER_GAP = 6;        // gap between left and right groups
  // center of display
  static constexpr int CX = W / 2;

  lv_obj_t *jog_seg_left[SEG_COUNT];   // left/reverse segments (index 0 = innermost)
  lv_obj_t *jog_seg_right[SEG_COUNT];  // right/forward segments (index 0 = innermost)

  for (int i = 0; i < SEG_COUNT; i++) {
    // Right segments: start from center going right
    int rx = CX + CENTER_GAP / 2 + i * (SEG_W + SEG_GAP);
    jog_seg_right[i] = lv_obj_create(ov_jog);
    lv_obj_set_size(jog_seg_right[i], SEG_W, SEG_H);
    lv_obj_set_pos(jog_seg_right[i], rx, SEG_Y);
    lv_obj_set_style_bg_opa(jog_seg_right[i], LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(jog_seg_right[i], lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_border_width(jog_seg_right[i], 0, LV_PART_MAIN);
    lv_obj_set_style_radius(jog_seg_right[i], 1, LV_PART_MAIN);
    lv_obj_set_style_pad_all(jog_seg_right[i], 0, LV_PART_MAIN);
    lv_obj_remove_flag(jog_seg_right[i], LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(jog_seg_right[i], LV_OBJ_FLAG_HIDDEN);

    // Left segments: start from center going left
    int lx = CX - CENTER_GAP / 2 - (i + 1) * SEG_W - i * SEG_GAP;
    jog_seg_left[i] = lv_obj_create(ov_jog);
    lv_obj_set_size(jog_seg_left[i], SEG_W, SEG_H);
    lv_obj_set_pos(jog_seg_left[i], lx, SEG_Y);
    lv_obj_set_style_bg_opa(jog_seg_left[i], LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_color(jog_seg_left[i], lv_color_white(), LV_PART_MAIN);
    lv_obj_set_style_border_width(jog_seg_left[i], 0, LV_PART_MAIN);
    lv_obj_set_style_radius(jog_seg_left[i], 1, LV_PART_MAIN);
    lv_obj_set_style_pad_all(jog_seg_left[i], 0, LV_PART_MAIN);
    lv_obj_remove_flag(jog_seg_left[i], LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(jog_seg_left[i], LV_OBJ_FLAG_HIDDEN);
  }

  // --- Knob overlay: dim F/R/V letter (left) + value (right) ---
  lv_obj_t *ov_knob = make_overlay_panel();
  lv_obj_t *lbl_knob_letter = lv_label_create(ov_knob);
  lv_obj_set_style_text_font(lbl_knob_letter, &jetbrainsmono_48, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl_knob_letter, lv_color_make(0x60, 0x60, 0x60), LV_PART_MAIN);
  lv_obj_align(lbl_knob_letter, LV_ALIGN_LEFT_MID, 4, 0);
  lv_label_set_text(lbl_knob_letter, "");

  lv_obj_t *lbl_knob_val = lv_label_create(ov_knob);
  lv_obj_set_style_text_font(lbl_knob_val, &jetbrainsmono_48, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl_knob_val, lv_color_white(), LV_PART_MAIN);
  lv_obj_align(lbl_knob_val, LV_ALIGN_LEFT_MID, 40, 0);
  lv_label_set_text(lbl_knob_val, "");

  // --- Jog-select overlay: title (top) + value (bottom, centered) ---
  lv_obj_t *ov_select = make_overlay_panel();
  lv_obj_t *lbl_select_title = lv_label_create(ov_select);
  lv_obj_set_style_text_font(lbl_select_title, &roboto_18, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl_select_title, lv_color_white(), LV_PART_MAIN);
  lv_obj_align(lbl_select_title, LV_ALIGN_TOP_MID, 0, 2);
  lv_label_set_text(lbl_select_title, "");

  lv_obj_t *lbl_select_val = lv_label_create(ov_select);
  lv_obj_set_style_text_font(lbl_select_val, &roboto_28, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl_select_val, lv_color_white(), LV_PART_MAIN);
  lv_obj_align(lbl_select_val, LV_ALIGN_BOTTOM_MID, 0, -2);
  lv_label_set_text(lbl_select_val, "");

  // --- Bootsel overlay: USB boot message (centered) ---
  lv_obj_t *ov_bootsel = make_overlay_panel();
  lv_obj_t *lbl_boot_title = lv_label_create(ov_bootsel);
  lv_obj_set_style_text_font(lbl_boot_title, &roboto_18, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl_boot_title, lv_color_white(), LV_PART_MAIN);
  lv_obj_align(lbl_boot_title, LV_ALIGN_TOP_MID, 0, 2);
  lv_label_set_text(lbl_boot_title, "");

  lv_obj_t *lbl_boot_text = lv_label_create(ov_bootsel);
  lv_obj_set_style_text_font(lbl_boot_text, &roboto_18, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl_boot_text, lv_color_white(), LV_PART_MAIN);
  lv_obj_align(lbl_boot_text, LV_ALIGN_BOTTOM_MID, 0, -2);
  lv_label_set_text(lbl_boot_text, "");

  // Helper: hide all overlay panels.
  auto hide_all_overlays = [&]() {
    lv_obj_add_flag(ov_jog,     LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ov_knob,    LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ov_select,  LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(ov_bootsel, LV_OBJ_FLAG_HIDDEN);
  };

  // Which overlay is currently active (for transient expiry).
  lv_obj_t *active_overlay = nullptr;

  // ---------------------------------------------------------------
  // E-stop full-screen panel (on top of everything, hidden initially)
  // ---------------------------------------------------------------
  lv_obj_t *estop_panel = lv_obj_create(scr);
  lv_obj_set_size(estop_panel, W, H);
  lv_obj_set_pos(estop_panel, 0, 0);
  lv_obj_set_style_bg_color(estop_panel, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(estop_panel, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(estop_panel, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(estop_panel, 0, LV_PART_MAIN);
  lv_obj_add_flag(estop_panel, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t *lbl_estop = lv_label_create(estop_panel);
  lv_obj_set_style_text_font(lbl_estop, &roboto_28, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl_estop, lv_color_white(), LV_PART_MAIN);
  lv_obj_align(lbl_estop, LV_ALIGN_CENTER, 0, 0);
  lv_label_set_text(lbl_estop, LV_SYMBOL_WARNING "  E-STOP  " LV_SYMBOL_WARNING);

  // E-stop blink animation helper: cycles text opacity.
  auto estop_anim_cb = [](void *obj, int32_t v) {
    lv_obj_set_style_text_opa((lv_obj_t *)obj, (lv_opa_t)v, LV_PART_MAIN);
  };
  lv_anim_t estop_anim;
  lv_anim_init(&estop_anim);
  lv_anim_set_var(&estop_anim, lbl_estop);
  lv_anim_set_exec_cb(&estop_anim, estop_anim_cb);
  lv_anim_set_values(&estop_anim, LV_OPA_COVER, LV_OPA_20);
  lv_anim_set_duration(&estop_anim, 400);
  lv_anim_set_playback_duration(&estop_anim, 400);
  lv_anim_set_repeat_count(&estop_anim, LV_ANIM_REPEAT_INFINITE);

  // ---------------------------------------------------------------
  // Waiting-for-connection screen (visible until first machine state)
  // ---------------------------------------------------------------
  lv_obj_t *wait_panel = lv_obj_create(scr);
  lv_obj_set_size(wait_panel, W, H);
  lv_obj_set_pos(wait_panel, 0, 0);
  lv_obj_set_style_bg_color(wait_panel, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(wait_panel, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(wait_panel, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(wait_panel, 0, LV_PART_MAIN);

  lv_obj_t *lbl_wait = lv_label_create(wait_panel);
  lv_obj_set_style_text_font(lbl_wait, &roboto_20, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl_wait, lv_color_white(), LV_PART_MAIN);
  lv_obj_align(lbl_wait, LV_ALIGN_CENTER, 0, -10);
  lv_label_set_text(lbl_wait, "Pico CNC HMI");

  lv_obj_t *lbl_version = lv_label_create(wait_panel);
  lv_obj_set_style_text_font(lbl_version, &roboto_14, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl_version, lv_color_make(0x80, 0x80, 0x80), LV_PART_MAIN);
  lv_obj_align(lbl_version, LV_ALIGN_CENTER, 0, 10);
  lv_label_set_text(lbl_version, "v" FW_VERSION_STRING);

  LVGL_UNLOCK(inst->mutex);

  // ---------------------------------------------------------------
  // Helper lambdas to format override values using hmi_config
  // ---------------------------------------------------------------
  auto fmt_feed  = [](char *buf, size_t n, uint8_t seg) {
    if (!hmi_config.valid || hmi_config.max_feed_pct == 0) {
      snprintf(buf, n, "-%%");
    } else {
      snprintf(buf, n, "%u%%", hmi_feed_pct(seg));
    }
  };
  auto fmt_rapid = [](char *buf, size_t n, uint8_t seg) {
    if (!hmi_config.valid || hmi_config.max_rapid_pct == 0) {
      snprintf(buf, n, "-%%");
    } else {
      snprintf(buf, n, "%u%%", hmi_rapid_pct(seg));
    }
  };
  auto fmt_vel   = [](char *buf, size_t n, uint8_t seg) {
    if (!hmi_config.valid || hmi_config.max_vel_x10 == 0) {
      snprintf(buf, n, "-");
    } else {
      uint16_t v = hmi_maxvel_x10(seg);
      snprintf(buf, n, "%u.%u", v / 10, v % 10);
    }
  };

  // State tracking
  // ---------------------------------------------------------------
  bool connected   = false;
  bool in_estop    = false;
  TickType_t transient_until = 0;   // non-zero = overlay visible until this tick

  machine_state_t ms = {};

  // Incremental jog flash animation state.
  // When the MPG wheel clicks, all 7 segments flash briefly in the jog
  // direction — giving crisp visual feedback for each click.
  bool       jog_anim_active = false;
  int8_t     jog_anim_dir    = 0;       // +1 or -1
  TickType_t jog_anim_start  = 0;
  static constexpr int JOG_FLASH_MS = 100;  // how long the flash stays on

  while (true)
  {
    cmd_t cmd;
    bool got_cmd = (xQueueReceive(inst->cmd_queue, &cmd, pdMS_TO_TICKS(50)) == pdTRUE);

    // ----------------------------------------------------------
    // Transient overlay expiry
    // Checked before command dispatch so that goto update_status_row
    // in DISPLAY_CMD_MACHINE_STATE / DISPLAY_CMD_ESTOP cannot skip it.
    // ----------------------------------------------------------
    if (transient_until != 0 && xTaskGetTickCount() >= transient_until)
    {
      printf("[disp] overlay EXPIRED at tick %lu\n", (unsigned long)xTaskGetTickCount());
      transient_until = 0;
      hide_all_overlays();
      active_overlay = nullptr;
      jog_anim_active = false;  // kill any running animation
    }

    // ----------------------------------------------------------
    // Jog overlay — polled from shared dirty-flag struct.
    // Text is pre-formatted by main_task; gui_task just renders it.
    // Throttled to max 20Hz to limit LVGL relayout pressure.
    // The transient expiry timer is only extended when we actually
    // render, so stale/redundant dirty flags won't keep the overlay
    // visible forever.
    // ----------------------------------------------------------
    if (connected && inst->jog_overlay.dirty)
    {
      inst->jog_overlay.dirty = false;
      TickType_t now = xTaskGetTickCount();

      // Throttle the actual LVGL text updates to max every 50ms.
      static TickType_t last_jog_render = 0;
      if (now - last_jog_render >= pdMS_TO_TICKS(50) ||
          lv_obj_has_flag(ov_jog, LV_OBJ_FLAG_HIDDEN))
      {
        last_jog_render = now;
        // Only extend the expiry when we actually render.
        transient_until = now + pdMS_TO_TICKS(DISPLAY_TRANSIENT_MS);
        printf("[disp] JOG render: axis=%d pos=%s spd=%d cont=%d\n",
            inst->jog_overlay.axis, inst->jog_overlay.pos_text,
            inst->jog_overlay.speed_level, inst->jog_overlay.continuous);
        lv_label_set_text_static(lbl_jog_axis, inst->jog_overlay.top_text);
        lv_label_set_text_static(lbl_jog_pos,  inst->jog_overlay.pos_text);

        int8_t spd = inst->jog_overlay.speed_level;

        if (inst->jog_overlay.continuous) {
          // Continuous/shuttle: static bar display.
          jog_anim_active = false;
          int mag = (spd < 0) ? -spd : spd;
          for (int i = 0; i < SEG_COUNT; i++) {
            if (spd > 0 && i < mag) {
              uint8_t brt = 0x30 + (i * (0xFF - 0x30)) / (SEG_COUNT - 1);
              lv_obj_set_style_bg_color(jog_seg_right[i], lv_color_make(brt, brt, brt), LV_PART_MAIN);
              lv_obj_remove_flag(jog_seg_right[i], LV_OBJ_FLAG_HIDDEN);
            } else {
              lv_obj_add_flag(jog_seg_right[i], LV_OBJ_FLAG_HIDDEN);
            }
            if (spd < 0 && i < mag) {
              uint8_t brt = 0x30 + (i * (0xFF - 0x30)) / (SEG_COUNT - 1);
              lv_obj_set_style_bg_color(jog_seg_left[i], lv_color_make(brt, brt, brt), LV_PART_MAIN);
              lv_obj_remove_flag(jog_seg_left[i], LV_OBJ_FLAG_HIDDEN);
            } else {
              lv_obj_add_flag(jog_seg_left[i], LV_OBJ_FLAG_HIDDEN);
            }
          }
        } else if (spd != 0) {
          // Incremental jog: flash all 7 segments in the click direction.
          jog_anim_active = true;
          jog_anim_dir    = spd;  // +1 or -1
          jog_anim_start  = now;
          // Light up all segments in the direction instantly.
          lv_obj_t **segs     = (spd > 0) ? jog_seg_right : jog_seg_left;
          lv_obj_t **off_segs = (spd > 0) ? jog_seg_left  : jog_seg_right;
          for (int i = 0; i < SEG_COUNT; i++) {
            uint8_t brt = 0x30 + (i * (0xFF - 0x30)) / (SEG_COUNT - 1);
            lv_obj_set_style_bg_color(segs[i], lv_color_make(brt, brt, brt), LV_PART_MAIN);
            lv_obj_remove_flag(segs[i], LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(off_segs[i], LV_OBJ_FLAG_HIDDEN);
          }
        } else {
          // Incremental jog with no direction (shouldn't happen, but clear bar).
          jog_anim_active = false;
          for (int i = 0; i < SEG_COUNT; i++) {
            lv_obj_add_flag(jog_seg_left[i],  LV_OBJ_FLAG_HIDDEN);
            lv_obj_add_flag(jog_seg_right[i], LV_OBJ_FLAG_HIDDEN);
          }
        }

        hide_all_overlays();
        lv_obj_remove_flag(ov_jog, LV_OBJ_FLAG_HIDDEN);
        active_overlay = ov_jog;
      }
    }

    // ----------------------------------------------------------
    // Incremental jog flash tick.
    // All 7 segments flash on instantly, then snap off after JOG_FLASH_MS.
    // ----------------------------------------------------------
    if (jog_anim_active && !lv_obj_has_flag(ov_jog, LV_OBJ_FLAG_HIDDEN))
    {
      TickType_t elapsed = xTaskGetTickCount() - jog_anim_start;
      if (elapsed >= pdMS_TO_TICKS(JOG_FLASH_MS)) {
        // Flash expired — hide all segments.
        jog_anim_active = false;
        for (int i = 0; i < SEG_COUNT; i++) {
          lv_obj_add_flag(jog_seg_left[i],  LV_OBJ_FLAG_HIDDEN);
          lv_obj_add_flag(jog_seg_right[i], LV_OBJ_FLAG_HIDDEN);
        }
      }
    }

    if (got_cmd)
    {
      switch (cmd.cmd)
      {
        // ----------------------------------------------------------
        case DISPLAY_CMD_DISCONNECTED:
        {
          connected = false;
          in_estop  = false;
          transient_until = 0;
          lv_obj_remove_flag(wait_panel,  LV_OBJ_FLAG_HIDDEN);
          lv_obj_add_flag(estop_panel,    LV_OBJ_FLAG_HIDDEN);
          hide_all_overlays();
          active_overlay = nullptr;
          lv_obj_set_style_line_opa(divider, LV_OPA_TRANSP, LV_PART_MAIN);
          lv_label_set_text(lbl_power,  "");
          lv_label_set_text(lbl_mode,   "");
          lv_label_set_text(lbl_interp, "");
          lv_label_set_text(lbl_homed,  "");
          lv_label_set_text(lbl_feed,   "");
          lv_label_set_text(lbl_rapid,  "");
          lv_label_set_text(lbl_vel,    "");
          break;
        }

        // ----------------------------------------------------------
        case DISPLAY_CMD_ESTOP:
        {
          ms = cmd.state;
          in_estop = cmd.state.estop;
          transient_until = 0;
          lv_obj_add_flag(wait_panel, LV_OBJ_FLAG_HIDDEN);
          hide_all_overlays();
          active_overlay = nullptr;
          if (in_estop) {
            lv_obj_remove_flag(estop_panel, LV_OBJ_FLAG_HIDDEN);
            lv_anim_start(&estop_anim);
          } else {
            lv_anim_delete(lbl_estop, estop_anim_cb);
            lv_obj_set_style_text_opa(lbl_estop, LV_OPA_COVER, LV_PART_MAIN);
            lv_obj_add_flag(estop_panel, LV_OBJ_FLAG_HIDDEN);
          }
          if (!in_estop) goto update_status_row;
          break;
        }

        // ----------------------------------------------------------
        case DISPLAY_CMD_MACHINE_STATE:
        {
          ms = cmd.state;
          connected = true;
          // DISPLAY_CMD_MACHINE_STATE is only dispatched when estop == false,
          // so unconditionally clear the estop panel here in case we're coming
          // out of an estop condition.
          in_estop = false;
          lv_obj_add_flag(estop_panel, LV_OBJ_FLAG_HIDDEN);
          goto update_status_row;
        }

        // ----------------------------------------------------------
        case DISPLAY_CMD_KNOB:
        {
          if (!connected) break;
          static const char *knob_letters[] = { "F", "R", "V" };
          const char *letter = (cmd.knob_index < 3) ? knob_letters[cmd.knob_index] : "?";
          char val[32];
          if      (cmd.knob_index == 0) fmt_feed (val, sizeof(val), cmd.knob_seg);
          else if (cmd.knob_index == 1) fmt_rapid(val, sizeof(val), cmd.knob_seg);
          else                          fmt_vel  (val, sizeof(val), cmd.knob_seg);
          // Only extend the expiry when the displayed value changes,
          // so encoder settling noise can't keep the overlay stuck.
          static uint8_t last_knob_index = 0xFF;
          static uint8_t last_knob_seg   = 0xFF;
          bool changed = (cmd.knob_index != last_knob_index ||
                          cmd.knob_seg   != last_knob_seg);
          if (changed) {
            last_knob_index = cmd.knob_index;
            last_knob_seg   = cmd.knob_seg;
            transient_until = xTaskGetTickCount() + pdMS_TO_TICKS(DISPLAY_TRANSIENT_MS);
          }
          lv_label_set_text(lbl_knob_letter, letter);
          lv_label_set_text(lbl_knob_val, val);
          hide_all_overlays();
          lv_obj_remove_flag(ov_knob, LV_OBJ_FLAG_HIDDEN);
          active_overlay = ov_knob;
          break;
        }

        // DISPLAY_CMD_JOG: handled via jog_overlay dirty-flag polling (above).

        // ----------------------------------------------------------
        case DISPLAY_CMD_KEY:
        {
#ifdef DISPLAY_DEBUG_KEYS
          if (!connected) break;
          if (!cmd.key_press) break;  // only show on press, not release
          transient_until = xTaskGetTickCount() + pdMS_TO_TICKS(DISPLAY_TRANSIENT_MS);
          // Reuse select overlay for debug key display.
          lv_label_set_text_fmt(lbl_select_title, LV_SYMBOL_KEYBOARD " KEY");
          lv_label_set_text_fmt(lbl_select_val, "0x%02X", cmd.key_code);
          hide_all_overlays();
          lv_obj_remove_flag(ov_select, LV_OBJ_FLAG_HIDDEN);
          active_overlay = ov_select;
#endif
          break;
        }

        // ----------------------------------------------------------
        case DISPLAY_CMD_JOG_SELECT:
        {
          if (!connected) break;
          char top_buf[32], bot_buf[24];
          if (cmd.jog_select_is_axis) {
            static const char *const axis_names[3] = { "X", "Y", "Z" };
            int sel = (cmd.jog_axis >= 1 && cmd.jog_axis <= 3) ? cmd.jog_axis - 1 : 0;
            snprintf(top_buf, sizeof(top_buf), "Jog Axis");
            snprintf(bot_buf, sizeof(bot_buf), "%s", axis_names[sel]);
          } else {
            static const char *const inc_names[3] = { "0.010", "0.001", "0.0001" };
            int sel = (cmd.jog_increment >= 1 && cmd.jog_increment <= 3) ? cmd.jog_increment - 1 : 0;
            snprintf(top_buf, sizeof(top_buf), "Jog Increment");
            snprintf(bot_buf, sizeof(bot_buf), "%s", inc_names[sel]);
          }
          transient_until = xTaskGetTickCount() + pdMS_TO_TICKS(DISPLAY_TRANSIENT_MS);
          lv_label_set_text(lbl_select_title, top_buf);
          lv_label_set_text(lbl_select_val, bot_buf);
          hide_all_overlays();
          lv_obj_remove_flag(ov_select, LV_OBJ_FLAG_HIDDEN);
          active_overlay = ov_select;
          break;
        }

        // ----------------------------------------------------------
        case DISPLAY_CMD_BOOTSEL:
        {
          // Show message on OLED before MCU resets into BOOTSEL.
          // Hide everything else and display the text permanently.
          transient_until = 0;
          lv_obj_add_flag(wait_panel,   LV_OBJ_FLAG_HIDDEN);
          lv_obj_add_flag(estop_panel,  LV_OBJ_FLAG_HIDDEN);
          lv_label_set_text(lbl_boot_title, LV_SYMBOL_USB "  USB Boot");
          lv_label_set_text(lbl_boot_text, cmd.bootsel_text);
          hide_all_overlays();
          lv_obj_remove_flag(ov_bootsel, LV_OBJ_FLAG_HIDDEN);
          active_overlay = ov_bootsel;
          // Force immediate render so the OLED gets updated before reset.
          lv_timer_handler();
          break;
        }
      }
    }
    // Debug heartbeat — once per second, verify gui_task is alive.
    {
      static TickType_t last_hb = 0;
      TickType_t now_hb = xTaskGetTickCount();
      if (now_hb - last_hb >= pdMS_TO_TICKS(1000)) {
        last_hb = now_hb;
        UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
        printf("[disp] heartbeat tick=%lu until=%lu stk=%lu jog=%d knob=%d sel=%d boot=%d\n",
            (unsigned long)now_hb, (unsigned long)transient_until,
            (unsigned long)hwm,
            !lv_obj_has_flag(ov_jog, LV_OBJ_FLAG_HIDDEN),
            !lv_obj_has_flag(ov_knob, LV_OBJ_FLAG_HIDDEN),
            !lv_obj_has_flag(ov_select, LV_OBJ_FLAG_HIDDEN),
            !lv_obj_has_flag(ov_bootsel, LV_OBJ_FLAG_HIDDEN));
      }
    }

    lv_timer_handler_run_in_period(5);
    continue;

    // ----------------------------------------------------------
    // Status row update (jumped to by MACHINE_STATE / ESTOP-clear)
    // ----------------------------------------------------------
    update_status_row:
    {
      // Row 1: power icon, mode, interp state, homed.
      // When the machine is not enabled, suppress mode/interp and show MACHINE OFF.
      const char *mode_str;
      const char *interp_str;

      if (!ms.enabled) {
        mode_str   = "OFF";
        interp_str = "";
      } else {
        switch (ms.mode) {
          case MODE_MANUAL: mode_str = "MAN";  break;
          case MODE_AUTO:   mode_str = "AUTO"; break;
          case MODE_MDI:    mode_str = "MDI";  break;
          default:          mode_str = "---";  break;
        }
        switch (ms.interp_state) {
          case INTERP_IDLE:    interp_str = "IDLE";  break;
          case INTERP_READING: interp_str = ms.step_mode ? "STEP" : "RUN"; break;
          case INTERP_PAUSED:  interp_str = ms.step_mode ? "STEP" : "PAUSE"; break;
          case INTERP_WAITING: interp_str = ms.step_mode ? "STEP" : "RUN";   break;
          default:             interp_str = "OFF";   break;
        }
      }

      // Row 2: override values
      char feed_buf[24], rapid_buf[24], vel_buf[24];
      fmt_feed (feed_buf,  sizeof(feed_buf),  ms.feed_seg);
      fmt_rapid(rapid_buf, sizeof(rapid_buf), ms.rapid_seg);
      fmt_vel  (vel_buf,   sizeof(vel_buf),   ms.maxvel_seg);

      lv_obj_add_flag(wait_panel,  LV_OBJ_FLAG_HIDDEN);
      lv_obj_add_flag(estop_panel, LV_OBJ_FLAG_HIDDEN);
      lv_obj_set_style_line_opa(divider, LV_OPA_COVER, LV_PART_MAIN);

      // When disabled, mode label carries the MACHINE OFF text (with icon); hide
      // the dedicated power label to avoid double-icon clutter.
      lv_obj_set_style_text_color(lbl_mode,
          ms.enabled ? lv_color_white() : lv_color_make(0x80, 0x80, 0x80),
          LV_PART_MAIN);
      lv_obj_add_flag(lbl_power, LV_OBJ_FLAG_HIDDEN);  // power icon folded into lbl_mode
      lv_label_set_text(lbl_mode,   mode_str);
      lv_label_set_text(lbl_interp, interp_str);
      lv_label_set_text(lbl_homed, LV_SYMBOL_HOME);
      if (ms.homed) {
        lv_obj_set_style_text_color(lbl_homed, lv_color_white(), LV_PART_MAIN);
        lv_obj_add_flag(slash_homed, LV_OBJ_FLAG_HIDDEN);
      } else {
        lv_obj_set_style_text_color(lbl_homed, lv_color_make(0x30, 0x30, 0x30), LV_PART_MAIN);
        lv_obj_remove_flag(slash_homed, LV_OBJ_FLAG_HIDDEN);
      }
      // Status row needs F:/R:/V: prefixes (overlay shows them as big letters).
      char feed_disp[28], rapid_disp[28], vel_disp[28];
      snprintf(feed_disp,  sizeof(feed_disp),  "F:%s", feed_buf);
      snprintf(rapid_disp, sizeof(rapid_disp), "R:%s", rapid_buf);
      snprintf(vel_disp,   sizeof(vel_disp),   "V:%s", vel_buf);
      lv_label_set_text(lbl_feed,  feed_disp);
      lv_label_set_text(lbl_rapid, rapid_disp);
      lv_label_set_text(lbl_vel,   vel_disp);
    }

    lv_timer_handler_run_in_period(5);
  }
}
