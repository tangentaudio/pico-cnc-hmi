#include <stdio.h>
#include <FreeRTOS.h>
#include "semphr.h"
#include <task.h>
#include "spi.hh"
#include "oled_sh1122.hh"
#include "lvgl.h"
#include "task_display.hh"

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
  // Power / machine-enabled indicator (left)
  lv_obj_t *lbl_power = lv_label_create(scr);
  lv_obj_set_style_text_font(lbl_power, &lv_font_montserrat_18, LV_PART_MAIN);
  lv_obj_set_pos(lbl_power, 2, 5);
  lv_label_set_text(lbl_power, "");

  // Mode (MAN / AUTO / MDI)
  lv_obj_t *lbl_mode = lv_label_create(scr);
  lv_obj_set_style_text_font(lbl_mode, &lv_font_montserrat_14, LV_PART_MAIN);
  lv_obj_align(lbl_mode, LV_ALIGN_TOP_LEFT, 3, 8);
  lv_label_set_text(lbl_mode, "");

  // Interp state (IDLE / RUN / PAUSE / STEP) — fixed-width centred in row 1
  lv_obj_t *lbl_interp = lv_label_create(scr);
  lv_obj_set_style_text_font(lbl_interp, &lv_font_montserrat_18, LV_PART_MAIN);
  lv_obj_set_style_text_align(lbl_interp, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN);
  lv_obj_set_width(lbl_interp, 90);
  lv_obj_align(lbl_interp, LV_ALIGN_TOP_MID, 0, 5);
  lv_label_set_text(lbl_interp, "");

  // Homed indicator — LV_SYMBOL_HOME icon, right side of row 1
  lv_obj_t *lbl_homed = lv_label_create(scr);
  lv_obj_set_style_text_font(lbl_homed, &lv_font_montserrat_20, LV_PART_MAIN);
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
  lv_obj_t *lbl_feed = lv_label_create(scr);
  lv_obj_set_style_text_font(lbl_feed, &lv_font_montserrat_16, LV_PART_MAIN);
  lv_obj_set_pos(lbl_feed, 2, MID + 6);
  lv_label_set_text(lbl_feed, "");

  lv_obj_t *lbl_rapid = lv_label_create(scr);
  lv_obj_set_style_text_font(lbl_rapid, &lv_font_montserrat_16, LV_PART_MAIN);
  lv_obj_set_pos(lbl_rapid, 84, MID + 6);
  lv_label_set_text(lbl_rapid, "");

  lv_obj_t *lbl_vel = lv_label_create(scr);
  lv_obj_set_style_text_font(lbl_vel, &lv_font_montserrat_16, LV_PART_MAIN);
  lv_obj_set_pos(lbl_vel, 162, MID + 6);
  lv_label_set_text(lbl_vel, "");

  // ---------------------------------------------------------------
  // Transient overlay panel (full screen, hidden by default)
  // ---------------------------------------------------------------
  lv_obj_t *overlay = lv_obj_create(scr);
  lv_obj_set_size(overlay, W, H);
  lv_obj_set_pos(overlay, 0, 0);
  lv_obj_set_style_bg_color(overlay, lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_bg_opa(overlay, LV_OPA_COVER, LV_PART_MAIN);
  lv_obj_set_style_border_width(overlay, 0, LV_PART_MAIN);
  lv_obj_set_style_pad_all(overlay, 0, LV_PART_MAIN);
  lv_obj_add_flag(overlay, LV_OBJ_FLAG_HIDDEN);

  lv_obj_t *lbl_overlay_top = lv_label_create(overlay);
  lv_obj_set_style_text_font(lbl_overlay_top, &lv_font_montserrat_28, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl_overlay_top, lv_color_white(), LV_PART_MAIN);
  lv_obj_align(lbl_overlay_top, LV_ALIGN_TOP_MID, 0, 2);
  lv_label_set_text(lbl_overlay_top, "");

  lv_obj_t *lbl_overlay_bot = lv_label_create(overlay);
  lv_obj_set_style_text_font(lbl_overlay_bot, &lv_font_montserrat_22, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl_overlay_bot, lv_color_white(), LV_PART_MAIN);
  lv_obj_align(lbl_overlay_bot, LV_ALIGN_BOTTOM_MID, 0, -2);
  lv_label_set_text(lbl_overlay_bot, "");

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
  lv_obj_set_style_text_font(lbl_estop, &lv_font_montserrat_28, LV_PART_MAIN);
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
  lv_obj_set_style_text_font(lbl_wait, &lv_font_montserrat_18, LV_PART_MAIN);
  lv_obj_set_style_text_color(lbl_wait, lv_color_white(), LV_PART_MAIN);
  lv_obj_align(lbl_wait, LV_ALIGN_CENTER, 0, 0);
  lv_label_set_text(lbl_wait, LV_SYMBOL_USB "  Waiting for connection...");

  LVGL_UNLOCK(inst->mutex);

  // ---------------------------------------------------------------
  // Helper lambdas to format override values using hmi_config
  // ---------------------------------------------------------------
  auto fmt_feed  = [](char *buf, size_t n, uint8_t seg) {
    if (!hmi_config.valid || hmi_config.max_feed_pct == 0) {
      snprintf(buf, n, "F: -%%");
    } else {
      snprintf(buf, n, "F: %u%%", hmi_feed_pct(seg));
    }
  };
  auto fmt_rapid = [](char *buf, size_t n, uint8_t seg) {
    if (!hmi_config.valid || hmi_config.max_rapid_pct == 0) {
      snprintf(buf, n, "R: -%%");
    } else {
      snprintf(buf, n, "R: %u%%", hmi_rapid_pct(seg));
    }
  };
  auto fmt_vel   = [](char *buf, size_t n, uint8_t seg) {
    if (!hmi_config.valid || hmi_config.max_vel_x10 == 0) {
      snprintf(buf, n, "V: - IPM");
    } else {
      uint16_t v = hmi_maxvel_x10(seg);
      snprintf(buf, n, "V: %u.%u IPM", v / 10, v % 10);
    }
  };

  // ---------------------------------------------------------------
  // State tracking
  // ---------------------------------------------------------------
  bool connected   = false;
  bool in_estop    = false;
  TickType_t transient_until = 0;   // non-zero = overlay visible until this tick

  machine_state_t ms = {};

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
      printf("[disp] overlay expired at tick %lu\n", (unsigned long)xTaskGetTickCount());
      transient_until = 0;
      lv_obj_add_flag(overlay, LV_OBJ_FLAG_HIDDEN);
    }
    else if (transient_until != 0)
    {
      printf("[disp] waiting: now=%lu until=%lu\n", (unsigned long)xTaskGetTickCount(), (unsigned long)transient_until);
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
          lv_obj_add_flag(overlay,        LV_OBJ_FLAG_HIDDEN);
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
          lv_obj_add_flag(overlay,    LV_OBJ_FLAG_HIDDEN);
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
          char top[32], bot[32];
          static const char *knob_names[] = { "Feed", "Rapid", "Max Velocity" };
          const char *name = (cmd.knob_index < 3) ? knob_names[cmd.knob_index] : "?";
          snprintf(top, sizeof(top), "%s", name);
          if      (cmd.knob_index == 0) fmt_feed (bot, sizeof(bot), cmd.knob_seg);
          else if (cmd.knob_index == 1) fmt_rapid(bot, sizeof(bot), cmd.knob_seg);
          else                          fmt_vel  (bot, sizeof(bot), cmd.knob_seg);
          transient_until = xTaskGetTickCount() + pdMS_TO_TICKS(DISPLAY_TRANSIENT_MS);
          lv_obj_set_style_text_font(lbl_overlay_top, &lv_font_montserrat_28, LV_PART_MAIN);
          lv_obj_set_style_text_font(lbl_overlay_bot, &lv_font_montserrat_22, LV_PART_MAIN);
          lv_label_set_text(lbl_overlay_top, top);
          lv_label_set_text(lbl_overlay_bot, bot);
          lv_obj_remove_flag(overlay, LV_OBJ_FLAG_HIDDEN);
          break;
        }

        // ----------------------------------------------------------
        case DISPLAY_CMD_JOG:
        {
          if (!connected) break;
          TickType_t now = xTaskGetTickCount();
          printf("[disp] JOG cmd: axis=%d cont=%d pos=%.4f  now=%lu until=%lu\n",
              cmd.jog_axis, (int)cmd.jog_continuous, (double)cmd.jog_pos[(cmd.jog_axis>=1&&cmd.jog_axis<=3)?cmd.jog_axis-1:0],
              (unsigned long)now, (unsigned long)transient_until);
          transient_until = now + pdMS_TO_TICKS(DISPLAY_TRANSIENT_MS);
          static const char *const axis_names[3] = { "X", "Y", "Z" };
          int sel = (cmd.jog_axis >= 1 && cmd.jog_axis <= 3) ? cmd.jog_axis - 1 : 0;
          char top_buf[32], bot_buf[24];
          snprintf(top_buf, sizeof(top_buf), "Jog %s %s",
              cmd.jog_continuous ? "Continuous" : "Incremental",
              axis_names[sel]);
          snprintf(bot_buf, sizeof(bot_buf), "%+.4f", (double)cmd.jog_pos[sel]);
          lv_obj_set_style_text_font(lbl_overlay_top, &lv_font_montserrat_18, LV_PART_MAIN);
          lv_obj_set_style_text_font(lbl_overlay_bot, &lv_font_unscii_16, LV_PART_MAIN);
          lv_label_set_text(lbl_overlay_top, top_buf);
          lv_label_set_text(lbl_overlay_bot, bot_buf);
          lv_obj_remove_flag(overlay, LV_OBJ_FLAG_HIDDEN);
          break;
        }

        // ----------------------------------------------------------
        case DISPLAY_CMD_KEY:
        {
#ifdef DISPLAY_DEBUG_KEYS
          if (!connected) break;
          if (!cmd.key_press) break;  // only show on press, not release
          transient_until = xTaskGetTickCount() + pdMS_TO_TICKS(DISPLAY_TRANSIENT_MS);
          lv_label_set_text_fmt(lbl_overlay_top, LV_SYMBOL_KEYBOARD " KEY");
          lv_label_set_text_fmt(lbl_overlay_bot, "0x%02X", cmd.key_code);
          lv_obj_remove_flag(overlay, LV_OBJ_FLAG_HIDDEN);
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
          lv_obj_set_style_text_font(lbl_overlay_top, &lv_font_montserrat_18, LV_PART_MAIN);
          lv_obj_set_style_text_font(lbl_overlay_bot, &lv_font_montserrat_28, LV_PART_MAIN);
          lv_label_set_text(lbl_overlay_top, top_buf);
          lv_label_set_text(lbl_overlay_bot, bot_buf);
          lv_obj_remove_flag(overlay, LV_OBJ_FLAG_HIDDEN);
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
          lv_obj_set_style_text_font(lbl_overlay_top, &lv_font_montserrat_18, LV_PART_MAIN);
          lv_obj_set_style_text_font(lbl_overlay_bot, &lv_font_montserrat_18, LV_PART_MAIN);
          lv_label_set_text(lbl_overlay_top, LV_SYMBOL_USB "  USB Boot");
          lv_label_set_text(lbl_overlay_bot, cmd.bootsel_text);
          lv_obj_remove_flag(overlay, LV_OBJ_FLAG_HIDDEN);
          // Force immediate render so the OLED gets updated before reset.
          lv_timer_handler();
          break;
        }
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
          case INTERP_PAUSED:  interp_str = "PAUSE"; break;
          case INTERP_WAITING: interp_str = "RUN";   break;
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
      lv_label_set_text(lbl_feed,  feed_buf);
      lv_label_set_text(lbl_rapid, rapid_buf);
      lv_label_set_text(lbl_vel,   vel_buf);
    }
    lv_timer_handler_run_in_period(5);
  }
}
