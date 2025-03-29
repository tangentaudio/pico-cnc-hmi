// PICO 2 GPIO MAP HW REV0
// =======================
// GPIO  0 - UART0 TX
// GPIO  1 - UART0 RX
// GPIO  2 - SPI0_SCK
// GPIO  3 - SPI0_DOUT
// GPIO  4 - LCD_A0
// GPIO  5 - SPI0_CS0
// GPIO  6 - JOG_A
// GPIO  7 - JOG_B
// GPIO  8 - SHUTTLE_0
// GPIO  9 - SHUTTLE_1
// GPIO 10 - SHUTTLE_2
// GPIO 11 - SHUTTLE_3
// GPIO 12 - ENC0_A
// GPIO 13 - ENC0_B
// GPIO 14 - ENC1_A
// GPIO 15 - ENC1_B
// GPIO 16 - ENC2_A
// GPIO 17 - ENC2_B
// GPIO 18 - SPARE
// GPIO 19 - /KEY_INT
// GPIO 20 - I2C0_SDA
// GPIO 21 - I2C0_SCL
// GPIO 22 - /PERIPH_RESET
// GPIO 26 - SPARE
// GPIO 27 - SPARE
// GPIO 28 - SPARE

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "bsp/board_api.h"
#include "pico/time.h"
#include "tusb.h"
#include "i2c.hh"
#include "encoder.hh"
#include "tca8418.hh"
#include "tlc59116.hh"
#include "ws2812.hh"
#ifdef ENABLE_DISPLAY
#include "spi.hh"
#include "oled_sh1122.hh"
#include "lvgl.h"
#endif


//#define LED_TEST

#define PIN_PERIPH_RESETN 22

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum
{
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void led_blinking_task(void);

union pkt_u
{
  struct pkt_s
  {
    int knob1;
    int knob2;
    int knob3;
    uint8_t button1;
    uint8_t button2;
    uint8_t button3;
  } s;
  unsigned char buf[64];
} pkt;



#ifdef ENABLE_DISPLAY
void align_area(lv_event_t *e) {
  auto *area = (lv_area_t *) lv_event_get_param(e);


  printf("align_area before %d %d %d %d  ", area->x1, area->y1, area->x2, area->y2);

  area->x1 &= ~3;
  area->x2 = ((area->x2 + 4) & ~3) - 1;
  printf("after %d %d %d %d\n", area->x1, area->y1, area->x2, area->y2);
}
#endif


int main(void)
{
  Encoder encoders;
  I2C i2c;
  TCA8418 matrix(i2c);
  TLC59116 leds(i2c);
  WS2812 rgbleds;
  #ifdef ENABLE_DISPLAY
  SPI spi;
  OLED oled(spi);
  #endif

  board_init();
  tud_init(BOARD_TUD_RHPORT);

  if (board_init_after_tusb)
  {
    board_init_after_tusb();
  }

  printf("peripheral reset...");
  gpio_init(PIN_PERIPH_RESETN);
  gpio_set_dir(PIN_PERIPH_RESETN, GPIO_OUT);
  gpio_pull_up(PIN_PERIPH_RESETN);
  gpio_put(PIN_PERIPH_RESETN, 0);
  board_delay(100);
  gpio_put(PIN_PERIPH_RESETN, 1);
  printf("OK\n");

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
  lv_display_t* display = lv_display_create(SH1122_HOR_RES, SH1122_VER_RES);
  lv_display_set_color_format (display, LV_COLOR_FORMAT);
  lv_display_set_user_data(display, &oled);

  lv_display_set_buffers(display, disp_buf1, NULL, sizeof(disp_buf1), LV_DISPLAY_RENDER_MODE_FULL);
  lv_display_set_flush_cb(display, [](lv_display_t * display, const lv_area_t *area, uint8_t* px_map) {
      OLED* oled = static_cast<OLED*>(lv_display_get_user_data(display));
      oled->lv_sh1122_flush_cb(display, area, px_map);
  });

  //lv_display_add_event_cb(display, align_area, LV_EVENT_INVALIDATE_AREA, nullptr);

  lv_display_set_default(display);
  //lv_display_set_antialiasing(display, true);

  printf("OK\n");
  #endif


  printf("I2C init...");
  i2c.init();
  printf("OK\n");

  printf("Matrix keypad init...");
  matrix.init();
  printf("OK\n");

  printf("PWM LED driver init...");
  leds.init();
  printf("OK\n");

  printf("RGB LED driver init...");
  rgbleds.init();
  printf("OK\n");

#ifdef LED_TEST
  printf("PWM LED test...");
  for (uint8_t led = 0; led < 12; led++)
  {
    for (uint8_t bright = 0; bright < 0x7f; bright++)
    {
      leds.setLED(led, bright);
    }
  }
  for (uint8_t led = 0; led < 12; led++)
  {
    for (uint8_t bright = 0x7f; bright > 0; bright--)
    {
      leds.setLED(led, bright);
    }
  }
  printf("OK\n");

  printf("RGB LED test...");
  for (uint8_t led = 0; led < WS2812_NUM_LEDS; led++)
  {
    rgbleds.setLED(led, rgbleds.urgb_u32(0x7f, 0, 0), true);
    board_delay(5);
  }
  for (uint8_t led = 0; led < WS2812_NUM_LEDS; led++)
  {
    rgbleds.setLED(led, rgbleds.urgb_u32(0, 0x7f, 0), true);
    board_delay(5);
  }
  for (uint8_t led = 0; led < WS2812_NUM_LEDS; led++)
  {
    rgbleds.setLED(led, rgbleds.urgb_u32(0, 0, 0x7f), true);
    board_delay(5);
  }
  board_delay(100);
  for (uint8_t led = 0; led < WS2812_NUM_LEDS; led++)
  {
    rgbleds.setLED(led, 0, true);
    board_delay(5);
  }
  printf("OK\n");
#endif



  printf("Encoder init...");
  encoders.init();
  encoders.set_limits(0, 0, 0, 1);
  encoders.set_limits(1, 0, 14, 4);
  encoders.set_limits(2, 0, 14, 4);
  encoders.set_limits(3, 0, 14, 4);
  printf("OK\n");


  #ifdef ENABLE_DISPLAY
  lv_obj_set_style_bg_color(lv_screen_active(), lv_color_black(), LV_PART_MAIN);
  lv_obj_set_style_text_font(lv_screen_active(), &lv_font_montserrat_28, LV_PART_MAIN);
  lv_obj_set_style_line_color(lv_screen_active(), lv_color_white(), LV_PART_MAIN);
  lv_obj_set_style_line_width(lv_screen_active(), 1, LV_PART_MAIN);

  lv_obj_t * labeljog = lv_label_create(lv_screen_active());
  lv_label_set_text(labeljog, "");
  lv_obj_set_pos(labeljog, 64, 0);

  lv_obj_t * labelkey = lv_label_create(lv_screen_active());
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

  lv_obj_t* barencoder[3];
  for (uint8_t i=0; i<3; i++) {
    lv_obj_t *barenc = lv_bar_create(lv_screen_active());
    lv_bar_set_orientation(barenc, LV_BAR_ORIENTATION_HORIZONTAL);
    lv_bar_set_range(barenc, 0, 14);
    lv_bar_set_mode(barenc, LV_BAR_MODE_NORMAL);
    lv_obj_remove_style_all(barenc);
    lv_obj_add_style(barenc, &style_bg, 0);
    lv_obj_add_style(barenc, &style_indic, LV_PART_INDICATOR);
    lv_bar_set_value(barenc, 0, LV_ANIM_OFF);
    lv_obj_set_size(barenc, 36, 20);
    lv_obj_set_pos(barenc, 128 + (38*i), 32);

    barencoder[i] = barenc;
  }

  uint32_t time_till_next = lv_timer_handler();
  
  static absolute_time_t last_time = get_absolute_time();
  absolute_time_t since_last = last_time;
  absolute_time_t now = last_time;
  static int t=0;
  #endif


  bool force_encoder_update = false;
  while (1)
  {
    #ifdef ENABLE_DISPLAY
    absolute_time_t now = get_absolute_time();

    int64_t elapsed_time_us = absolute_time_diff_us(last_time, now);
    int64_t elapsed_time_ms = elapsed_time_us / 1000;

    int64_t since_last_us = absolute_time_diff_us(since_last, now);
    int64_t since_last_ms = since_last_us / 1000;
    
    if (since_last_ms >= 33) {
      time_till_next = lv_timer_handler();

      lv_task_handler();
      lv_tick_inc(since_last_ms);

      since_last = now;
    }


    last_time = now;
    #endif


    tud_task();
    led_blinking_task();
    
    bool upd = encoders.task();

    if ( upd || force_encoder_update)
    {
      force_encoder_update = false;

      int shuttle = encoders.value(4);
      int jog = encoders.value(0);

      
      int v1 = encoders.value(1);
      rgbleds.setRing(0, v1, rgbleds.urgb_u32(0x7f, 0, 0), false);

      int v2 = encoders.value(2);
      rgbleds.setRing(1, v2, rgbleds.urgb_u32(0, 0x7f, 0), false);
      
      int v3 = encoders.value(3);
      rgbleds.setRing(2, v3, rgbleds.urgb_u32(0, 0, 0x7f), true);

      #ifdef ENABLE_DISPLAY
      lv_label_set_text_fmt(labeljog, "%5d", jog);
      lv_bar_set_value(barshuttle, shuttle, LV_ANIM_OFF);
      lv_bar_set_value(barencoder[0], v1, LV_ANIM_OFF);
      lv_bar_set_value(barencoder[1], v2, LV_ANIM_OFF);
      lv_bar_set_value(barencoder[2], v3, LV_ANIM_OFF);
      #endif

      printf("enc=%-05d %-05d %-05d\n", v1, v2, v3);

      pkt.s.knob1 = encoders.value(0);
      pkt.s.knob2 = encoders.value(1);
      pkt.s.knob3 = encoders.value(2);
      pkt.s.button1 = 0;
      pkt.s.button2 = 0;
      pkt.s.button3 = 0;

      tud_hid_report(0, &pkt, sizeof(pkt));
    }

    uint8_t avail = matrix.available();
    if (avail)
    {
      if (avail & 0x80) {
        // not really much we can do about it, just report it for debugging
        printf("key overflow\n");
      }

      uint8_t evt = matrix.getEvent();
      uint8_t key = evt & 0x7F;
      bool pressed = evt & 0x80;

      printf("%s event: %x %s\n", key > 0x5B ? "GPIO" : "key", key, pressed ? "press" : "release");

      #ifdef ENABLE_DISPLAY
      if (pressed)
        lv_label_set_text_fmt(labelkey, LV_SYMBOL_DOWN "%2.2X", key);
      else
        lv_label_set_text_fmt(labelkey, LV_SYMBOL_UP "%2.2X", key);
      #endif

      if (pressed)
      {
        uint8_t led = 0;
        switch (key)
        {

        case 0x2f:
          led = 1;
          break;
        case 0x30:
          led = 2;
          break;
        case 0x33:
          led = 9;
          break;
        case 0x34:
          led = 10;
          break;
        case 0x35:
          led = 11;
          break;
        case 0x36:
          led = 12;
          break;
        case 0x37:
          led = 3;
          break;
        case 0x38:
          led = 4;
          break;
        case 0x39:
          led = 5;
          break;
        case 0x3a:
          led = 6;
          break;
        case 0x3d:
          led = 7;
          break;
        case 0x3e:
          led = 8;
          break;
        case 0x68:
          encoders.set_value(1, 0);
          force_encoder_update = true;
          break;
        case 0x71:
          encoders.set_value(2, 0);
          force_encoder_update = true;
          break;
        case 0x72:
          encoders.set_value(3, 0);
          force_encoder_update = true;
          break;
        default:
          led = 0xff;
        }
        if (led > 0 && led <= 16)
        {
          bool cur = leds.getLED(led - 1) > 0;

          leds.setLED(led - 1, cur ? 0 : 64);
        }
      }
    }
  }
}


//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void)remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
  // TODO not Implemented
  (void)itf;
  (void)report_id;
  (void)report_type;
  (void)buffer;
  (void)reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
  // This example doesn't use multiple report and report ID
  (void)itf;
  (void)report_id;
  (void)report_type;

  // echo back anything we received from host
  // tud_hid_report(0, buffer, bufsize);
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void)
{
  static uint32_t start_ms = 0;
  static bool led_state = false;

  // Blink every interval ms
  if (board_millis() - start_ms < blink_interval_ms)
    return; // not enough time
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}
