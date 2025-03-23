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
#include "spi.hh"
#include "oled_sh1122.hh"
#include "i2c.hh"
#include "encoder.hh"
#include "tca8418.hh"
#include "tlc59116.hh"
#include "lvgl.h"

#define PIN_PERIPH_RESETN 22
#define PIN_KEY_INT 19

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

int main(void)
{
  Encoder encoders;
  I2C i2c;
  TCA8418 matx(i2c);
  TLC59116 leds(i2c);
  SPI spi;
  OLED oled(spi);

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

  printf("SPI init...");
  spi.init();
  printf("OK\n");

  printf("OLED init...");
  oled.init();
  printf("OK\n");

  printf("LVGL init...");

  lv_init();

  lv_display_t* display = lv_display_create(SH1122_HOR_RES, SH1122_VER_RES);
  lv_display_set_color_format(display, LV_COLOR_FORMAT_I1);

  static uint8_t buf1[(SH1122_HOR_RES * SH1122_VER_RES / 8) + 8];
  lv_display_set_buffers(display, buf1, NULL, sizeof(buf1), LV_DISPLAY_RENDER_MODE_FULL);
  lv_display_set_user_data(display, &oled);
  lv_display_set_flush_cb(display, [](lv_display_t * display, const lv_area_t *area, uint8_t* px_map) {
      OLED* oled = static_cast<OLED*>(lv_display_get_user_data(display));
      oled->lv_sh1122_flush_cb(display, area, px_map);
  });
  lv_display_set_default(display);
  lv_display_set_antialiasing(display, false);
  

  printf("OK\n");

  printf("I2C init...");
  i2c.init();
  printf("OK\n");

  printf("Matrix keypad init...");
  matx.init();
  printf("OK\n");
  printf("Matrix keypad config...");
  matx.matrix(7, 8);
  matx.enableDebounce();
  matx.enableInterrupts();
  printf("OK\n");

  printf("LED driver init...");
  leds.init();
  printf("OK\n");

#ifdef LED_TEST
  printf("LED test...");
  for (uint8_t led = 0; led < 12; led++)
  {
    for (uint8_t bright = 0; bright < 0x7f; bright++)
    {
      leds.setLED(led, bright);
    }
  }
  board_delay(500);
  for (uint8_t bright = 0x7f; bright > 0; bright--)
  {
    for (uint8_t led = 0; led < 12; led++)
    {
      leds.setLED(led, bright, false);
    }
    leds.update();
    board_delay(10);
  }
  printf("OK\n");
#endif

  printf("Encoder init...");
  encoders.init();
  printf("OK\n");

  gpio_init(PIN_KEY_INT);
  gpio_set_dir(PIN_KEY_INT, GPIO_IN);
  gpio_pull_up(PIN_KEY_INT);

  printf("System initialized.\n");

  lv_obj_set_style_text_color(lv_screen_active(), lv_color_hsv_to_rgb(0,0,255), LV_PART_MAIN);
  lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hsv_to_rgb(0,0,0), LV_PART_MAIN);
  lv_obj_set_style_text_font(lv_screen_active(), &lv_font_montserrat_28, LV_PART_MAIN);

  lv_obj_t * label = lv_label_create(lv_screen_active());
  lv_label_set_text(label, "PICO LVGL!");
  
  lv_obj_set_pos(label, 0, 0);
  //lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
  
  lv_obj_t * label2 = lv_label_create(lv_screen_active());
  lv_obj_set_pos(label2, 0, 32);

  printf("wrote LVGL string\n");

  uint32_t time_till_next = lv_timer_handler();
  
  static absolute_time_t last_time = get_absolute_time();
  absolute_time_t since_last = last_time;
  absolute_time_t now = last_time;

  while (1)
  {
    absolute_time_t now = get_absolute_time();

    int64_t elapsed_time_us = absolute_time_diff_us(last_time, now);
    int64_t elapsed_time_ms = elapsed_time_us / 1000;

    int64_t since_last_us = absolute_time_diff_us(since_last, now);
    int64_t since_last_ms = since_last_us / 1000;
    if (since_last_ms >= 20) {
      lv_tick_inc(since_last_ms);
      time_till_next = lv_timer_handler();
      since_last = now;
    }

    last_time = now;


    tud_task();
    led_blinking_task();

    if (encoders.task())
    {
      printf("shuttle=%d %d\n", encoders.value(4), encoders.value(0));

      char s[80];
      snprintf(s, sizeof(s), "%-05d %-05d", encoders.value(4), encoders.value(0));
      lv_label_set_text(label2, s);

      //snprintf(s, sizeof(s), "%-05d %-05d", encoders.value(4), encoders.value(0));
      //oled.DrawString(0, 16, s);
      //snprintf(s, sizeof(s), "%-05d %-05d %-05d", encoders.value(1), encoders.value(2), encoders.value(3));
      //oled.DrawString(0, 32, s);
      //char s[80];
      //snprintf(s, sizeof(s), "%-05d %-05d", encoders.value(4), encoders.value(0));

      pkt.s.knob1 = encoders.value(0);
      pkt.s.knob2 = encoders.value(1);
      pkt.s.knob3 = encoders.value(2);
      pkt.s.button1 = 0;
      pkt.s.button2 = 0;
      pkt.s.button3 = 0;

      tud_hid_report(0, &pkt, sizeof(pkt));
    }

    if (gpio_get(PIN_KEY_INT) == 0)
    {
      if (matx.available())
      {
        uint8_t evt = matx.getEvent();
        uint8_t key = evt & 0x7F;
        bool pressed = evt & 0x80;

        printf("key event: %x %s\n", evt & 0x7F, evt & 0x80 ? "press" : "release");

        //snprintf(s, sizeof(s), "KEY %02X %s", evt & 0x7F, evt & 0x80 ? "PRESSED" : "RELEASE");
        //oled.DrawString(0, 48, s);


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
