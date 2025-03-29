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
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include "bsp/board_api.h"
#include "pico/time.h"
#include "tusb.h"
#include "i2c.hh"
#include "encoder.hh"
#include "tca8418.hh"
#include "tlc59116.hh"
#include "ws2812.hh"

#define PIN_PERIPH_RESETN 22

enum
{
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void led_task(void *unused_arg);
void matrix_task(void *unused_arg);
void encoder_task(void *unused_arg);
void main_task(void *unused_arg);

I2C i2c;
Encoder encoders;
WS2812 rgbleds;
TLC59116 leds;
TCA8418 matrix;

QueueHandle_t led_cmd_queue;
QueueHandle_t matrix_event_queue;
QueueHandle_t encoder_event_queue;
QueueHandle_t encoder_cmd_queue;

typedef enum
{
  LED_CMD_SET_SIMPLE_LED,
  LED_CMD_SET_RGB_LED,
  LED_CMD_SET_RING,
} led_cmds;

typedef struct led_cmd
{
  led_cmds cmd;
  uint8_t led;
  uint8_t value;
  uint32_t color;
  bool update_now;
} led_cmd_t;


typedef struct matrix_event
{
  uint8_t code;
  bool press;
  bool gpio;
  bool err;
} matrix_event_t;

typedef struct encoder_event
{
  uint8_t encoder;
  int8_t value;
} encoder_event_t;

typedef enum {
  ENCODER_CMD_SET_VALUE,
} encoder_cmds;

typedef struct encoder_cmd
{
  encoder_cmds cmd;
  uint8_t encoder;
  int8_t value;
} encoder_cmd_t;


int main(void)
{
  board_init();
  /*
  tud_init(BOARD_TUD_RHPORT);

  if (board_init_after_tusb)
    board_init_after_tusb();
    */

  printf("peripheral reset...");
  gpio_init(PIN_PERIPH_RESETN);
  gpio_set_dir(PIN_PERIPH_RESETN, GPIO_OUT);
  gpio_pull_up(PIN_PERIPH_RESETN);
  gpio_put(PIN_PERIPH_RESETN, 0);
  board_delay(100);
  gpio_put(PIN_PERIPH_RESETN, 1);
  printf("OK\n");

  i2c.init();
  encoders.init();
  rgbleds.init();
  leds.init(&i2c);
  matrix.init(&i2c);

  TaskHandle_t led_task_handle;
  TaskHandle_t matrix_task_handle;
  TaskHandle_t encoder_task_handle;
  TaskHandle_t main_task_handle;

  BaseType_t matrix_task_status = xTaskCreate(matrix_task, "MATRIX_TASK", 1024, nullptr, 3, &matrix_task_handle);
  BaseType_t encoder_task_status = xTaskCreate(encoder_task, "ENCODER_TASK", 1024, nullptr, 1, &encoder_task_handle);
  BaseType_t led_task_status = xTaskCreate(led_task, "LED_TASK", 1024, nullptr, 2, &led_task_handle);
  BaseType_t main_task_status = xTaskCreate(main_task, "MAIN_TASK", 1024, nullptr, 2, &main_task_handle);

  if (led_task_status != pdPASS || matrix_task_status != pdPASS || encoder_task_status != pdPASS)
  {
    printf("Failed to create tasks:\n");
    printf("LED_TASK: %d\n", led_task_status);
    printf("MATRIX_TASK: %d\n", matrix_task_status);
    printf("ENCODER_TASK: %d\n", encoder_task_status);
    return 1;
  }

  led_cmd_queue = xQueueCreate(10, sizeof(led_cmd_t));
  matrix_event_queue = xQueueCreate(10, sizeof(matrix_event_t));
  encoder_event_queue = xQueueCreate(10, sizeof(encoder_event_t));
  encoder_cmd_queue = xQueueCreate(10, sizeof(encoder_cmd_t));

  vTaskStartScheduler();

  while (true) {
    // forever
  }

}


void main_task(void* unused)
{
  const uint8_t key_to_led[] = {
      0x2f, 0x30, 0x37, 0x38, 0x39, 0x3a, 0x3d, 0x3e, 0x33, 0x34, 0x35, 0x36};

  const uint32_t dot_colors[] = {
      rgbleds.urgb_u32(0x7f, 0, 0),
      rgbleds.urgb_u32(0, 0x7f, 0),
      rgbleds.urgb_u32(0, 0, 0x7f),
  };

  while (true)
  {
    encoder_event_t enc_evt;
    if (xQueueReceive(encoder_event_queue, &enc_evt, 10) == pdTRUE)
    {
      printf("encoder=%d value=%d\n", enc_evt.encoder, enc_evt.value);

      if (enc_evt.encoder >= 1 && enc_evt.encoder <= 3) {
        led_cmd_t cmd;
        cmd.cmd = LED_CMD_SET_RING;
        cmd.led = enc_evt.encoder - 1;
        cmd.value = enc_evt.value;
        cmd.color = dot_colors[enc_evt.encoder - 1];
        cmd.update_now = true;
        xQueueSend(led_cmd_queue, &cmd, 0);
      }
    }

    matrix_event_t mtx_evt;
    if (xQueueReceive(matrix_event_queue, &mtx_evt, 10) == pdTRUE)
    {
      printf("evt=%02x %s %s\n", mtx_evt.code, mtx_evt.press ? "press" : "release", mtx_evt.gpio ? "gpio" : "key");

      if (mtx_evt.gpio && mtx_evt.press) {
        uint8_t enc = 0xff;
        if (mtx_evt.code == 0x68) {
          enc = 1;
        } else if (mtx_evt.code == 0x71) {
          enc = 2;
        } else if (mtx_evt.code == 0x72) {
          enc = 3;
        }

        if (enc != 0xff) {
          encoder_cmd_t cmd;
          cmd.cmd = ENCODER_CMD_SET_VALUE;
          cmd.encoder = enc;
          cmd.value = 0;
          xQueueSend(encoder_cmd_queue, &cmd, 0);
        } 
        
      } else {
        uint8_t led = 0;
        for (int i = 0; i < 12; i++) {
          if (mtx_evt.code == key_to_led[i]) {
            led = i + 1;
            break;
          }
        }
        printf("led=%d\n", led);
        if (led > 0 && led <= 16)
        {
          led_cmd_t cmd;
          cmd.cmd = LED_CMD_SET_SIMPLE_LED;
          cmd.led = led - 1;
          cmd.value = mtx_evt.press ? 64 : 0;
          cmd.update_now = true;
          xQueueSend(led_cmd_queue, &cmd, 0);
        }
      }

    }
  // tud_task();
  }

}

void led_task(void *unused)
{
  bool state = true;
  while (true)
  {
    led_cmd_t cmd;
    if (xQueueReceive(led_cmd_queue, &cmd, 250) == pdTRUE)
    {
      switch (cmd.cmd)
      {
      case LED_CMD_SET_SIMPLE_LED:
        leds.setLED(cmd.led, cmd.value, cmd.update_now);
        break;
      case LED_CMD_SET_RGB_LED:
        rgbleds.setLED(cmd.led, cmd.value, cmd.update_now);
        break;
      case LED_CMD_SET_RING:
        rgbleds.setRing(cmd.led, cmd.value, cmd.color, cmd.update_now);
        break;
      }
    }

    if (state)
    {
      board_led_write(true);
    }
    else
    {
      board_led_write(false);
    }
    state = !state;
  }
}

void matrix_task(void *unused)
{
  while (true)
  {
    uint8_t avail = matrix.available();
    if (avail)
    {
      if (avail & 0x80)
      {
        matrix_event_t qevt;
        qevt.err = true;
        qevt.code = 0;
        xQueueSend(matrix_event_queue, &qevt, 0);
      }

      uint8_t evt = matrix.getEvent();

      matrix_event_t qevt;
      qevt.code = evt & 0x7F;
      qevt.press = evt & 0x80;
      qevt.gpio = qevt.code > 0x5B;
      qevt.err = false;
      xQueueSend(matrix_event_queue, &qevt, 0);
    }
    vTaskDelay(10);
  }
}

void encoder_task(void *unused_arg)
{
  bool force_encoder_update = false;

  encoders.set_limits(0, 0, 0, 1);
  encoders.set_limits(1, 0, 14, 4);
  encoders.set_limits(2, 0, 14, 4);
  encoders.set_limits(3, 0, 14, 4);

  int last_value[5] = {-1, -1, -1, -1, -1};

  while (true)
  {
    encoder_cmd_t cmd;
    if (xQueueReceive(encoder_cmd_queue, &cmd, 250) == pdTRUE)
    {
      if (cmd.cmd == ENCODER_CMD_SET_VALUE)
      {
        encoders.set_value(cmd.encoder, cmd.value);
        force_encoder_update = true;
      }
    }

    bool upd = encoders.task();

    if (upd || force_encoder_update)
    {
      force_encoder_update = false;

      for (int i = 1; i < 5; i++)
      {
        if (encoders.value(i) != last_value[i])
        {
          encoder_event_t evt;
          evt.encoder = i;
          evt.value = encoders.value(i);
          xQueueSend(encoder_event_queue, &evt, 0);
          last_value[i] = encoders.value(i);
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


      /*
      pkt.s.knob1 = encoders.value(0);
      pkt.s.knob2 = encoders.value(1);
      pkt.s.knob3 = encoders.value(2);
      pkt.s.button1 = 0;
      pkt.s.button2 = 0;
      pkt.s.button3 = 0;

      tud_hid_report(0, &pkt, sizeof(pkt));
      */

/*
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
*/

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
