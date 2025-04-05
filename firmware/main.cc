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
#include <boards/pico.h>
#include "pico/stdlib.h"
#include "drivers/i2c.hh"
#include "task_encoder.hh"
#include "task_matrix.hh"
#include "task_led.hh"
#ifdef ENABLE_DISPLAY
#include "task_display.hh"
#endif
#ifdef ENABLE_USB
#include "drivers/usb.h"
#include "tusb.h"
#endif

#define PIN_PERIPH_RESETN 22

I2C *i2c;
TaskMatrix *task_matrix;
TaskEncoder *task_encoder;
TaskLED *task_led;

#ifdef ENABLE_DISPLAY
TaskDisplay *task_display;
#endif

void main_task(void *unused);
void usb_task(void *unused);

int main(void)
{
  stdout_uart_init();

  gpio_init(PIN_PERIPH_RESETN);
  gpio_set_dir(PIN_PERIPH_RESETN, GPIO_OUT);
  gpio_pull_up(PIN_PERIPH_RESETN);
  gpio_put(PIN_PERIPH_RESETN, 0);
  sleep_ms(100);
  gpio_put(PIN_PERIPH_RESETN, 1);

#ifdef ENABLE_DISPLAY
  task_display = new TaskDisplay();
  task_display->init();
#endif

  i2c = new I2C();
  i2c->init();
  task_encoder = new TaskEncoder();
  task_encoder->init();

  task_matrix = new TaskMatrix();
  task_matrix->init(i2c);

  task_led = new TaskLED();
  task_led->init(i2c);

  BaseType_t matrix_task_status = xTaskCreate(task_matrix->task, "MATRIX_TASK", 256, (void *)task_matrix, 4, nullptr);
  BaseType_t encoder_task_status = xTaskCreate(task_encoder->task, "ENCODER_TASK", 256, (void *)task_encoder, 4, nullptr);
  BaseType_t led_task_status = xTaskCreate(task_led->task, "LED_TASK", 256, (void *)task_led, 2, nullptr);
#ifdef ENABLE_DISPLAY
  BaseType_t display_task1_status = xTaskCreate(task_display->timer_task, "DISPLAY_TASK_TIMER", 1024, (void *)task_display, 3, nullptr);
  BaseType_t display_task2_status = xTaskCreate(task_display->task_handler_task, "DISPLAY_TASK_HANDLER", 1024, (void *)task_display, 3, nullptr);
  BaseType_t display_gui_status = xTaskCreate(task_display->gui_task, "DISPLAY_GUI_TASK", 2048, (void *)task_display, 2, nullptr);
#endif
#ifdef ENABLE_USB
  BaseType_t usb_task_status = xTaskCreate(usb_task, "USB_TASK", 2048, nullptr, 1, nullptr);
#endif
  BaseType_t main_task_status = xTaskCreate(main_task, "MAIN_TASK", 2048, nullptr, 1, nullptr);

  assert(main_task_status == pdPASS && led_task_status == pdPASS && matrix_task_status == pdPASS && encoder_task_status == pdPASS
#ifdef ENABLE_USB
         && usb_task_status
#endif
#ifdef ENABLE_DISPLAY
         && display_task1_status == pdPASS && display_task2_status == pdPASS && display_gui_status == pdPASS
#endif
  );

  vTaskStartScheduler();

  while (true)
  {
    // forever
  }
}

void usb_task(void *unused)
{
#ifdef ENABLE_USB
  usb_init();
#endif

  while (true)
  {
#ifdef ENABLE_USB
    usb_periodic();
#endif
    vTaskDelay(1);
  }
}

void main_task(void *unused)
{
  bool enc_updated = false;
  bool key_updated = false;
  uint8_t keybuf[6] = {0, 0, 0, 0, 0, 0};
  uint8_t modifiers;

  const uint32_t dot_colors[] = {
      WS2812::urgb_u32(0x3f, 0x1f, 0),
      WS2812::urgb_u32(0, 0x3f, 0x3f),
      WS2812::urgb_u32(0x3f, 0, 0x3f),
  };

  static uint8_t led_states[16];
  for (uint i = 0; i < 16; i++)
  {
    led_states[i] = 0;
  }

  while (true)
  {

    TaskEncoder::event_t enc_evt;
    if (xQueueReceive(task_encoder->event_queue, &enc_evt, 1) == pdTRUE)
    {
      printf("encoder=%d value=%d\n", enc_evt.encoder, enc_evt.value);

      if (enc_evt.encoder >= 1 && enc_evt.encoder <= 3)
      {
        TaskLED::cmd_t cmd;
        cmd.cmd = TaskLED::LED_CMD_SET_RING;
        cmd.led = enc_evt.encoder - 1;
        cmd.value = enc_evt.value;
        cmd.color = dot_colors[enc_evt.encoder - 1];
        cmd.update_now = true;
        xQueueSend(task_led->cmd_queue, &cmd, 0);
      }

#ifdef ENABLE_DISPLAY
      TaskDisplay::cmd_t cmd;
      cmd.cmd = TaskDisplay::DISPLAY_CMD_UPDATE_ENCODER;
      cmd.encoder = enc_evt.encoder;
      cmd.value = enc_evt.value;
      xQueueSend(task_display->cmd_queue, &cmd, 0);
#endif

      enc_updated = true;
    }

    TaskMatrix::event_t mtx_evt;
    if (xQueueReceive(task_matrix->event_queue, &mtx_evt, 1) == pdTRUE)
    {

      printf("evt=%02x %s %s\n", mtx_evt.code, mtx_evt.press ? "press" : "release", mtx_evt.gpio ? "gpio" : "key");

#ifdef ENABLE_DISPLAY
      TaskDisplay::cmd_t cmd;
      cmd.cmd = TaskDisplay::DISPLAY_CMD_UPDATE_KEY;
      cmd.code = mtx_evt.code;
      cmd.press = mtx_evt.press;
      xQueueSend(task_display->cmd_queue, &cmd, 0);
#endif

      if (mtx_evt.gpio && mtx_evt.press)
      {
        uint8_t enc;
        if (TaskMatrix::led_encoder_map(mtx_evt.code, enc))
        {
          // pressing an encoder button resets value
          TaskEncoder::cmd_t cmd;
          cmd.cmd = TaskEncoder::ENCODER_CMD_SMART_SET_VALUE;
          cmd.encoder = enc + 1;
          cmd.value = 0;
          xQueueSend(task_encoder->cmd_queue, &cmd, 0);
        }
      }
      else if (!mtx_evt.gpio)
      {
        if (mtx_evt.press) {
          uint8_t led = 0;
          if (TaskMatrix::led_key_map(mtx_evt.code, led))
          {
            led_states[led] = led_states[led] ? 0 : 32;

            // pressing a key with an associated LED toggles it
            TaskLED::cmd_t cmd;
            cmd.cmd = TaskLED::LED_CMD_SET_SIMPLE_LED;
            cmd.led = led;
            cmd.value = led_states[led];
            cmd.update_now = true;
            xQueueSend(task_led->cmd_queue, &cmd, 0);
          }
        }

        uint8_t hid_keycode = 0;
        bool found = TaskMatrix::hid_keycode(mtx_evt.code, hid_keycode, modifiers);
        if (found && mtx_evt.press)
        {
          TaskMatrix::hid_n_key_buf_add(keybuf, hid_keycode);
          key_updated = true;
        }
        else if (found && !mtx_evt.press)
        {
          TaskMatrix::hid_n_key_buf_remove(keybuf, hid_keycode);
          key_updated = true;
        }
      }
    }

#ifdef ENABLE_USB
    if (enc_updated)
    {
      usb_pkt pkt;
      pkt.s.knob1 = task_encoder->get_value(1);
      pkt.s.knob2 = task_encoder->get_value(2);
      pkt.s.knob3 = task_encoder->get_value(3);
      pkt.s.shuttle = task_encoder->get_value(4);
      pkt.s.jog = task_encoder->get_value(0);

      tud_hid_report(0, &pkt, sizeof(pkt));
      enc_updated = false;
    }

    if (tud_hid_n_ready(ITF_KEYBOARD))
    {
      if (key_updated)
      {
        printf("key updated\n");

        if (TaskMatrix::hid_n_key_buf_is_empty(keybuf))
        {
          modifiers = 0;
        }

        tud_hid_n_keyboard_report(ITF_KEYBOARD, 0, modifiers, keybuf);

        key_updated = false;
      }
    }
#endif
  }
}
