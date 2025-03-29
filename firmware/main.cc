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
#include "pico/time.h"

#ifdef USB_ENABLED
#include "bsp/board_api.h"
#include "tusb.h"
#endif

#include "i2c.hh"

#include "task_encoder.hh"
#include "task_matrix.hh"
#include "task_led.hh"



#define PIN_PERIPH_RESETN 22

I2C* i2c;
TaskMatrix* task_matrix;
TaskEncoder* task_encoder;
TaskLED* task_led;

void main_task(void *unused);

int main(void)
{
  printf("Reset peripherals...");
  gpio_init(PIN_PERIPH_RESETN);
  gpio_set_dir(PIN_PERIPH_RESETN, GPIO_OUT);
  gpio_pull_up(PIN_PERIPH_RESETN);
  gpio_put(PIN_PERIPH_RESETN, 0);
  sleep_ms(100);
  gpio_put(PIN_PERIPH_RESETN, 1);
  printf("OK\n");

  i2c = new I2C();
  i2c->init();
  task_encoder = new TaskEncoder();
  task_encoder->init();
 
  task_matrix = new TaskMatrix();
  task_matrix->init(i2c);

  task_led = new TaskLED();
  task_led->init(i2c);

  #ifdef USB_ENABLED
  board_init();
  tud_init(BOARD_TUD_RHPORT);

  if (board_init_after_tusb)
    board_init_after_tusb();
  #endif


  BaseType_t matrix_task_status = xTaskCreate(task_matrix->task, "MATRIX_TASK", 1024, (void*)task_matrix, 3, nullptr);
  BaseType_t encoder_task_status = xTaskCreate(task_encoder->task, "ENCODER_TASK", 1024, (void*)task_encoder, 1, nullptr);
  BaseType_t led_task_status = xTaskCreate(task_led->task, "LED_TASK", 1024, (void*)task_led, 2, nullptr);
  BaseType_t main_task_status = xTaskCreate(main_task, "MAIN_TASK", 1024, nullptr, 2, nullptr);

  assert(main_task_status == pdPASS && led_task_status == pdPASS && matrix_task_status == pdPASS && encoder_task_status == pdPASS);

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
      WS2812::urgb_u32(0x7f, 0, 0),
      WS2812::urgb_u32(0, 0x7f, 0),
      WS2812::urgb_u32(0, 0, 0x7f),
  };

  while (true)
  {
    TaskEncoder::event_t enc_evt;
    if (xQueueReceive(task_encoder->event_queue, &enc_evt, 10) == pdTRUE)
    {
      printf("encoder=%d value=%d\n", enc_evt.encoder, enc_evt.value);

      if (enc_evt.encoder >= 1 && enc_evt.encoder <= 3) {
        TaskLED::cmd_t cmd;
        cmd.cmd = TaskLED::LED_CMD_SET_RING;
        cmd.led = enc_evt.encoder - 1;
        cmd.value = enc_evt.value;
        cmd.color = dot_colors[enc_evt.encoder - 1];
        cmd.update_now = true;
        xQueueSend(task_led->cmd_queue, &cmd, 0);
      }
    }

    TaskMatrix::event_t mtx_evt;
    if (xQueueReceive(task_matrix->event_queue, &mtx_evt, 10) == pdTRUE)
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
          TaskEncoder::cmd_t cmd;
          cmd.cmd = TaskEncoder::ENCODER_CMD_SET_VALUE;
          cmd.encoder = enc;
          cmd.value = 0;
          xQueueSend(task_encoder->cmd_queue, &cmd, 0);
        } 
        
      } else {
        uint8_t led = 0;
        for (int i = 0; i < 12; i++) {
          if (mtx_evt.code == key_to_led[i]) {
            led = i + 1;
            break;
          }
        }
        if (led > 0 && led <= 16)
        {
          TaskLED::cmd_t cmd;
          cmd.cmd = TaskLED::LED_CMD_SET_SIMPLE_LED;
          cmd.led = led - 1;
          cmd.value = mtx_evt.press ? 64 : 0;
          cmd.update_now = true;
          xQueueSend(task_led->cmd_queue, &cmd, 0);
        }
      }

    }

    #ifdef USB_ENABLED
    tud_task();
    #endif
  }

}

