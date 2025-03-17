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
#include "tusb.h"

#include <hardware/pio.h>
#include "quadrature.pio.h"

#define ENCODER_1_PINS 12
#define ENCODER_2_PINS 14
#define ENCODER_3_PINS 16



/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

PIO pio;
uint sm1, sm2, sm3;

void led_blinking_task(void);

union pkt_u {
  struct pkt_s {
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
  board_init();
  tud_init(BOARD_TUD_RHPORT);

  if (board_init_after_tusb) {
    board_init_after_tusb();
  }

  pio = pio0;
  pio_add_program(pio, &quadrature_encoder_program);
  sm1 = pio_claim_unused_sm(pio, true);
  quadrature_encoder_program_init(pio, sm1, ENCODER_1_PINS, 0);
  
  sm2 = pio_claim_unused_sm(pio, true);
  quadrature_encoder_program_init(pio, sm2, ENCODER_2_PINS, 0);

  sm3 = pio_claim_unused_sm(pio, true);
  quadrature_encoder_program_init(pio, sm3, ENCODER_3_PINS, 0);

  
  //gpio_init(ENCODER_1_PIN_BUTTON);
  //gpio_set_dir(ENCODER_1_PIN_BUTTON, GPIO_IN);
  //gpio_pull_up(ENCODER_1_PIN_BUTTON);

  while (1)
  {
    static int last_value_1 = -1, last_value_2 = -1, last_value_3 = -1;
    bool update = false;

    tud_task();
    led_blinking_task();

    // note: thanks to two's complement arithmetic delta will always
    // be correct even when new_value wraps around MAXINT / MININT
    int new_value = quadrature_encoder_get_count(pio, sm1);
    if (new_value != last_value_1) {
        printf("position #1 %8d\n", new_value);
        last_value_1 = new_value;
        update = true;
    }

    new_value = quadrature_encoder_get_count(pio, sm2);
    if (new_value != last_value_2) {
        printf("position #2 %8d\n", new_value);
        last_value_2 = new_value;
        update = true;
    }

    new_value = quadrature_encoder_get_count(pio, sm3);
    if (new_value != last_value_3) {
        printf("position #3 %8d\n", new_value);
        last_value_3 = new_value;
        update = true;
    }


    //bool btn = gpio_get(ENCODER_1_PIN_BUTTON);
    //if ( btn != last_btn_1) {
    //  last_btn_1 = btn;
    //  update = true;
    //} 

    if (update) {
      pkt.s.knob1 = last_value_1;
      pkt.s.knob2 = last_value_2;
      pkt.s.knob3 = last_value_3;
      pkt.s.button1 = 0;
      pkt.s.button2 = 0;
      pkt.s.button3 = 0;

      tud_hid_report(0, &pkt, sizeof(pkt));
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
  (void) remote_wakeup_en;
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
uint16_t tud_hid_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  // TODO not Implemented
  (void) itf;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
  // This example doesn't use multiple report and report ID
  (void) itf;
  (void) report_id;
  (void) report_type;

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
  if ( board_millis() - start_ms < blink_interval_ms) return; // not enough time
  start_ms += blink_interval_ms;

  board_led_write(led_state);
  led_state = 1 - led_state; // toggle
}
