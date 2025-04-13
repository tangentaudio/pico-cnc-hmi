#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <queue.h>
#include <boards/pico.h>
#include "pico/stdlib.h"
#include "bsp/board_api.h"
#include "tusb.h"
#include "usb.h"


QueueHandle_t usb_out_queue;

void usb_init(void)
{
  board_init();
  //tud_init(BOARD_TUD_RHPORT);
  
  tusb_rhport_init_t dev_init = {
     .role = TUSB_ROLE_DEVICE,
     .speed = TUSB_SPEED_AUTO
  };
  tusb_init(BOARD_TUD_RHPORT, &dev_init); // initialize device stack on roothub port 0  

  if (board_init_after_tusb)
    board_init_after_tusb();

  usb_out_queue = xQueueCreate(10, sizeof(usb_out_pkt));
}

void usb_periodic(void)
{
  tud_task();
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  //blink_interval_ms = BLINK_MOUNTED;
  printf("mount cb\n");
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  //blink_interval_ms = BLINK_NOT_MOUNTED;
  printf("umount cb\n");
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void)remote_wakeup_en;
  //blink_interval_ms = BLINK_SUSPENDED;
  printf("suspend cb\n");
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  //blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
  printf("resume cb\n");
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
  
  if (bufsize == 64) {
    usb_out_pkt pkt;
    memcpy(&pkt, buffer, bufsize);

    xQueueSend(usb_out_queue, &pkt, 0);
  } else {
    printf("report itf=%d id=%d type=%d size=%d\n", itf, report_id, report_type, bufsize);
    for(uint8_t i=0; i<bufsize; i++)
    {
      printf("%02x ", buffer[i]);
    }
    printf("\n");
  }
}


void usb_hid_periodic(void)
{
  // Poll every 10ms
  const uint32_t interval_ms = 10;
  static uint32_t start_ms = 0;

  if ( board_millis() - start_ms < interval_ms) return; // not enough time
  start_ms += interval_ms;

  uint32_t const btn = board_button_read();

  #ifdef REMOTE_WAKEUP
  // Remote wakeup
  if ( tud_suspended() && btn )
  {
    // Wake up host if we are in suspend mode
    // and REMOTE_WAKEUP feature is enabled by host
    tud_remote_wakeup();
  }
  #endif

  // Keeb
  if ( tud_hid_n_ready(ITF_KEYBOARD) )
  {
    // use to avoid send multiple consecutive zero report for keyboard
    static bool has_key = false;

    if ( btn )
    {
      uint8_t keycode[6] = { 0 };
      keycode[0] = HID_KEY_A;

      tud_hid_n_keyboard_report(ITF_KEYBOARD, 0, 0, keycode);

      has_key = true;
    }else
    {
      // send empty key report if previously has key pressed
      if (has_key) tud_hid_n_keyboard_report(ITF_KEYBOARD, 0, 0, NULL);
      has_key = false;
    }
  }

}

void usb_dump_out_pkt(usb_out_pkt* pkt)
{
  printf("usb_out_pkt: heartbeat=%d estop=%d enabled=%d mode=%d interp_state=%d feedrate_override=%d rapidrate_override=%d maxvel_override=%d\n",
    pkt->s.heartbeat, pkt->s.estop, pkt->s.enabled, pkt->s.mode, pkt->s.interp_state, pkt->s.feedrate_override, pkt->s.rapidrate_override, pkt->s.maxvel_override);
}

void usb_dump_in_pkt(usb_in_pkt* pkt)
{
  printf("usb_in_pkt: knob1=%d knob2=%d knob3=%d axis=%d step=%d jog=%d shuttle=%d motion_cmd=%d\n",
    pkt->s.knob1, pkt->s.knob2, pkt->s.knob3, pkt->s.axis, pkt->s.step, pkt->s.jog, pkt->s.shuttle, pkt->s.motion_cmd);
}
