#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include <boards/pico.h>
#include "pico/stdlib.h"
#include "bsp/board_api.h"
#include "tusb.h"
#include "usb.h"


QueueHandle_t usb_out_queue;
volatile uint32_t usb_out_queue_drops = 0;

hmi_config_t hmi_config = {
  .valid        = false,
  .max_feed_pct  = 100,
  .max_rapid_pct = 100,
  .max_vel_x10   = 10,   // 1.0 unit/s — safe non-zero default
  .min_vel_x10   = 0,
  .curve_x100    = 200,  // exponent 2.0
};

// Compute actual velocity × 10 from a segment index using the configured
// power-law curve, mirroring hmi.py seg_to_maxvel().
uint16_t hmi_maxvel_x10(uint8_t seg)
{
  if (seg == 0 || !hmi_config.valid) return 0;
  if (seg > 14) seg = 14;
  // frac = (seg / 14.0) ^ (curve_x100 / 100.0)
  // We do this in fixed-point via double to avoid needing math.h float on M33.
  // This runs infrequently (display refresh only) so the cost is acceptable.
  double frac_base = (double)seg / 14.0;
  double exponent  = (double)hmi_config.curve_x100 / 100.0;
  double frac      = 1.0;
  // Cheap integer-exponent fast path (curve is almost always 2.0 or 3.0)
  uint16_t exp_int = hmi_config.curve_x100 / 100;
  if (hmi_config.curve_x100 == exp_int * 100) {
    frac = 1.0;
    for (uint16_t i = 0; i < exp_int; i++) frac *= frac_base;
  } else {
    // Fall back to pow() for non-integer exponents
    extern double pow(double, double);
    frac = pow(frac_base, exponent);
  }
  uint32_t span = (uint32_t)hmi_config.max_vel_x10 - (uint32_t)hmi_config.min_vel_x10;
  uint32_t val  = (uint32_t)hmi_config.min_vel_x10 + (uint32_t)(frac * (double)span + 0.5);
  return (uint16_t)(val < 65535 ? val : 65535);
}

void usb_init(void)
{
  board_init();

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
  if (bufsize == 64 && buffer[0] == USB_PKT_HEADER_CONFIG) {
    // Config packet — cache values, do not enqueue as a runtime packet.
    usb_cfg_pkt cfg;
    memcpy(&cfg, buffer, sizeof(cfg));
    hmi_config.max_feed_pct  = cfg.s.max_feed_pct  ? cfg.s.max_feed_pct  : 100;
    hmi_config.max_rapid_pct = cfg.s.max_rapid_pct ? cfg.s.max_rapid_pct : 100;
    hmi_config.max_vel_x10   = cfg.s.max_vel_x10   ? cfg.s.max_vel_x10   : 10;
    hmi_config.min_vel_x10   = cfg.s.min_vel_x10;
    hmi_config.curve_x100    = cfg.s.curve_x100     ? cfg.s.curve_x100    : 200;
    hmi_config.valid         = true;
    printf("hmi_config: feed=%d%% rapid=%d%% maxvel=%.1f minvel=%.1f curve=%.2f\n",
           hmi_config.max_feed_pct, hmi_config.max_rapid_pct,
           hmi_config.max_vel_x10 / 10.0f, hmi_config.min_vel_x10 / 10.0f,
           hmi_config.curve_x100 / 100.0f);
  } else if (bufsize == 64) {
    usb_out_pkt pkt;
    memcpy(&pkt, buffer, bufsize);

    if (xQueueSend(usb_out_queue, &pkt, 0) != pdTRUE) {
      // Cannot block or printf safely in USB callback; count silently.
      // main_task() logs usb_out_queue_drops periodically if non-zero.
      extern volatile uint32_t usb_out_queue_drops;
      usb_out_queue_drops++;
    }
  } else {
    printf("report itf=%d id=%d type=%d size=%d\n", itf, report_id, report_type, bufsize);
    for(uint8_t i=0; i<bufsize; i++)
    {
      printf("%02x ", buffer[i]);
    }
    printf("\n");
  }
}

void usb_dump_out_pkt(usb_out_pkt* pkt)
{
  printf("usb_out_pkt: heartbeat=%d estop=%d enabled=%d mode=%d interp_state=%d feedrate_override=%d rapidrate_override=%d maxvel_override=%d task_paused=%d inpos=%d coolant=%d optional_stop=%d homed=%d\n",
    pkt->s.heartbeat, pkt->s.estop, pkt->s.enabled, pkt->s.mode, pkt->s.interp_state, pkt->s.feedrate_override, pkt->s.rapidrate_override, pkt->s.maxvel_override, pkt->s.task_paused, pkt->s.inpos, pkt->s.coolant, pkt->s.optional_stop, pkt->s.homed);
}

void usb_dump_in_pkt(usb_in_pkt* pkt)
{
  printf("usb_in_pkt: knob1=%d knob2=%d knob3=%d axis=%d step=%d jog=%d shuttle=%d motion_cmd=%d\n",
    pkt->s.knob1, pkt->s.knob2, pkt->s.knob3, pkt->s.axis, pkt->s.step, pkt->s.jog, pkt->s.shuttle, pkt->s.motion_cmd);
}
