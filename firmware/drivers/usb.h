#ifndef __USB_H
#define __USB_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "bsp/board_api.h"
#include "tusb.h"

void usb_init(void);
void usb_periodic(void);
void usb_hid_periodic(void);


typedef union in_pkt_u
{
  struct  __attribute__ ((packed)) in_pkt_s
  {
    uint8_t knob1;
    uint8_t knob2;
    uint8_t knob3;
    uint8_t axis;
    uint32_t step;
    int jog;
    int shuttle;
    uint8_t motion_cmd;
  } s;
  unsigned char buf[64];
}  __attribute__ ((packed)) usb_in_pkt;

typedef union out_pkt_u
{
  struct  __attribute__ ((packed)) out_pkt_s
  {
    uint8_t header;
    uint8_t estop;
    uint8_t enabled;
    uint8_t mode;
    uint8_t interp_state;
    uint8_t feedrate_override;
    uint8_t rapidrate_override;
    uint8_t maxvel_override;
  } s;
  unsigned char buf[64];
}  __attribute__ ((packed)) usb_out_pkt;

typedef enum {
  INTERP_OFF = 0,
  INTERP_IDLE,
  INTERP_READING,
  INTERP_PAUSED,
  INTERP_WAITING
} interp_t;

typedef enum {
  MODE_UNKNOWN = 0,
  MODE_MANUAL,
  MODE_AUTO,
  MODE_MDI,
  MODE_TELEOP
} mode_t;

// Interface index depends on the order in configuration descriptor
enum {
  ITF_GENERIC_HID = 0,
  ITF_KEYBOARD = 1,
};

extern QueueHandle_t usb_out_queue;

void usb_dump_out_pkt(usb_out_pkt* pkt);
void usb_dump_in_pkt(usb_in_pkt* pkt);

#ifdef __cplusplus
} // extern "C"
#endif



#endif