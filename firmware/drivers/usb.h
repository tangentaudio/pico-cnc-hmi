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

typedef union pkt_u
{
  struct pkt_s
  {
    uint8_t knob1;
    uint8_t knob2;
    uint8_t knob3;
    int8_t shuttle;
    int jog;
  } s;
  unsigned char buf[64];
} usb_pkt;


#ifdef __cplusplus
} // extern "C"
#endif


#endif