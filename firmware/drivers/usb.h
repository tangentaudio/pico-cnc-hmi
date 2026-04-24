#ifndef __USB_H
#define __USB_H

#ifdef __cplusplus
extern "C"
{
#endif

#include "bsp/board_api.h"
#include "tusb.h"

extern volatile uint32_t usb_out_queue_drops;

void usb_init(void);
void usb_periodic(void);


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
    uint8_t heartbeat;
    uint8_t estop;
    uint8_t enabled;
    uint8_t mode;
    uint8_t interp_state;
    uint8_t feedrate_override;
    uint8_t rapidrate_override;
    uint8_t maxvel_override;
    uint8_t step_mode;            // hmi_step_mode from hmi.py (was task_paused)
    uint8_t paused;           // s.paused from LinuxCNC (was inpos)
    uint8_t coolant;
    uint8_t optional_stop;
    uint8_t homed;
    uint8_t cmd;          // command byte: 0=none, 0xB0=reboot to BOOTSEL
    uint8_t _pad;         // pad to 4-byte-align the pos fields (offset 16)
    int32_t pos_x;        // X axis position × 10000 (e.g. 12345 = 1.2345")
    int32_t pos_y;        // Y axis position × 10000
    int32_t pos_z;        // Z axis position × 10000
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

// Packet header bytes
#define USB_PKT_HEADER_RUNTIME  0xAA
#define USB_PKT_HEADER_CONFIG   0xAB

// One-time configuration packet sent by the host once on connect.
// The HMI caches this so the display task can show real units/percentages
// and so the knob-reset-to-100% logic knows the right target segment.
//
// All fields use fixed-point integer encoding to avoid floating point on the wire:
//   max_feed_pct, max_rapid_pct  — override ceiling as percent (e.g. 150 for 150%)
//   max_vel_x10, min_vel_x10     — max/min velocity × 10 (e.g. 600 for 60.0 IPM)
//   curve_x100                   — maxvel power-law exponent × 100 (e.g. 200 for 2.0)
typedef union cfg_pkt_u
{
  struct __attribute__((packed)) cfg_pkt_s
  {
    uint8_t  header;          // USB_PKT_HEADER_CONFIG (0xAB)
    uint16_t max_feed_pct;    // max feed override ceiling as percent (e.g. 150)
    uint16_t max_rapid_pct;   // max rapid override ceiling as percent (e.g. 100)
    uint16_t max_vel_x10;     // configured_maxvel * 10
    uint16_t min_vel_x10;     // configured_maxvel_min * 10
    uint16_t curve_x100;      // maxvel power-law curve exponent * 100
  } s;
  unsigned char buf[64];
} __attribute__((packed)) usb_cfg_pkt;

// Cached config values, updated when a config packet is received.
// valid is set to true once the first config packet arrives.
typedef struct {
  bool     valid;
  uint16_t max_feed_pct;
  uint16_t max_rapid_pct;
  uint16_t max_vel_x10;
  uint16_t min_vel_x10;
  uint16_t curve_x100;
} hmi_config_t;

extern hmi_config_t hmi_config;

// Piecewise-linear override mapping: segments 0-7 span 0-100%, segments 7-14 span
// 100%-max%.  Segment 7 is always the visual 100% anchor (top-centre ring LED).
// Returns feed/rapid override as integer percent (e.g. 73 for 73%).
static inline uint16_t hmi_feed_pct(uint8_t seg) {
    if (seg <= 7) return (uint16_t)((uint32_t)seg * 100u / 7u);
    uint16_t over = (hmi_config.max_feed_pct  > 100u) ? (hmi_config.max_feed_pct  - 100u) : 0u;
    return (uint16_t)(100u + (uint32_t)(seg - 7u) * over / 7u);
}
static inline uint16_t hmi_rapid_pct(uint8_t seg) {
    // When max rapid <= 100%: full ring spans 0-100% linearly.
    // When max rapid > 100%: piecewise, seg 7 = 100% anchor.
    if (!hmi_config.valid || hmi_config.max_rapid_pct <= 100u)
        return (uint16_t)((uint32_t)seg * 100u / 14u);
    if (seg <= 7) return (uint16_t)((uint32_t)seg * 100u / 7u);
    uint16_t over = hmi_config.max_rapid_pct - 100u;
    return (uint16_t)(100u + (uint32_t)(seg - 7u) * over / 7u);
}

// Compute the display value for the maxvel knob from its segment (0-14).
// Returns actual velocity × 10 (e.g. 423 for 42.3 IPM).
// Uses the same power-law mapping as hmi.py seg_to_maxvel().
uint16_t hmi_maxvel_x10(uint8_t seg);

// Feed: segment 7 is the 100% anchor (piecewise range can exceed 100%).
// Rapid: full ring when max <= 100% (reset to seg 14), piecewise anchor at seg 7 otherwise.
static inline uint8_t hmi_feed_reset_seg(void)  { return 7; }
static inline uint8_t hmi_rapid_reset_seg(void) {
    return (hmi_config.valid && hmi_config.max_rapid_pct > 100u) ? 7 : 14;
}

// Interface index depends on the order in configuration descriptor
enum {
  ITF_GENERIC_HID = 0,
  ITF_KEYBOARD = 1,
};

extern QueueHandle_t usb_out_queue;

void usb_dump_out_pkt(usb_out_pkt* pkt);
void usb_dump_in_pkt(usb_in_pkt* pkt);

// Feature report: firmware version (GET_REPORT type=FEATURE, report_id=0)
typedef struct __attribute__((packed)) {
    uint8_t  major;
    uint8_t  minor;
    uint16_t build;
    char     hash[8];
    uint8_t  dirty;
} usb_version_report_t;

extern usb_version_report_t usb_version_report;

#ifdef __cplusplus
} // extern "C"
#endif



#endif