#ifndef __TASK_LED_HH
#define __TASK_LED_HH

#include <FreeRTOS.h>
#include <queue.h>
#include "tlc59116.hh"
#include "ws2812.hh"
#include "i2c.hh"

class TaskLED
{
public:
  TaskLED();
  ~TaskLED();

  void init(I2C *i2cbus);
  static void task(void *param);

  QueueHandle_t cmd_queue;

  typedef struct event
  {
    uint8_t led;
    uint8_t value;
  } event_t;

  typedef enum {
    LED_CMD_SET_SIMPLE_LED,
    LED_CMD_SET_RGB_LED,
    LED_CMD_SET_RING,
  } cmds;

  typedef enum {
    NORMAL = LED_MODE_BRIGHT,
    BLINK = LED_MODE_BRIGHTBLINK
  } modes;

  typedef struct cmd
  {
    cmds cmd;
    uint8_t led;
    uint8_t value;
    uint8_t mode;
    uint32_t color;
    bool update_now;
  } cmd_t;

protected:
  TLC59116 m_leds;
  WS2812 m_rgbleds;
};



#endif