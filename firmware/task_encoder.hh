#ifndef __TASK_ENCODER_HH
#define __TASK_ENCODER_HH

#include <FreeRTOS.h>
#include <queue.h>
#include "semphr.h"
#include "encoder.hh"

class TaskEncoder
{

public:
  TaskEncoder();
  ~TaskEncoder();

  void init();
  static void task(void *param);

  int get_value(uint8_t encoder, bool raw = true);
  QueueHandle_t event_queue;
  QueueHandle_t cmd_queue;


  typedef struct event
  {
    uint8_t encoder;
    int8_t value;
  } event_t;

  typedef enum {
    ENCODER_CMD_SET_VALUE,
    ENCODER_CMD_SMART_SET_VALUE,
  } cmds;

  typedef struct cmd
  {
    cmds cmd;
    uint8_t encoder;
    int8_t value;
  } cmd_t;

protected:
  Encoder m_encoders;
  SemaphoreHandle_t m_mutex;
};

#endif
