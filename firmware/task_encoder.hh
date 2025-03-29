#ifndef __TASK_ENCODER_HH
#define __TASK_ENCODER_HH

#include <FreeRTOS.h>
#include <queue.h>
#include "encoder.hh"

class TaskEncoder
{

public:
  TaskEncoder();
  ~TaskEncoder();

  void init();
  static void task(void *param);

  QueueHandle_t event_queue;
  QueueHandle_t cmd_queue;

  typedef struct event
  {
    uint8_t encoder;
    int8_t value;
  } event_t;

  typedef enum {
    ENCODER_CMD_SET_VALUE,
  } cmds;

  typedef struct cmd
  {
    cmds cmd;
    uint8_t encoder;
    int8_t value;
  } cmd_t;

protected:
  Encoder m_encoders;
};

#endif
