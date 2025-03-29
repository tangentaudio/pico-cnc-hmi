#include <stdio.h>
#include "task_encoder.hh"

TaskEncoder::TaskEncoder()
{
  event_queue = xQueueCreate(10, sizeof(event_t));
  cmd_queue = xQueueCreate(10, sizeof(cmd_t));
}

TaskEncoder::~TaskEncoder()
{
}

void TaskEncoder::init()
{
  m_encoders.init();

  m_encoders.set_limits(0, 0, 0, 1);
  m_encoders.set_limits(1, 0, 14, 4);
  m_encoders.set_limits(2, 0, 14, 4);
  m_encoders.set_limits(3, 0, 14, 4);
}

void TaskEncoder::task(void *param)
{
  TaskEncoder* inst = static_cast<TaskEncoder*>(param);
  
  Encoder &encoders = inst->m_encoders;

  bool force_encoder_update = false;

  int last_value[5] = {-1, -1, -1, -1, -1};

  while (true)
  {
    cmd_t cmd;
    if (xQueueReceive(inst->cmd_queue, &cmd, 10) == pdTRUE)
    {
      if (cmd.cmd == ENCODER_CMD_SET_VALUE)
      {
        encoders.set_value(cmd.encoder, cmd.value);
        force_encoder_update = true;
      }
      else if (cmd.cmd == ENCODER_CMD_SMART_SET_VALUE)
      {
        if (cmd.encoder >= 1 && cmd.encoder <= 3)
        {
          int v = encoders.value(cmd.encoder);
          int min = encoders.get_min(cmd.encoder);
          int max = encoders.get_max(cmd.encoder);
          int span = max - min;

          printf("smart set encoder %d value %d min %d max %d span %d\n", cmd.encoder, v, min, max, span);

          if (v < min + span / 2) {
            encoders.set_value(cmd.encoder, max);
            force_encoder_update = true;
          } else {
            encoders.set_value(cmd.encoder, min);
            force_encoder_update = true;
          }
        }
      }
    }

    bool upd = encoders.task();

    if (upd || force_encoder_update)
    {
      force_encoder_update = false;

      for (int i = 1; i < 5; i++)
      {
        if (encoders.value(i) != last_value[i])
        {
          event_t evt;
          evt.encoder = i;
          evt.value = encoders.value(i);
          xQueueSend(inst->event_queue, &evt, 0);
          last_value[i] = encoders.value(i);
        }
      }
    }
  }
}

