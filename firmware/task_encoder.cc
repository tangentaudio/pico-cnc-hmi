#include <stdio.h>
#include <stdlib.h>
#include "task_encoder.hh"

TaskEncoder::TaskEncoder()
{
  event_queue = xQueueCreate(10, sizeof(event_t));
  cmd_queue = xQueueCreate(10, sizeof(cmd_t));

  m_mutex = xSemaphoreCreateMutex();
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

int TaskEncoder::get_value(uint8_t encoder, bool raw)
{
  xSemaphoreTake(m_mutex, portMAX_DELAY);
  int value = m_encoders.value(encoder);
  xSemaphoreGive(m_mutex);

  if (raw)
    return value;

   if (encoder == 4) {
    // shuttle
    uint8_t shidx = abs(value);
    bool pos = value >= 0;
    const int lookup[8] = {0, 5, 10, 20, 50, 100, 250, 600};

    return (pos ? lookup[shidx] : -lookup[shidx]);
   }

  return value;
}

void TaskEncoder::task(void *param)
{
  TaskEncoder* inst = static_cast<TaskEncoder*>(param);
  
  Encoder &encoders = inst->m_encoders;

  bool force_encoder_update = false;

  // Initialize last_value to 0 (not -1) to match the hardware-reset encoder
  // state on startup.  Using -1 would cause spurious "0 != -1" events to fire
  // immediately, triggering IN packets with knob=0 before the startup sync
  // (set_encoder_value) can set them to the correct LinuxCNC values.
  int last_value[5] = {0, 0, 0, 0, 0};

  while (true)
  {
    cmd_t cmd;
    if (xQueueReceive(inst->cmd_queue, &cmd, 10) == pdTRUE)
    {
      if (cmd.cmd == ENCODER_CMD_SET_VALUE)
      {
        // Sync encoder position: used only on startup to match ring display.
        // After initial sync, encoder position is independent user input.
        encoders.set_value(cmd.encoder, cmd.value);
        last_value[cmd.encoder] = cmd.value;  // no event fired
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

      for (int i = 0; i < 5; i++)
      {
        if (encoders.value(i) != last_value[i])
        {
          event_t evt;
          evt.encoder = i;
          evt.value = encoders.value(i);
          if (xQueueSend(inst->event_queue, &evt, 0) != pdTRUE)
          {
            printf("encoder event queue full: encoder=%d value=%d\n", evt.encoder, evt.value);
          }
          last_value[i] = encoders.value(i);
        }
      }
    }
  }
}

