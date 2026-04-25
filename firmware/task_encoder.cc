#include <stdio.h>
#include <stdlib.h>
#include "task_encoder.hh"

TaskEncoder::TaskEncoder()
{
  event_queue = xQueueCreate(10, sizeof(event_t));
  cmd_queue = xQueueCreate(10, sizeof(cmd_t));

  m_mutex = xSemaphoreCreateMutex();

  for (int i = 0; i < 5; i++)
    m_last_value[i] = 0;
}

TaskEncoder::~TaskEncoder()
{
}

void TaskEncoder::init()
{
  m_encoders.init();

  m_encoders.set_limits(0, 0, 0, 4);  // jog wheel: div=4 so 1 physical click = 1 count
  m_encoders.set_limits(1, 0, 14, 4);
  m_encoders.set_limits(2, 0, 14, 4);
  m_encoders.set_limits(3, 0, 14, 4);
}

int TaskEncoder::get_value(uint8_t encoder, bool raw)
{
  xSemaphoreTake(m_mutex, portMAX_DELAY);
  int value = m_encoders.value(encoder);
  xSemaphoreGive(m_mutex);

  return value;
}

void TaskEncoder::sync_all_immediate(int enc1, int enc2, int enc3)
{
  xSemaphoreTake(m_mutex, portMAX_DELAY);
  m_encoders.set_value(1, enc1);
  m_encoders.set_value(2, enc2);
  m_encoders.set_value(3, enc3);
  // Update shadow values so change-detection in task() sees no delta —
  // suppressing the spurious "0 → N" event that would otherwise fire the
  // next time the encoder task polls after a reconnect.
  m_last_value[1] = enc1;
  m_last_value[2] = enc2;
  m_last_value[3] = enc3;
  printf("sync_all_immediate: enc1=%d enc2=%d enc3=%d => v1=%d v2=%d v3=%d\n",
         enc1, enc2, enc3,
         m_encoders.value(1), m_encoders.value(2), m_encoders.value(3));
  xSemaphoreGive(m_mutex);
}

void TaskEncoder::sync_value_immediate(uint8_t encoder, int value)
{
  xSemaphoreTake(m_mutex, portMAX_DELAY);
  m_encoders.set_value(encoder, value);
  m_last_value[encoder] = value;
  printf("sync_value_immediate: enc=%d value=%d => v=%d\n",
         encoder, value, m_encoders.value(encoder));
  xSemaphoreGive(m_mutex);
}

void TaskEncoder::task(void *param)
{
  TaskEncoder* inst = static_cast<TaskEncoder*>(param);
  
  Encoder &encoders = inst->m_encoders;

  bool force_encoder_update = false;

  // m_last_value[] is now a member (initialized to 0 in constructor) rather
  // than a local variable.  This allows sync_all_immediate() to update the
  // shadow atomically under the mutex so get_value() is immediately correct
  // after a reconnect, before this task drains the cmd queue.

  while (true)
  {
    // Drain ALL pending commands before polling hardware.
    cmd_t cmd;
    while (xQueueReceive(inst->cmd_queue, &cmd, 0) == pdTRUE)
    {
      if (cmd.cmd == ENCODER_CMD_SET_VALUE)
      {
        // Suppress the change event: write both the encoder value and the
        // shadow together under the mutex.
        xSemaphoreTake(inst->m_mutex, portMAX_DELAY);
        encoders.set_value(cmd.encoder, cmd.value);
        inst->m_last_value[cmd.encoder] = cmd.value;
        xSemaphoreGive(inst->m_mutex);
      }
      else if (cmd.cmd == ENCODER_CMD_SET_VALUE_NOTIFY)
      {
        // Like SET_VALUE but fires an encoder event so the IN packet is
        // triggered AFTER the value is applied.  Don't update m_last_value —
        // let the change detection loop below fire the event.
        xSemaphoreTake(inst->m_mutex, portMAX_DELAY);
        encoders.set_value(cmd.encoder, cmd.value);
        xSemaphoreGive(inst->m_mutex);
        force_encoder_update = true;
      }
      else if (cmd.cmd == ENCODER_CMD_SMART_SET_VALUE)
      {
        if (cmd.encoder >= 1 && cmd.encoder <= 3)
        {
          xSemaphoreTake(inst->m_mutex, portMAX_DELAY);
          int v = encoders.value(cmd.encoder);
          int min = encoders.get_min(cmd.encoder);
          int max = encoders.get_max(cmd.encoder);
          int span = max - min;

          printf("smart set encoder %d value %d min %d max %d span %d\n", cmd.encoder, v, min, max, span);

          if (v < min + span / 2) {
            encoders.set_value(cmd.encoder, max);
          } else {
            encoders.set_value(cmd.encoder, min);
          }
          xSemaphoreGive(inst->m_mutex);
          force_encoder_update = true;
        }
      }
    }

    // Poll hardware and fire change events, all under the mutex so that
    // sync_all_immediate() from main_task cannot race with m_last_value updates.
    xSemaphoreTake(inst->m_mutex, portMAX_DELAY);
    bool upd = encoders.task();

    if (upd || force_encoder_update)
    {
      force_encoder_update = false;

      for (int i = 0; i < 5; i++)
      {
        int cur = encoders.value(i);
        if (cur != inst->m_last_value[i])
        {
          event_t evt;
          evt.encoder = i;
          evt.value = cur;
          if (xQueueSend(inst->event_queue, &evt, 0) != pdTRUE)
          {
            printf("encoder event queue full: encoder=%d value=%d\n", evt.encoder, evt.value);
          }
          inst->m_last_value[i] = cur;
        }
      }
    }
    xSemaphoreGive(inst->m_mutex);

    vTaskDelay(10);
  }
}
