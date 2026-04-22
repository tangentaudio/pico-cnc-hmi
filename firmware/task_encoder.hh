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

  // Synchronously set all 3 override encoder values and suppress spurious
  // change events.  Safe to call from any task under the mutex.  Intended
  // for connect/reconnect time: ensures get_value() returns the correct
  // host-authoritative value immediately, before the first user button press,
  // without relying on the encoder task draining an async queue message.
  void sync_all_immediate(int enc1, int enc2, int enc3);

  // Single-encoder variant: re-sync one override encoder to the host-
  // authoritative value.  Used while LinuxCNC's startup packets are still
  // settling and the user has not yet physically moved that encoder.
  void sync_value_immediate(uint8_t encoder, int value);

  QueueHandle_t event_queue;
  QueueHandle_t cmd_queue;

  typedef struct event
  {
    uint8_t encoder;
    int8_t value;
  } event_t;

  typedef enum {
    ENCODER_CMD_SET_VALUE,
    ENCODER_CMD_SET_VALUE_NOTIFY,
    ENCODER_CMD_SMART_SET_VALUE,
  } cmds;

  // Simplified cmd_t: the sync union branch is gone now that SYNC_ALL is
  // replaced by the synchronous sync_all_immediate() API.
  typedef struct cmd
  {
    cmds cmd;
    uint8_t encoder;
    int8_t value;
  } cmd_t;

protected:
  Encoder m_encoders;
  SemaphoreHandle_t m_mutex;

  // Shadow of encoder positions used for change-detection in task().
  // Promoted from a task()-local to a member so sync_all_immediate() can
  // update it atomically alongside m_cur_values, preventing a spurious
  // "0 → N" event firing after a reconnect sync.
  int m_last_value[5];
};

#endif
