
#include "task_matrix.hh"


TaskMatrix::TaskMatrix()
{
    event_queue = xQueueCreate(10, sizeof(event_t));
}

TaskMatrix::~TaskMatrix()
{
}

void TaskMatrix::init(I2C *i2cbus)
{
  m_i2c = i2cbus;
  m_matrix.init(m_i2c);
}


void TaskMatrix::task(void *param)
{
  TaskMatrix* inst = static_cast<TaskMatrix*>(param);

  while (true)
  {
    uint8_t avail = inst->m_matrix.available();
    if (avail)
    {
      if (avail & 0x80)
      {
        event_t qevt;
        qevt.err = true;
        qevt.code = 0;
        xQueueSend(inst->event_queue, &qevt, 0);
      }

      uint8_t evt = inst->m_matrix.getEvent();

      event_t qevt;
      qevt.code = evt & 0x7F;
      qevt.press = evt & 0x80;
      qevt.gpio = qevt.code > 0x5B;
      qevt.err = false;
      xQueueSend(inst->event_queue, &qevt, 0);
    }
    vTaskDelay(10);
  }
}

