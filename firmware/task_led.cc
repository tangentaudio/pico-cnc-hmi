#include "task_led.hh"

TaskLED::TaskLED()
{
  cmd_queue = xQueueCreate(10, sizeof(cmd_t));
}

TaskLED::~TaskLED()
{
}

void TaskLED::init(I2C *i2cbus)
{
  m_leds.init(i2cbus);
  m_rgbleds.init();

  #ifdef PICO_DEFAULT_LED_PIN
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  #endif
}

void TaskLED::task(void *param)
{
  TaskLED *inst = static_cast<TaskLED *>(param);
  bool state = false;

  while (true)
  {
    cmd_t cmd;
    if (xQueueReceive(inst->cmd_queue, &cmd, 250) == pdTRUE)
    {
      switch (cmd.cmd)
      {
      case LED_CMD_SET_SIMPLE_LED:
        inst->m_leds.setLED(cmd.led, cmd.value, cmd.mode, cmd.update_now);
        break;
      case LED_CMD_SET_RGB_LED:
        inst->m_rgbleds.setLED(cmd.led, cmd.value, cmd.update_now);
        break;
      case LED_CMD_SET_RING:
        inst->m_rgbleds.setRing(cmd.led, cmd.value, cmd.color, cmd.update_now);
        break;
      }
    }

    if (state)
    {
      #ifdef PICO_DEFAULT_LED_PIN
      gpio_put(PICO_DEFAULT_LED_PIN, 1);
      #endif
    }
    else
    {
      #ifdef PICO_DEFAULT_LED_PIN
      gpio_put(PICO_DEFAULT_LED_PIN, 0);
      #endif
    }
    state = !state;

    vTaskDelay(10);
  }
}



