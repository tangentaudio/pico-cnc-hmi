// PICO 2 GPIO MAP HW REV0
// =======================
// GPIO  0 - UART0 TX
// GPIO  1 - UART0 RX
// GPIO  2 - SPI0_SCK
// GPIO  3 - SPI0_DOUT
// GPIO  4 - LCD_A0
// GPIO  5 - SPI0_CS0
// GPIO  6 - JOG_A
// GPIO  7 - JOG_B
// GPIO  8 - SHUTTLE_0
// GPIO  9 - SHUTTLE_1
// GPIO 10 - SHUTTLE_2
// GPIO 11 - SHUTTLE_3
// GPIO 12 - ENC0_A
// GPIO 13 - ENC0_B
// GPIO 14 - ENC1_A
// GPIO 15 - ENC1_B
// GPIO 16 - ENC2_A
// GPIO 17 - ENC2_B
// GPIO 18 - SPARE
// GPIO 19 - /KEY_INT
// GPIO 20 - I2C0_SDA
// GPIO 21 - I2C0_SCL
// GPIO 22 - /PERIPH_RESET
// GPIO 26 - SPARE
// GPIO 27 - SPARE
// GPIO 28 - SPARE

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <boards/pico.h>
#include "pico/stdlib.h"
#include "drivers/i2c.hh"
#include "task_encoder.hh"
#include "task_matrix.hh"
#include "task_led.hh"
#ifdef ENABLE_DISPLAY
#include "task_display.hh"
#endif
#ifdef ENABLE_USB
#include "drivers/usb.h"
#include "tusb.h"
#endif

#define PIN_PERIPH_RESETN 22

I2C *i2c;
TaskMatrix *task_matrix;
TaskEncoder *task_encoder;
TaskLED *task_led;

#ifdef ENABLE_DISPLAY
TaskDisplay *task_display;
#endif

void main_task(void *unused);
void usb_task(void *unused);

int main(void)
{
  stdout_uart_init();

  #ifdef ENABLE_USB
  usb_init();
  #endif

  gpio_init(PIN_PERIPH_RESETN);
  gpio_set_dir(PIN_PERIPH_RESETN, GPIO_OUT);
  gpio_pull_up(PIN_PERIPH_RESETN);
  gpio_put(PIN_PERIPH_RESETN, 0);
  sleep_ms(100);
  gpio_put(PIN_PERIPH_RESETN, 1);

#ifdef ENABLE_DISPLAY
  task_display = new TaskDisplay();
  task_display->init();
#endif

  i2c = new I2C();
  i2c->init();
  task_encoder = new TaskEncoder();
  task_encoder->init();

  task_matrix = new TaskMatrix();
  task_matrix->init(i2c);

  task_led = new TaskLED();
  task_led->init(i2c);

  BaseType_t matrix_task_status = xTaskCreate(task_matrix->task, "MATRIX_TASK", 256, (void *)task_matrix, 4, nullptr);
  BaseType_t encoder_task_status = xTaskCreate(task_encoder->task, "ENCODER_TASK", 256, (void *)task_encoder, 4, nullptr);
  BaseType_t led_task_status = xTaskCreate(task_led->task, "LED_TASK", 256, (void *)task_led, 2, nullptr);
#ifdef ENABLE_DISPLAY
  BaseType_t display_task1_status = xTaskCreate(task_display->timer_task, "DISPLAY_TASK_TIMER", 1024, (void *)task_display, 3, nullptr);
  BaseType_t display_task2_status = xTaskCreate(task_display->task_handler_task, "DISPLAY_TASK_HANDLER", 1024, (void *)task_display, 3, nullptr);
  BaseType_t display_gui_status = xTaskCreate(task_display->gui_task, "DISPLAY_GUI_TASK", 2048, (void *)task_display, 2, nullptr);
#endif
#ifdef ENABLE_USB
  BaseType_t usb_task_status = xTaskCreate(usb_task, "USB_TASK", 2048, nullptr, 1, nullptr);
#endif
  BaseType_t main_task_status = xTaskCreate(main_task, "MAIN_TASK", 2048, nullptr, 1, nullptr);

  assert(main_task_status == pdPASS && led_task_status == pdPASS && matrix_task_status == pdPASS && encoder_task_status == pdPASS
#ifdef ENABLE_USB
         && usb_task_status
#endif
#ifdef ENABLE_DISPLAY
         && display_task1_status == pdPASS && display_task2_status == pdPASS && display_gui_status == pdPASS
#endif
  );

  vTaskStartScheduler();

  while (true)
  {
    // forever
  }
}

void usb_task(void *unused)
{

  while (true)
  {
#ifdef ENABLE_USB
    usb_periodic();
#endif
    vTaskDelay(1);
  }
}

void set_simple_led(uint8_t led, uint8_t value, uint8_t mode = TaskLED::NORMAL, bool now = false)
{
  TaskLED::cmd_t cmd;
  cmd.cmd = TaskLED::LED_CMD_SET_SIMPLE_LED;
  cmd.led = led;
  cmd.value = value;
  cmd.mode = mode;
  cmd.update_now = now;
  xQueueSend(task_led->cmd_queue, &cmd, 0);
}

void set_led_interp_state(interp_t state)
{
  switch (state)
  {
  case INTERP_IDLE:
    set_simple_led(8, 0, TaskLED::NORMAL);
    set_simple_led(9, 0, TaskLED::NORMAL);
    set_simple_led(10, 64, TaskLED::NORMAL);
    set_simple_led(11, 0, TaskLED::NORMAL, true);
    break;
  case INTERP_READING:
    set_simple_led(8, 0, TaskLED::NORMAL);
    set_simple_led(9, 0, TaskLED::NORMAL);
    set_simple_led(10, 0, TaskLED::NORMAL);
    set_simple_led(11, 64, TaskLED::BLINK, true);
    break;
  case INTERP_PAUSED:
    set_simple_led(8, 0, TaskLED::NORMAL);
    set_simple_led(9, 64, TaskLED::BLINK);
    set_simple_led(10, 0, TaskLED::NORMAL);
    set_simple_led(11, 0, TaskLED::NORMAL, true);
    break;
  case INTERP_WAITING:
    set_simple_led(8, 0, TaskLED::NORMAL);
    set_simple_led(9, 0, TaskLED::NORMAL);
    set_simple_led(10, 0, TaskLED::NORMAL);
    set_simple_led(11, 64, TaskLED::NORMAL, true);
    break;
  default:
    // off
    set_simple_led(8, 0, TaskLED::NORMAL);
    set_simple_led(9, 0, TaskLED::NORMAL);
    set_simple_led(10, 0, TaskLED::NORMAL);
    set_simple_led(11, 0, TaskLED::NORMAL, true);
  }
}

void set_led_selected_increment(uint8_t increment)
{
  switch (increment)
  {
  case 1:
    set_simple_led(0, 32);
    set_simple_led(1, 0);
    set_simple_led(2, 0, TaskLED::NORMAL, true);
    break;
  case 2:
    set_simple_led(0, 0, TaskLED::NORMAL);
    set_simple_led(1, 32, TaskLED::NORMAL);
    set_simple_led(2, 0, TaskLED::NORMAL, true);
    break;
  case 3:
    set_simple_led(0, 0, TaskLED::NORMAL);
    set_simple_led(1, 0, TaskLED::NORMAL);
    set_simple_led(2, 32, TaskLED::NORMAL, true);
    break;
  default:
    set_simple_led(0, 0, TaskLED::NORMAL);
    set_simple_led(1, 0, TaskLED::NORMAL);
    set_simple_led(2, 0, TaskLED::NORMAL, true);
    break;
  }
}

void set_led_selected_axis(uint8_t axis)
{
  switch (axis)
  {
  case 1:
    set_simple_led(3, 32, TaskLED::NORMAL);
    set_simple_led(4, 0, TaskLED::NORMAL);
    set_simple_led(5, 0, TaskLED::NORMAL, true);
    break;
  case 2:
    set_simple_led(3, 0, TaskLED::NORMAL);
    set_simple_led(4, 32, TaskLED::NORMAL);
    set_simple_led(5, 0, TaskLED::NORMAL, true);
    break;
  case 3:
    set_simple_led(3, 0, TaskLED::NORMAL);
    set_simple_led(4, 0, TaskLED::NORMAL);
    set_simple_led(5, 32, TaskLED::NORMAL, true);
    break;
  default:
    set_simple_led(3, 0, TaskLED::NORMAL);
    set_simple_led(4, 0, TaskLED::NORMAL);
    set_simple_led(5, 0, TaskLED::NORMAL, true);
    break;
  }
}

void set_encoder_value(uint8_t encoder, int8_t value, bool smart = false)
{
  TaskEncoder::cmd_t cmd;
  cmd.cmd = smart ? TaskEncoder::ENCODER_CMD_SMART_SET_VALUE : TaskEncoder::ENCODER_CMD_SET_VALUE;
  cmd.encoder = encoder;
  cmd.value = value;

  xQueueSend(task_encoder->cmd_queue, &cmd, 0) == pdTRUE;
}

void main_task(void *unused)
{
  bool usb_in_pending = false;
  bool key_updated = false;
  uint8_t selected_axis = 1;
  uint8_t selected_increment = 2;
  uint8_t motion_command = 0;

  const uint32_t jog_increments[] = {0, 100, 10, 1};

  uint8_t keybuf[6] = {0, 0, 0, 0, 0, 0};
  uint8_t modifiers;

  usb_out_pkt last_out_pkt;

  bool machine_state_changed = false;
  bool machine_estop = true;
  bool machine_enabled = false;
  interp_t machine_interp_state = INTERP_OFF;

  mode_t machine_mode = MODE_UNKNOWN;

  bool initial_feedrate = false;
  bool initial_rapidrate = false;
  bool initial_maxvel = false;

  bzero(&last_out_pkt, sizeof(last_out_pkt));

  const uint32_t dot_colors[] = {
      WS2812::urgb_u32(0x3f, 0x1f, 0),
      WS2812::urgb_u32(0, 0x3f, 0x3f),
      WS2812::urgb_u32(0x3f, 0, 0x3f),
  };

  while (true)
  {
    #ifdef ENABLE_USB
    machine_state_changed = false;
    usb_out_pkt out_pkt;
    if (xQueueReceive(usb_out_queue, &out_pkt, 1) == pdTRUE)
    {
      usb_dump_out_pkt(&out_pkt);

      if (out_pkt.s.estop != machine_estop)
      {
        printf("machine_estop %d -> %d\n", machine_estop, out_pkt.s.estop);
        machine_estop = out_pkt.s.estop;
        machine_state_changed = true;
      }

      if (out_pkt.s.enabled != machine_enabled)
      {
        printf("machine_enabled %d -> %d\n", machine_enabled, out_pkt.s.enabled);
        machine_enabled = out_pkt.s.enabled;
        machine_state_changed = true;
      }

      if (out_pkt.s.mode != machine_mode)
      {
        printf("machine_mode %d -> %d\n", machine_mode, out_pkt.s.mode);
        machine_mode = (mode_t)out_pkt.s.mode;
        machine_state_changed = true;
      }

      if (out_pkt.s.interp_state != machine_interp_state)
      {
        printf("machine_interp_state %d -> %d\n", machine_interp_state, out_pkt.s.interp_state);
        machine_interp_state = (interp_t)out_pkt.s.interp_state;
        machine_state_changed = true;
      }

      if (!machine_estop && machine_enabled)
      {
        if (machine_state_changed || out_pkt.s.feedrate_override != last_out_pkt.s.feedrate_override && !initial_feedrate)
        {
          uint32_t feedrate_segments = (out_pkt.s.feedrate_override * 1000) / 7142;
          set_encoder_value(1, feedrate_segments, false);
          printf("feedrate_segments=%d\n", feedrate_segments);
          initial_feedrate = true;
        }
        if (machine_state_changed || out_pkt.s.rapidrate_override != last_out_pkt.s.rapidrate_override && !initial_rapidrate)
        {
          uint32_t rapidrate_segments = (out_pkt.s.rapidrate_override * 1000) / 7142;
          set_encoder_value(2, rapidrate_segments, false);
          printf("rapidrate_segments=%d\n", rapidrate_segments);
          initial_rapidrate = true;
        }
        if (machine_state_changed || out_pkt.s.maxvel_override != last_out_pkt.s.maxvel_override && !initial_maxvel)
        {
          uint32_t maxvel_segments = (out_pkt.s.maxvel_override * 1000) / 7142;
          set_encoder_value(3, maxvel_segments, false);
          printf("maxvel_segments=%d\n", maxvel_segments);
          initial_maxvel = true;
        }

        if ((machine_mode == MODE_AUTO || machine_mode == MODE_TELEOP) && machine_state_changed)
        {
          // in MODE_AUTO show the current interp_state on the start/stop/pause/step LEDs
          set_led_interp_state(machine_interp_state);
        }
        else if (machine_state_changed)
        {
          // not in MODE_AUTO so shut off the start/stop/pause/step LEDs
          set_led_interp_state(INTERP_OFF);
        }
      }
      else
      {
        // machine is in estop or disabled, turn off the start/stop/pause/step LEDs
        set_led_interp_state(INTERP_OFF);
        initial_feedrate = false;
        initial_rapidrate = false;
        initial_maxvel = false;
      }

      memcpy(&last_out_pkt, &out_pkt, sizeof(out_pkt));
    }

    #endif


    TaskEncoder::event_t enc_evt;
    if (xQueueReceive(task_encoder->event_queue, &enc_evt, 1) == pdTRUE)
    {
      printf("encoder=%d value=%d\n", enc_evt.encoder, enc_evt.value);

      if (enc_evt.encoder >= 1 && enc_evt.encoder <= 3)
      {
        TaskLED::cmd_t cmd;
        cmd.cmd = TaskLED::LED_CMD_SET_RING;
        cmd.led = enc_evt.encoder - 1;
        cmd.value = enc_evt.value;
        cmd.color = dot_colors[enc_evt.encoder - 1];
        cmd.update_now = true;
        xQueueSend(task_led->cmd_queue, &cmd, 0);
      }

#ifdef ENABLE_DISPLAY
      TaskDisplay::cmd_t cmd;
      cmd.cmd = TaskDisplay::DISPLAY_CMD_UPDATE_ENCODER;
      cmd.encoder = enc_evt.encoder;
      cmd.value = enc_evt.value;
      xQueueSend(task_display->cmd_queue, &cmd, 0);
#endif

      usb_in_pending = true;
    }

    TaskMatrix::event_t mtx_evt;
    if (xQueueReceive(task_matrix->event_queue, &mtx_evt, 1) == pdTRUE)
    {

      printf("evt=%02x %s %s\n", mtx_evt.code, mtx_evt.press ? "press" : "release", mtx_evt.gpio ? "gpio" : "key");

#ifdef ENABLE_DISPLAY
      TaskDisplay::cmd_t cmd;
      cmd.cmd = TaskDisplay::DISPLAY_CMD_UPDATE_KEY;
      cmd.code = mtx_evt.code;
      cmd.press = mtx_evt.press;
      xQueueSend(task_display->cmd_queue, &cmd, 0);
#endif

      if (mtx_evt.gpio && mtx_evt.press)
      {
        uint8_t enc;
        if (TaskMatrix::led_encoder_map(mtx_evt.code, enc))
        {
          set_encoder_value(enc + 1, 0, true);
        }
      }
      else if (!mtx_evt.gpio)
      {
        if (!machine_estop && machine_enabled && (machine_mode == MODE_AUTO || machine_mode == MODE_TELEOP))
        {
          if (mtx_evt.code == 0x33)
          {
            if (mtx_evt.press)
              motion_command |= 0x08;
            else
              motion_command &= ~0x08;
            usb_in_pending = true;
          }
          else if (mtx_evt.code == 0x34)
          {
            if (mtx_evt.press)
              motion_command |= 0x04;
            else
              motion_command &= ~0x04;
            usb_in_pending = true;
          }
          else if (mtx_evt.code == 0x35)
          {
            if (mtx_evt.press)
              motion_command |= 0x02;
            else
              motion_command &= ~0x02;
            usb_in_pending = true;
          }
          else if (mtx_evt.code == 0x36)
          {
            if (mtx_evt.press)
              motion_command |= 0x01;
            else
              motion_command &= ~0x01;
            usb_in_pending = true;
          }
        }

        if (!machine_estop && machine_enabled && machine_mode == MODE_MANUAL && mtx_evt.press)
        {
          if (mtx_evt.code == 0x2f)
          {
            selected_increment = 1;
            usb_in_pending = true;
          }
          else if (mtx_evt.code == 0x30)
          {
            selected_increment = 2;
            usb_in_pending = true;
          }
          else if (mtx_evt.code == 0x37)
          {
            selected_increment = 3;
            usb_in_pending = true;
          }
          else if (mtx_evt.code == 0x38)
          {
            selected_axis = 1;
            usb_in_pending = true;
          }
          else if (mtx_evt.code == 0x39)
          {
            selected_axis = 2;
            usb_in_pending = true;
          }
          else if (mtx_evt.code == 0x3a)
          {
            selected_axis = 3;
            usb_in_pending = true;
          }
        }

        uint8_t hid_keycode = 0;
        bool found = TaskMatrix::hid_keycode(mtx_evt.code, hid_keycode, modifiers);
        if (found && mtx_evt.press)
        {
          TaskMatrix::hid_n_key_buf_add(keybuf, hid_keycode);
          key_updated = true;
        }
        else if (found && !mtx_evt.press)
        {
          TaskMatrix::hid_n_key_buf_remove(keybuf, hid_keycode);
          key_updated = true;
        }
      }
    }

#ifdef ENABLE_USB
  if (machine_state_changed || usb_in_pending)
  {
    if (!machine_estop && machine_enabled && machine_mode == MODE_MANUAL)
    {
      printf("enabled and in manual mode selected_increment=%d selected_axis=%d\n", selected_increment, selected_axis);
      // show the selected increment and axis on the LEDs if enabled and in MODE_MANUAL
      set_led_selected_increment(selected_increment);
      set_led_selected_axis(selected_axis);
    }
    else
    {
      // otherwise turn off the increment and axis LEDs
      set_led_selected_increment(0);
      set_led_selected_axis(0);
    }
  }
  if (usb_in_pending)
    {
      usb_in_pkt pkt;
      pkt.s.knob1 = task_encoder->get_value(1);
      pkt.s.knob2 = task_encoder->get_value(2);
      pkt.s.knob3 = task_encoder->get_value(3);
      pkt.s.axis = selected_axis;
      pkt.s.step = jog_increments[selected_increment];
      pkt.s.shuttle = task_encoder->get_value(4, false);
      pkt.s.jog = task_encoder->get_value(0);
      pkt.s.motion_cmd = motion_command;

      tud_hid_report(0, &pkt, sizeof(pkt));
      usb_in_pending = false;
    }

    if (tud_hid_n_ready(ITF_KEYBOARD))
    {
      if (key_updated)
      {
        if (TaskMatrix::hid_n_key_buf_is_empty(keybuf))
        {
          modifiers = 0;
        }

        tud_hid_n_keyboard_report(ITF_KEYBOARD, 0, modifiers, keybuf);

        key_updated = false;
      }
    }
#endif
  }
}
