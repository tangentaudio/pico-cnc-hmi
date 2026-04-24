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
#include <timers.h>
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
  BaseType_t display_gui_status = xTaskCreate(task_display->gui_task, "DISPLAY_GUI_TASK", 2048, (void *)task_display, 3, nullptr);
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
         && display_gui_status == pdPASS
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

void set_ring_led(uint8_t ring, uint8_t value, uint32_t color = 0xFF0000, bool now = false)
{
  TaskLED::cmd_t cmd;
  cmd.cmd = TaskLED::LED_CMD_SET_RING;
  cmd.led = ring;
  cmd.value = value;
  cmd.color = color;
  cmd.update_now = now;
  xQueueSend(task_led->cmd_queue, &cmd, 0);
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

void set_led_interp_state(interp_t state, bool step_mode = false, bool paused = false)
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
    if (step_mode)
    {
      // Single-step: blink when waiting for input (paused), solid when executing.
      TaskLED::modes m = paused ? TaskLED::BLINK : TaskLED::NORMAL;
      set_simple_led(8, 64, m);
      set_simple_led(9, 0, TaskLED::NORMAL);
      set_simple_led(10, 0, TaskLED::NORMAL);
      set_simple_led(11, 64, m, true);
    }
    else
    {
      // Normal run: program executing — run LED solid.
      set_simple_led(8, 0, TaskLED::NORMAL);
      set_simple_led(9, 0, TaskLED::NORMAL);
      set_simple_led(10, 0, TaskLED::NORMAL);
      set_simple_led(11, 64, TaskLED::NORMAL, true);
    }
    break;
  case INTERP_PAUSED:
    if (step_mode)
    {
      // Single-step: blink when waiting for input (paused), solid when executing.
      TaskLED::modes m = paused ? TaskLED::BLINK : TaskLED::NORMAL;
      set_simple_led(8, 64, m);
      set_simple_led(9, 0, TaskLED::NORMAL);
      set_simple_led(10, 0, TaskLED::NORMAL);
      set_simple_led(11, 64, m, true);
    }
    else
    {
      // Mid-run pause (AUTO_PAUSE) — pause LED blink.
      set_simple_led(8, 0, TaskLED::NORMAL);
      set_simple_led(9, 64, TaskLED::BLINK);
      set_simple_led(10, 0, TaskLED::NORMAL);
      set_simple_led(11, 0, TaskLED::NORMAL, true);
    }
    break;
  case INTERP_WAITING:
    if (step_mode)
    {
      // Single-step: blink when waiting for input (paused), solid when executing.
      TaskLED::modes m = paused ? TaskLED::BLINK : TaskLED::NORMAL;
      set_simple_led(8, 64, m);
      set_simple_led(9, 0, TaskLED::NORMAL);
      set_simple_led(10, 0, TaskLED::NORMAL);
      set_simple_led(11, 64, m, true);
    }
    else
    {
      // Normal run: briefly waiting for motion queue to drain — run LED stays solid.
      set_simple_led(8, 0, TaskLED::NORMAL);
      set_simple_led(9, 0, TaskLED::NORMAL);
      set_simple_led(10, 0, TaskLED::NORMAL);
      set_simple_led(11, 64, TaskLED::NORMAL, true);
    }
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

  if (xQueueSend(task_encoder->cmd_queue, &cmd, pdMS_TO_TICKS(2)) != pdTRUE)
  {
    printf("encoder cmd queue full: encoder=%d value=%d smart=%d\n", encoder, value, smart ? 1 : 0);
  }
}

volatile bool g_machine_alive = false;
  

#ifdef ENABLE_DISPLAY
// Pack current machine state into a display command and enqueue it.
static inline void display_send_machine_state(
    bool estop, bool enabled, bool homed, bool coolant,
    mode_t mode, interp_t interp, bool step_mode, bool paused,
    uint8_t feed_seg, uint8_t rapid_seg, uint8_t maxvel_seg)
{
  TaskDisplay::cmd_t dcmd;
  dcmd.cmd                  = estop ? TaskDisplay::DISPLAY_CMD_ESTOP
                                    : TaskDisplay::DISPLAY_CMD_MACHINE_STATE;
  dcmd.state.estop          = estop;
  dcmd.state.enabled        = enabled;
  dcmd.state.homed          = homed;
  dcmd.state.coolant        = coolant;
  dcmd.state.mode           = mode;
  dcmd.state.interp_state   = interp;
  dcmd.state.step_mode      = step_mode;
  dcmd.state.paused         = paused;
  dcmd.state.feed_seg       = feed_seg;
  dcmd.state.rapid_seg      = rapid_seg;
  dcmd.state.maxvel_seg     = maxvel_seg;
  xQueueSend(task_display->cmd_queue, &dcmd, 0);
}
#endif

void main_task(void *unused)
{
  bool usb_in_pending = false;
  bool key_updated = false;
  uint8_t selected_axis = 1;
  uint8_t selected_increment = 2;
  uint8_t motion_command = 0;

  const uint32_t jog_increments[] = {0, 100, 10, 1};

  uint8_t keybuf[6] = {0, 0, 0, 0, 0, 0};
  uint8_t modifiers = 0;
  bool sec_a_shift = false;          // true while Section A wide modifier key is held
  bool sec_a_shift_used = false;     // true if another key was pressed while shift held (tap-vs-hold)
  bool pending_space_release = false;  // auto-release Space on next iteration after tap
  uint8_t pressed_hid[0x60] = {};   // maps matrix code -> HID keycode sent on press (for correct release)

  usb_out_pkt last_out_pkt;

  bool machine_state_changed = false;
  bool machine_estop = true;
  bool machine_enabled = false;
  interp_t machine_interp_state = INTERP_OFF;
  bool machine_step_mode = false;
  bool machine_paused = false;
  bool machine_coolant = false;
  bool machine_optional_stop = false;
  bool machine_homed = false;
  float g_machine_pos[3] = {0.0f, 0.0f, 0.0f};
  bool  last_jog_continuous = false;  // set by most recent enc 0/4 event
  bool  shuttle_active      = false;  // true while shuttle is held at non-zero position

  TickType_t slow_blink_last = 0;
  bool slow_blink_on = false;

  mode_t machine_mode = MODE_UNKNOWN;

  bool initial_feedrate = false;
  bool initial_rapidrate = false;
  bool initial_maxvel = false;

  // Per-encoder flag: set when the user physically moves (or button-toggles) a
  // knob.  While false, host feedback changes are re-synced immediately to the
  // encoder so that LinuxCNC's startup settling packets don't leave the encoder
  // stuck at a transient 0 value.  Reset to false at each reconnect.
  bool enc_user_moved[4] = {false, false, false, false};

  // STOP + CYCLE START hold-to-home chord state.
  // Active only when: !machine_estop && machine_enabled && !machine_homed.
  // Sequence: press STOP, then press START while STOP held, hold both ≥ 2s → HOME ALL.
  bool       chord_stop_held   = false;  // STOP (0x35) currently pressed
  bool       chord_start_held  = false;  // START (0x36) currently pressed while STOP held
  bool       chord_fired       = false;  // chord completed; suppress release bits
  TickType_t chord_start_time  = 0;      // tick when both STOP+START were first held together

  bzero(&last_out_pkt, sizeof(last_out_pkt));

  const uint32_t dot_colors[] = {
      WS2812::urgb_u32(0x3f, 0x1f, 0),
      WS2812::urgb_u32(0, 0x3f, 0x3f),
      WS2812::urgb_u32(0x3f, 0, 0x3f),
  };

  TimerHandle_t hbeat_timer = xTimerCreate("hbeat_timer", pdMS_TO_TICKS(1000), pdFALSE, (void *)0, [](TimerHandle_t xTimer) {
    g_machine_alive = false;
  });

  while (true)
  {
    if (g_machine_alive) {
      #ifdef ENABLE_USB
      machine_state_changed = false;
      usb_out_pkt out_pkt;
      if (xQueueReceive(usb_out_queue, &out_pkt, 1) == pdTRUE)
      {
        usb_dump_out_pkt(&out_pkt);

        if (out_pkt.s.heartbeat != last_out_pkt.s.heartbeat)
        {
          xTimerReset(hbeat_timer, 0);
          g_machine_alive = true;
          if (usb_out_queue_drops > 0) {
            printf("usb: %lu OUT packets dropped\n", (unsigned long)usb_out_queue_drops);
            usb_out_queue_drops = 0;
          }
        }

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

        if (out_pkt.s.step_mode != machine_step_mode)
        {
          printf("machine_step_mode %d -> %d\n", machine_step_mode, out_pkt.s.step_mode);
          machine_step_mode = out_pkt.s.step_mode;
          machine_state_changed = true;
        }

        if (out_pkt.s.paused != machine_paused)
        {
          printf("machine_paused %d -> %d\n", machine_paused, out_pkt.s.paused);
          machine_paused = out_pkt.s.paused;
          machine_state_changed = true;
        }

        if (out_pkt.s.coolant != machine_coolant)
        {
          printf("machine_coolant %d -> %d\n", machine_coolant, out_pkt.s.coolant);
          machine_coolant = out_pkt.s.coolant;
          machine_state_changed = true;
        }

        if (out_pkt.s.optional_stop != machine_optional_stop)
        {
          printf("machine_optional_stop %d -> %d\n", machine_optional_stop, out_pkt.s.optional_stop);
          machine_optional_stop = out_pkt.s.optional_stop;
          machine_state_changed = true;
        }

        if (out_pkt.s.homed != machine_homed)
        {
          printf("machine_homed %d -> %d\n", machine_homed, out_pkt.s.homed);
          machine_homed = out_pkt.s.homed;
          machine_state_changed = true;
        }

        // Update cached axis positions for jog overlay display.
        bool pos_changed = (out_pkt.s.pos_x != last_out_pkt.s.pos_x ||
                            out_pkt.s.pos_y != last_out_pkt.s.pos_y ||
                            out_pkt.s.pos_z != last_out_pkt.s.pos_z);
        g_machine_pos[0] = out_pkt.s.pos_x * (1.0f / 10000.0f);
        g_machine_pos[1] = out_pkt.s.pos_y * (1.0f / 10000.0f);
        g_machine_pos[2] = out_pkt.s.pos_z * (1.0f / 10000.0f);
#ifdef ENABLE_DISPLAY
        // Only push position feedback while the shuttle is actively at a non-zero
        // position. This ensures the overlay timer is driven by physical user input
        // (encoder events), not by position feedback that may trickle in after the
        // shuttle returns to neutral due to simulator settling or following-error noise.
        if (pos_changed && shuttle_active &&
            !machine_estop && machine_enabled && machine_mode == MODE_MANUAL) {
          // printf("[main] pos-feedback JOG: shuttle_active=%d pos=%.4f,%.4f,%.4f\n",
          //     (int)shuttle_active, (double)g_machine_pos[0], (double)g_machine_pos[1], (double)g_machine_pos[2]);
          // DISPLAY_CMD_JOG disabled
          if (false) {
          TaskDisplay::cmd_t dcmd;
          dcmd.cmd            = TaskDisplay::DISPLAY_CMD_JOG;
          dcmd.jog_pos[0]     = g_machine_pos[0];
          dcmd.jog_pos[1]     = g_machine_pos[1];
          dcmd.jog_pos[2]     = g_machine_pos[2];
          dcmd.jog_axis       = selected_axis;
          dcmd.jog_continuous = true;
          xQueueSend(task_display->cmd_queue, &dcmd, 0);
          } // DISPLAY_CMD_JOG disabled
        }
#endif

        // Keep override encoders synchronized to host state even when disabled/estop,
        // so startup defaults are reflected on the panel immediately.
        bool feedrate_changed = (out_pkt.s.feedrate_override != last_out_pkt.s.feedrate_override);
        bool rapidrate_changed = (out_pkt.s.rapidrate_override != last_out_pkt.s.rapidrate_override);
        bool maxvel_changed = (out_pkt.s.maxvel_override != last_out_pkt.s.maxvel_override);

        // Update ring LED displays from OUT packet feedback (host-authoritative).
        // Also re-sync encoder position while the user hasn't physically moved it,
        // so that LinuxCNC's startup settling sequence (e.g. 0→7, 0→14) doesn't
        // leave the encoder stuck at a transient value and waste the first button press.
        if (!initial_feedrate || machine_state_changed || feedrate_changed)
        {
          uint32_t feedrate_segments = out_pkt.s.feedrate_override;
          if (!enc_user_moved[1])
            task_encoder->sync_value_immediate(1, feedrate_segments);
          set_ring_led(0, feedrate_segments, dot_colors[0], false);
          initial_feedrate = true;
        }
        if (!initial_rapidrate || machine_state_changed || rapidrate_changed)
        {
          uint32_t rapidrate_segments = out_pkt.s.rapidrate_override;
          if (!enc_user_moved[2])
            task_encoder->sync_value_immediate(2, rapidrate_segments);
          set_ring_led(1, rapidrate_segments, dot_colors[1], false);
          initial_rapidrate = true;
        }
        if (!initial_maxvel || machine_state_changed || maxvel_changed)
        {
          uint32_t maxvel_segments = out_pkt.s.maxvel_override;
          if (!enc_user_moved[3])
            task_encoder->sync_value_immediate(3, maxvel_segments);
          set_ring_led(2, maxvel_segments, dot_colors[2], false);
          initial_maxvel = true;
        }
        // Flush all ring LED updates atomically — avoids intermediate-state blips
        // from three separate full-strip refreshes.
        {
          TaskLED::cmd_t fcmd;
          fcmd.cmd = TaskLED::LED_CMD_FLUSH;
          xQueueSend(task_led->cmd_queue, &fcmd, 0);
        }

        // Refresh interp LEDs on state changes, OR continuously in step mode.
        // In step mode, fast PAUSED→READING→PAUSED transitions may be invisible
        // at the host poll rate, so we re-apply on every packet to keep the
        // blink/solid state (driven by paused) responsive.
        if (!machine_estop && machine_enabled
            && (machine_state_changed || machine_step_mode))
        {
          if (!machine_homed)
          {
            // Not homed: clear program LEDs; STOP LED slow-blinks via tick loop below.
            set_simple_led(8, 0, TaskLED::NORMAL);
            set_simple_led(9, 0, TaskLED::NORMAL);
            set_simple_led(11, 0, TaskLED::NORMAL, true);
            // Start blink phase lit so feedback is immediate.
            slow_blink_last = xTaskGetTickCount();
            slow_blink_on = true;
            set_simple_led(10, 32, TaskLED::NORMAL, true);
          }
          else
          {
            // Homed: normal interp state presentation.
            set_led_interp_state(machine_interp_state, machine_step_mode, machine_paused);
          }
        }
        else if (machine_state_changed)
        {
          // machine is in estop or disabled, turn off the start/stop/pause/step LEDs
          set_led_interp_state(INTERP_OFF);
        }

        if (machine_state_changed)
        {
          // M1 and Coolant LEDs always reflect host state.
          // LED 6 = M1 Optional Stop (key 0x3d), LED 7 = Coolant (key 0x3e).
          set_simple_led(6, machine_optional_stop ? 64 : 0, TaskLED::NORMAL);
          set_simple_led(7, machine_coolant ? 64 : 0, TaskLED::NORMAL, true);
        }

        // Send updated machine state to display whenever anything changed,
        // including override value changes (which are not part of machine_state_changed).
        if (machine_state_changed || feedrate_changed || rapidrate_changed || maxvel_changed) {
#ifdef ENABLE_DISPLAY
          display_send_machine_state(
              machine_estop, machine_enabled, machine_homed, machine_coolant,
              machine_mode, machine_interp_state, machine_step_mode, machine_paused,
              out_pkt.s.feedrate_override, out_pkt.s.rapidrate_override, out_pkt.s.maxvel_override);
#endif
        }

        memcpy(&last_out_pkt, &out_pkt, sizeof(out_pkt));
      }

      // Not-homed STOP LED blink.  Normally slow (~0.5 Hz).  Accelerates to
      // fast (~4 Hz) while the STOP+START home-all chord is being held.
      // Also checks whether the chord has been held for 2 s and fires if so.
      if (!machine_estop && machine_enabled && !machine_homed)
      {
        TickType_t now_ticks = xTaskGetTickCount();
        bool in_chord = chord_stop_held && chord_start_held && !chord_fired;

        if (in_chord)
        {
          // Check for 2-second hold completion.
          if ((now_ticks - chord_start_time) >= pdMS_TO_TICKS(2000))
          {
            printf("chord: HOME ALL fired\n");
            motion_command |= 0x40;  // HOME_ALL bit — decoded by hmi.py
            usb_in_pending  = true;
            chord_fired = true;
            // Brief full-brightness flash on both STOP and RUN LEDs as confirmation.
            set_simple_led(10, 64, TaskLED::NORMAL);
            set_simple_led(11, 64, TaskLED::NORMAL, true);
          }
          else
          {
            // Fast blink (~4 Hz = 125 ms half-period) during countdown.
            if ((now_ticks - slow_blink_last) >= pdMS_TO_TICKS(125))
            {
              slow_blink_last = now_ticks;
              slow_blink_on = !slow_blink_on;
              set_simple_led(10, slow_blink_on ? 32 : 0, TaskLED::NORMAL, true);
            }
          }
        }
        else
        {
          // Normal slow blink (~0.5 Hz = 1000 ms half-period).
          if ((now_ticks - slow_blink_last) >= pdMS_TO_TICKS(1000))
          {
            slow_blink_last = now_ticks;
            slow_blink_on = !slow_blink_on;
            set_simple_led(10, slow_blink_on ? 32 : 0, TaskLED::NORMAL, true);
          }
        }
      }

      #endif


      TaskEncoder::event_t enc_evt;
      if (xQueueReceive(task_encoder->event_queue, &enc_evt, 1) == pdTRUE)
      {
        if (enc_evt.encoder >= 1 && enc_evt.encoder <= 3)
          printf("knob enc=%d val=%d\n", enc_evt.encoder, enc_evt.value);

        // For encoders 1-3 (override rings), suppress ring LED updates from encoder events.
        // The display is host-authoritative (follows LinuxCNC feedback from OUT packets),
        // not physical encoder position. This prevents flicker from competing sources.
        // Encoder events still trigger USB IN packets (user input), but LED updates
        // only come from OUT packets (host feedback).

  #ifdef ENABLE_DISPLAY
        {
          TaskDisplay::cmd_t dcmd;
          // Encoders 1-3 are the override knobs; encoder 0 is jog.
          if (enc_evt.encoder >= 1 && enc_evt.encoder <= 3) {
            dcmd.cmd        = TaskDisplay::DISPLAY_CMD_KNOB;
            dcmd.knob_index = enc_evt.encoder - 1;  // 0=feed,1=rapid,2=maxvel
            // Read the current segment value directly from the encoder task.
            dcmd.knob_seg   = (uint8_t)task_encoder->get_value(enc_evt.encoder);
          } else {
            bool continuous = (enc_evt.encoder == 4); // enc 0 = incremental, enc 4 = shuttle
            if (continuous) {
              shuttle_active = (enc_evt.value != 0); // enc_evt.value is raw -7..+7, 0 = neutral
              // printf("[main] shuttle enc value=%d shuttle_active=%d\n", enc_evt.value, (int)shuttle_active);
            }
            last_jog_continuous = continuous;
            // DISPLAY_CMD_JOG disabled
            dcmd.cmd            = TaskDisplay::DISPLAY_CMD_JOG;
            dcmd.jog_pos[0]     = g_machine_pos[0];
            dcmd.jog_pos[1]     = g_machine_pos[1];
            dcmd.jog_pos[2]     = g_machine_pos[2];
            dcmd.jog_axis       = selected_axis;
            dcmd.jog_continuous = continuous;
          }
          if (false && xQueueSend(task_display->cmd_queue, &dcmd, 0) != pdTRUE) {
            printf("[main] display queue full, cmd dropped (cmd=%d)\n", (int)dcmd.cmd);
          }
        }
  #endif

        // Once the user physically rotates or button-toggles an override knob,
        // stop re-syncing that encoder from host feedback.
        if (enc_evt.encoder >= 1 && enc_evt.encoder <= 3)
          enc_user_moved[enc_evt.encoder] = true;

        usb_in_pending = true;
      }  // if (xQueueReceive(task_encoder...))

      // Auto-release Space after a shift-tap.  The press report was sent on
      // the previous iteration; now remove it and send the release report.
      if (pending_space_release) {
        TaskMatrix::hid_n_key_buf_remove(keybuf, 0x2C);
        key_updated = true;
        pending_space_release = false;
      }

      TaskMatrix::event_t mtx_evt;
      if (xQueueReceive(task_matrix->event_queue, &mtx_evt, 1) == pdTRUE)
      {

        printf("evt=%02x %s %s\n", mtx_evt.code, mtx_evt.press ? "press" : "release", mtx_evt.gpio ? "gpio" : "key");

  #ifdef ENABLE_DISPLAY
        {
          TaskDisplay::cmd_t dcmd;
          dcmd.cmd       = TaskDisplay::DISPLAY_CMD_KEY;
          dcmd.key_code  = mtx_evt.code;
          dcmd.key_press = mtx_evt.press;
          xQueueSend(task_display->cmd_queue, &dcmd, 0);
        }
  #endif

        if (mtx_evt.gpio && mtx_evt.press)
        {
          uint8_t enc;
          if (TaskMatrix::led_encoder_map(mtx_evt.code, enc))
          {
            // Toggle between 0 and the configured 100% segment.
            // 100% segment is derived from hmi_config (same source as hmi.py).
            uint8_t cur = (uint8_t)task_encoder->get_value(enc + 1);
            uint8_t reset_seg;
            if      (enc == 0) reset_seg = hmi_config.valid ? hmi_feed_reset_seg()  : 14;
            else if (enc == 1) reset_seg = hmi_config.valid ? hmi_rapid_reset_seg() : 14;
            else               reset_seg = 14;
            uint8_t target = cur == 0 ? reset_seg : 0;
            printf("knob btn: code=0x%02x enc=%d cur=%d target=%d\n", mtx_evt.code, enc, cur, target);
            // Use SET_VALUE_NOTIFY so the encoder task fires an event after
            // applying the value.  The event triggers usb_in_pending in the
            // encoder event handler, ensuring get_value() returns the new value
            // when the IN packet is formed.  Setting usb_in_pending directly
            // here would send stale values because get_value() reads the old
            // state before the async SET_VALUE is processed by the encoder task.
            {
              TaskEncoder::cmd_t ecmd;
              ecmd.cmd = TaskEncoder::ENCODER_CMD_SET_VALUE_NOTIFY;
              ecmd.encoder = enc + 1;
              ecmd.value = target;
              if (xQueueSend(task_encoder->cmd_queue, &ecmd, pdMS_TO_TICKS(2)) != pdTRUE)
              {
                printf("encoder cmd queue full: enc=%d val=%d\n", enc + 1, target);
              }
            }
          }
        }
        else if (!mtx_evt.gpio)
        {
          if (!machine_estop && machine_enabled)
          {
            // Cycle commands: gate only on estop/enabled.
            // LinuxCNC rejects c.auto() calls when not in AUTO/TELEOP mode,
            // so let hmi.py pass the command through and let LinuxCNC handle it.
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
            else if (mtx_evt.code == 0x35)  // STOP
            {
              if (!machine_homed)
              {
                // In not-homed state, intercept STOP for chord detection.
                // Do not send the STOP bit to LinuxCNC (nothing to abort).
                if (mtx_evt.press)
                {
                  chord_stop_held = true;
                  chord_fired     = false;
                }
                else
                {
                  // STOP released — cancel chord and restore RUN LED.
                  chord_stop_held  = false;
                  chord_start_held = false;
                  chord_fired      = false;
                  set_simple_led(11, 0, TaskLED::NORMAL, true);
                  // Reset blink timer so slow-blink resumes cleanly.
                  slow_blink_last = xTaskGetTickCount() - pdMS_TO_TICKS(1000);
                }
              }
              else
              {
                // Normal STOP behavior when homed.
                if (mtx_evt.press)
                  motion_command |= 0x02;
                else
                  motion_command &= ~0x02;
                usb_in_pending = true;
              }
            }
            else if (mtx_evt.code == 0x36)  // CYCLE START
            {
              if (!machine_homed && chord_stop_held)
              {
                // In not-homed state with STOP already held: join the chord.
                if (mtx_evt.press && !chord_fired)
                {
                  chord_start_held = true;
                  chord_start_time = xTaskGetTickCount();
                  // Dim RUN LED to signal chord countdown has begun.
                  set_simple_led(11, 16, TaskLED::NORMAL, true);
                  printf("chord: STOP+START held, counting...\n");
                }
                else if (!mtx_evt.press)
                {
                  chord_start_held = false;
                  if (!chord_fired)
                    set_simple_led(11, 0, TaskLED::NORMAL, true);  // cancelled
                  chord_fired = false;
                }
              }
              else
              {
                // Normal CYCLE START behavior when homed or STOP not held.
                if (mtx_evt.press)
                  motion_command |= 0x01;
                else
                  motion_command &= ~0x01;
                usb_in_pending = true;
              }
            }
            else if (mtx_evt.code == 0x3d)
            {
              if (mtx_evt.press)
                motion_command |= 0x10;
              else
                motion_command &= ~0x10;
              usb_in_pending = true;
            }
            else if (mtx_evt.code == 0x3e)
            {
              if (mtx_evt.press)
                motion_command |= 0x20;
              else
                motion_command &= ~0x20;
              usb_in_pending = true;
            }
          }

          // Jog controls (axis/increment buttons) are active in:
          //   MANUAL — always
          //   MDI    — always (commands are short, user may want to jog after)
          //   AUTO   — only when interp is IDLE (program finished/stopped)
          // During an active AUTO program the buttons stay disabled.
          bool jog_controls_active = !machine_estop && machine_enabled
              && (machine_mode == MODE_MANUAL
                  || machine_mode == MODE_MDI
                  || (machine_mode == MODE_AUTO && machine_interp_state == INTERP_IDLE));
          if (jog_controls_active && mtx_evt.press)
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

#ifdef ENABLE_DISPLAY
            // Send transient overlay for increment/axis selection changes.
            bool is_inc  = (mtx_evt.code == 0x2f || mtx_evt.code == 0x30 || mtx_evt.code == 0x37);
            bool is_axis = (mtx_evt.code == 0x38 || mtx_evt.code == 0x39 || mtx_evt.code == 0x3a);
            if (is_inc || is_axis) {
              TaskDisplay::cmd_t dcmd;
              dcmd.cmd = TaskDisplay::DISPLAY_CMD_JOG_SELECT;
              dcmd.jog_select_is_axis = is_axis;
              dcmd.jog_axis      = selected_axis;
              dcmd.jog_increment = selected_increment;
              xQueueSend(task_display->cmd_queue, &dcmd, 0);
            }
#endif
          }

          if (TaskMatrix::is_section_a_shift(mtx_evt.code)) {
            // Wide modifier key: tap sends Space, hold acts as shift.
            if (mtx_evt.press) {
              sec_a_shift = true;
              sec_a_shift_used = false;
            } else {
              sec_a_shift = false;
              if (!sec_a_shift_used) {
                // Tap: shift was released without being used as a modifier.
                // Emit a Space keystroke (HID 0x2C).
                TaskMatrix::hid_n_key_buf_add(keybuf, 0x2C);
                key_updated = true;
                pending_space_release = true;  // auto-release next iteration
              }
            }
          } else {
            // If shift is held and another key is pressed, mark shift as used.
            if (sec_a_shift && mtx_evt.press)
              sec_a_shift_used = true;

            uint8_t key_hid = 0;
            bool found = false;
            if (sec_a_shift)
              found = TaskMatrix::hid_keycode_shifted(mtx_evt.code, key_hid, modifiers);
            if (!found)
              found = TaskMatrix::hid_keycode(mtx_evt.code, key_hid, modifiers);
            if (found && mtx_evt.press)
            {
              if (mtx_evt.code < sizeof(pressed_hid))
                pressed_hid[mtx_evt.code] = key_hid;
              TaskMatrix::hid_n_key_buf_add(keybuf, key_hid);
              key_updated = true;
            }
            else if (found && !mtx_evt.press)
            {
              // Use the keycode recorded at press time so release always matches,
              // regardless of whether the shift modifier state has changed.
              uint8_t remove_hid = (mtx_evt.code < sizeof(pressed_hid) && pressed_hid[mtx_evt.code] != 0)
                                   ? pressed_hid[mtx_evt.code] : key_hid;
              if (mtx_evt.code < sizeof(pressed_hid))
                pressed_hid[mtx_evt.code] = 0;
              TaskMatrix::hid_n_key_buf_remove(keybuf, remove_hid);
              key_updated = true;
            }
          }
        }
      }

  #ifdef ENABLE_USB
    if (machine_state_changed || usb_in_pending)
    {
      bool jog_leds_active = !machine_estop && machine_enabled
          && (machine_mode == MODE_MANUAL
              || machine_mode == MODE_MDI
              || (machine_mode == MODE_AUTO && machine_interp_state == INTERP_IDLE));
      if (jog_leds_active)
      {
        // Show the selected increment and axis LEDs when jog controls are
        // active.  The user needs to see the current jog configuration
        // before touching the jog wheel (which auto-switches to MANUAL).
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
    if (usb_in_pending && initial_feedrate && initial_rapidrate && initial_maxvel)
      {
        usb_in_pkt pkt;
        bzero(&pkt, sizeof(pkt));
        pkt.s.knob1 = task_encoder->get_value(1);
        pkt.s.knob2 = task_encoder->get_value(2);
        pkt.s.knob3 = task_encoder->get_value(3);
        pkt.s.axis = selected_axis;
        pkt.s.step = jog_increments[selected_increment];
        pkt.s.shuttle = task_encoder->get_value(4, false);
        pkt.s.jog = task_encoder->get_value(0);
        pkt.s.motion_cmd = motion_command;

        printf("IN pkt: k1=%d k2=%d k3=%d\n", pkt.s.knob1, pkt.s.knob2, pkt.s.knob3);

        if (tud_hid_n_ready(ITF_GENERIC_HID)) {
          tud_hid_report(0, &pkt, sizeof(pkt));
          usb_in_pending = false;
        } else {
          printf("usb: generic HID not ready, drop\n");
          usb_in_pending = false;
        }
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
    else
    {
      // machine is not alive — turn off LEDs once on transition, then just wait
      static bool leds_cleared = false;
      if (!leds_cleared)
      {
        set_led_selected_axis(0);
        set_led_selected_increment(0);
        set_led_interp_state(INTERP_OFF);
        set_simple_led(6, 0, TaskLED::NORMAL);
        set_simple_led(7, 0, TaskLED::NORMAL, true);
        set_ring_led(0, 0, 0);
        set_ring_led(1, 0, 0);
        set_ring_led(2, 0, 0, true);
#ifdef ENABLE_DISPLAY
        {
          TaskDisplay::cmd_t dcmd;
          dcmd.cmd = TaskDisplay::DISPLAY_CMD_DISCONNECTED;
          xQueueSend(task_display->cmd_queue, &dcmd, 0);
        }
#endif
        leds_cleared = true;
      }

      printf("waiting for USB ...\n");

      usb_out_pkt out_pkt;
      if (xQueueReceive(usb_out_queue, &out_pkt, 500) == pdTRUE)
      {
        usb_dump_out_pkt(&out_pkt);

        g_machine_alive = true;
        leds_cleared = false;   // re-arm for next disconnect
        initial_feedrate = false;
        initial_rapidrate = false;
        initial_maxvel = false;
        enc_user_moved[1] = enc_user_moved[2] = enc_user_moved[3] = false;
        bzero(&last_out_pkt, sizeof(last_out_pkt));

        // Process the first OUT packet's override values immediately so encoder
        // positions are correct before any IN packet can be sent.  Without this
        // the sync is deferred until the second OUT packet (up to 500ms later)
        // and any knob touch in that window sends 0s to LinuxCNC.
        xTimerStart(hbeat_timer, 0);
        machine_estop       = out_pkt.s.estop;
        machine_enabled     = out_pkt.s.enabled;
        machine_mode        = (mode_t)out_pkt.s.mode;
        machine_interp_state = (interp_t)out_pkt.s.interp_state;
        machine_step_mode   = out_pkt.s.step_mode;
        machine_paused      = out_pkt.s.paused;
        g_machine_pos[0]    = out_pkt.s.pos_x * (1.0f / 10000.0f);
        g_machine_pos[1]    = out_pkt.s.pos_y * (1.0f / 10000.0f);
        g_machine_pos[2]    = out_pkt.s.pos_z * (1.0f / 10000.0f);
        // Synchronously set all 3 override encoders and their change-detection
        // shadow under the mutex.  This guarantees get_value() returns the
        // correct host-authoritative value before any user button press can
        // arrive, without relying on the encoder task draining an async queue
        // message within its next 10 ms loop tick.
        task_encoder->sync_all_immediate(
            out_pkt.s.feedrate_override,
            out_pkt.s.rapidrate_override,
            out_pkt.s.maxvel_override);
        set_ring_led(0, out_pkt.s.feedrate_override, dot_colors[0], false);
        set_ring_led(1, out_pkt.s.rapidrate_override, dot_colors[1], false);
        set_ring_led(2, out_pkt.s.maxvel_override, dot_colors[2], false);
        {
          TaskLED::cmd_t fcmd;
          fcmd.cmd = TaskLED::LED_CMD_FLUSH;
          xQueueSend(task_led->cmd_queue, &fcmd, 0);
        }
        // Reflect full machine state on all LEDs immediately at connect time,
        // without waiting for a state-change transition.
        if (!machine_estop && machine_enabled)
        {
          set_led_interp_state(machine_interp_state, machine_step_mode, machine_paused);
          if (machine_mode == MODE_MANUAL)
          {
            set_led_selected_increment(selected_increment);
            set_led_selected_axis(selected_axis);
          }
        }
        else
        {
          set_led_interp_state(INTERP_OFF);
          set_led_selected_increment(0);
          set_led_selected_axis(0);
        }
        initial_feedrate = true;
        initial_rapidrate = true;
        initial_maxvel = true;
        memcpy(&last_out_pkt, &out_pkt, sizeof(out_pkt));

        // Immediately reflect state on display at connect time.
#ifdef ENABLE_DISPLAY
        display_send_machine_state(
            machine_estop, machine_enabled, machine_homed, machine_coolant,
            machine_mode, machine_interp_state, machine_step_mode, machine_paused,
            out_pkt.s.feedrate_override, out_pkt.s.rapidrate_override, out_pkt.s.maxvel_override);
#endif
      }

    }
  }
}
