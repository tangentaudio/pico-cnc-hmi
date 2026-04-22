#include <map>
#include "tusb.h"
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
    TaskMatrix *inst = static_cast<TaskMatrix *>(param);

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

bool TaskMatrix::led_key_map(uint8_t key_code, uint8_t &led)
{

    for (int i = 0; i < sizeof(key_to_led_map); i++) {
        if (key_code == key_to_led_map[i]) {
            led = i;
            return true;
        }
    }
    return false;
}

bool TaskMatrix::led_encoder_map(uint8_t gpio_code, uint8_t &encoder)
{
    static constexpr uint8_t code_to_encoder_map[] = {0x68, 0x71, 0x72};

    for (int i = 0; i < sizeof(code_to_encoder_map); i++) {
        if (gpio_code == code_to_encoder_map[i]) {
            encoder = i;
            return true;
        }
    }
    return false;
}

bool TaskMatrix::hid_keycode(uint8_t matrix_key_code, uint8_t &hid_key_code, uint8_t &modifiers)
{
    const std::map<uint8_t, std::pair<uint8_t, uint8_t>> keycode_to_hid = {
        // Section A – Layout 2: G-code words grouped by usage frequency.
        // Row 1: G  M  T  F
        {0x2C, {HID_KEY_G, KEYBOARD_MODIFIER_LEFTSHIFT}},
        {0x05, {HID_KEY_M, KEYBOARD_MODIFIER_LEFTSHIFT}},
        {0x06, {HID_KEY_T, KEYBOARD_MODIFIER_LEFTSHIFT}},
        {0x07, {HID_KEY_F, KEYBOARD_MODIFIER_LEFTSHIFT}},
        // Row 2: X  Y  Z  S
        {0x08, {HID_KEY_X, KEYBOARD_MODIFIER_LEFTSHIFT}},
        {0x0F, {HID_KEY_Y, KEYBOARD_MODIFIER_LEFTSHIFT}},
        {0x10, {HID_KEY_Z, KEYBOARD_MODIFIER_LEFTSHIFT}},
        {0x11, {HID_KEY_S, KEYBOARD_MODIFIER_LEFTSHIFT}},
        // Row 3: I  J  K  R
        {0x12, {HID_KEY_I, KEYBOARD_MODIFIER_LEFTSHIFT}},
        {0x19, {HID_KEY_J, KEYBOARD_MODIFIER_LEFTSHIFT}},
        {0x1A, {HID_KEY_K, KEYBOARD_MODIFIER_LEFTSHIFT}},
        {0x1B, {HID_KEY_R, KEYBOARD_MODIFIER_LEFTSHIFT}},
        // Row 4: N  L  H  D
        {0x1C, {HID_KEY_N, KEYBOARD_MODIFIER_LEFTSHIFT}},
        {0x23, {HID_KEY_L, KEYBOARD_MODIFIER_LEFTSHIFT}},
        {0x24, {HID_KEY_H, KEYBOARD_MODIFIER_LEFTSHIFT}},
        {0x25, {HID_KEY_D, KEYBOARD_MODIFIER_LEFTSHIFT}},
        // Row 5: P  Q  [0x2E = wide SHIFT modifier – handled separately, not in this map]
        {0x26, {HID_KEY_P, KEYBOARD_MODIFIER_LEFTSHIFT}},
        {0x2D, {HID_KEY_Q, KEYBOARD_MODIFIER_LEFTSHIFT}},

        {0x01, {HID_KEY_KEYPAD_DIVIDE, 0}},
        {0x02, {HID_KEY_KEYPAD_MULTIPLY, 0}},
        {0x03, {HID_KEY_KEYPAD_SUBTRACT, 0}},
        {0x04, {HID_KEY_KEYPAD_ADD, 0}},
    
        {0x0B, {HID_KEY_7, 0}},
        {0x0C, {HID_KEY_8, 0}},
        {0x0D, {HID_KEY_9, 0}},
        {0x0E, {HID_KEY_TAB, KEYBOARD_MODIFIER_LEFTSHIFT}},
    
        {0x15, {HID_KEY_4, 0}},
        {0x16, {HID_KEY_5, 0}},
        {0x17, {HID_KEY_6, 0}},
        {0x18, {HID_KEY_TAB, 0}},
    
        {0x1F, {HID_KEY_1, 0}},
        {0x20, {HID_KEY_2, 0}},
        {0x21, {HID_KEY_3, 0}},
        {0x22, {HID_KEY_KEYPAD_ENTER, 0}},
    
        {0x29, {HID_KEY_BACKSPACE, 0}},
        {0x2A, {HID_KEY_0, 0}},
        {0x2B, {HID_KEY_PERIOD, 0}}
    };

    auto it = keycode_to_hid.find(matrix_key_code);
    if (it != keycode_to_hid.end()) {
        hid_key_code = it->second.first;
        modifiers = it->second.second;
        return true;
    }
    return false;
}

bool TaskMatrix::hid_n_key_buf_add(uint8_t* key_buf, uint8_t hid_key_code)
{
    for (int i = 0; i < 6; i++)
    {
        if (key_buf[i] == 0)
        {
            key_buf[i] = hid_key_code;
            return true;
        }
    }
    return false;
}
bool TaskMatrix::hid_n_key_buf_remove(uint8_t* key_buf, uint8_t hid_key_code)
{
    for (int i = 0; i < 6; i++)
    {
        if (key_buf[i] == hid_key_code)
        {
            key_buf[i] = 0;
            return true;
        }
    }
    return false;
}

bool TaskMatrix::is_section_a_shift(uint8_t code)
{
    return code == 0x2E;
}

bool TaskMatrix::hid_keycode_shifted(uint8_t matrix_key_code, uint8_t &hid_key_code, uint8_t &modifiers)
{
    // Shift layer for Section A – accessed while the wide modifier key (0x2E) is held.
    // Only keys that map to a different letter are listed; others fall back to the main layer.
    const std::map<uint8_t, std::pair<uint8_t, uint8_t>> shift_layer = {
        {0x2C, {HID_KEY_O, KEYBOARD_MODIFIER_LEFTSHIFT}},  // G -> O  (subroutine O-word)
        {0x05, {HID_KEY_E, KEYBOARD_MODIFIER_LEFTSHIFT}},  // M -> E
        {0x08, {HID_KEY_A, KEYBOARD_MODIFIER_LEFTSHIFT}},  // X -> A  (rotary axis)
        {0x0F, {HID_KEY_B, KEYBOARD_MODIFIER_LEFTSHIFT}},  // Y -> B  (rotary axis)
        {0x10, {HID_KEY_C, KEYBOARD_MODIFIER_LEFTSHIFT}},  // Z -> C  (rotary axis)
        {0x07, {HID_KEY_W, KEYBOARD_MODIFIER_LEFTSHIFT}},  // F -> W  (secondary linear axis)
        {0x12, {HID_KEY_U, KEYBOARD_MODIFIER_LEFTSHIFT}},  // I -> U  (secondary linear axis)
        {0x19, {HID_KEY_V, KEYBOARD_MODIFIER_LEFTSHIFT}},  // J -> V  (secondary linear axis)
        {0x0E, {HID_KEY_ESCAPE, 0}},                       // PREV -> Escape
        // Section B numpad – numlock-off navigation layer
        {0x0B, {HID_KEY_HOME, 0}},                         // 7 -> Home
        {0x0C, {HID_KEY_ARROW_UP, 0}},                     // 8 -> Up
        {0x0D, {HID_KEY_PAGE_UP, 0}},                      // 9 -> Page Up
        {0x15, {HID_KEY_ARROW_LEFT, 0}},                   // 4 -> Left
        {0x17, {HID_KEY_ARROW_RIGHT, 0}},                  // 6 -> Right
        {0x1F, {HID_KEY_END, 0}},                          // 1 -> End
        {0x20, {HID_KEY_ARROW_DOWN, 0}},                   // 2 -> Down
        {0x21, {HID_KEY_PAGE_DOWN, 0}},                    // 3 -> Page Down
        {0x2A, {HID_KEY_INSERT, 0}},                       // 0 -> Insert
        {0x2B, {HID_KEY_DELETE, 0}},                       // . -> Delete
    };

    auto it = shift_layer.find(matrix_key_code);
    if (it != shift_layer.end()) {
        hid_key_code = it->second.first;
        modifiers = it->second.second;
        return true;
    }
    return false;
}
