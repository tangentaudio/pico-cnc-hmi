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
        {0x2C, {HID_KEY_O, KEYBOARD_MODIFIER_LEFTSHIFT}},
        {0x05, {HID_KEY_N, KEYBOARD_MODIFIER_LEFTSHIFT}},
        {0x06, {HID_KEY_G, KEYBOARD_MODIFIER_LEFTSHIFT}},
        {0x07, {HID_KEY_R, KEYBOARD_MODIFIER_LEFTSHIFT}},
    
        {0x08, {HID_KEY_X, KEYBOARD_MODIFIER_LEFTSHIFT}},
        {0x0F, {HID_KEY_Y, KEYBOARD_MODIFIER_LEFTSHIFT}},
        {0x10, {HID_KEY_Z, KEYBOARD_MODIFIER_LEFTSHIFT}},
        {0x11, {HID_KEY_W, KEYBOARD_MODIFIER_LEFTSHIFT}},
    
        {0x12, {HID_KEY_M, KEYBOARD_MODIFIER_LEFTSHIFT}},
        {0x19, {HID_KEY_S, KEYBOARD_MODIFIER_LEFTSHIFT}},
        {0x1A, {HID_KEY_T, KEYBOARD_MODIFIER_LEFTSHIFT}},
        {0x1B, {HID_KEY_KEYPAD_HASH, 0}},
    
        {0x1C, {HID_KEY_F, KEYBOARD_MODIFIER_LEFTSHIFT}},
        {0x23, {HID_KEY_H, KEYBOARD_MODIFIER_LEFTSHIFT}},
        {0x24, {HID_KEY_L, KEYBOARD_MODIFIER_LEFTSHIFT}},
        {0x25, {HID_KEY_D, KEYBOARD_MODIFIER_LEFTSHIFT}},
    

        {0x26, {HID_KEY_KEYPAD_LEFT_PARENTHESIS, 0}},
        {0x2D, {HID_KEY_KEYPAD_RIGHT_PARENTHESIS, 0}},
        {0x2E, {HID_KEY_SPACE, 0}},
    
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
        {0x2B, {HID_KEY_KEYPAD_DECIMAL, 0}}
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
