#ifndef __TASK_MATRIX_HH
#define __TASK_MATRIX_HH

#include <FreeRTOS.h>
#include <queue.h>
#include <task.h>
#include "i2c.hh"
#include "tca8418.hh"

class TaskMatrix
{
public:
    TaskMatrix();
    ~TaskMatrix();

    typedef struct event
    {
        uint8_t code;
        bool press;
        bool gpio;
        bool err;
    } event_t;

    void init(I2C *i2cbus);
    static void task(void *param);

    QueueHandle_t event_queue;

    static bool led_key_map(uint8_t key_code, uint8_t &led);
    static uint8_t led_key_count() { return sizeof(key_to_led_map); }
    static bool led_encoder_map(uint8_t gpio_code, uint8_t &encoder);

    static bool hid_keycode(uint8_t matrix_key_code, uint8_t &hid_key_code, uint8_t &modifiers);
    
    static bool hid_n_key_buf_add(uint8_t* key_buf, uint8_t hid_key_code);
    static bool hid_n_key_buf_remove(uint8_t* key_buf, uint8_t hid_key_code);

    static bool hid_n_key_buf_is_empty(uint8_t* key_buf) {
        for (int i = 0; i < 6; i++)
        {
            if (key_buf[i] != 0)
                return false;
        }
        return true;
    }

protected:
    I2C *m_i2c;
    TCA8418 m_matrix;
    static constexpr uint8_t key_to_led_map[] = {0x2f, 0x30, 0x37, 0x38, 0x39, 0x3a, 0x3d, 0x3e, 0x33, 0x34, 0x35, 0x36};
};


#endif
