
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