#include <stdio.h>
#include "encoder.hh"
#include "quadrature.pio.h"

Encoder::Encoder() : m_pins{6, 12, 14, 16},
                     m_last_values{-1, -1, -1, -1},
                     m_last_shuttle(0xFF),
                     m_last_shuttle_val(0)
{
}

Encoder::~Encoder()
{
}

void Encoder::init()
{
    m_pio = pio0;
    pio_add_program(m_pio, &quadrature_encoder_program);
    for (uint i = 0; i < NUM_ENCODERS; i++)
    {
        m_sm[i] = pio_claim_unused_sm(m_pio, true);
        quadrature_encoder_program_init(m_pio, m_sm[i], m_pins[i], 0);
    }

    gpio_init(PIN_SHUTTLE0);
    gpio_set_dir(PIN_SHUTTLE0, GPIO_IN);
    gpio_pull_up(PIN_SHUTTLE0);
    gpio_init(PIN_SHUTTLE1);
    gpio_set_dir(PIN_SHUTTLE1, GPIO_IN);
    gpio_pull_up(PIN_SHUTTLE1);
    gpio_init(PIN_SHUTTLE2);
    gpio_set_dir(PIN_SHUTTLE2, GPIO_IN);
    gpio_pull_up(PIN_SHUTTLE2);
    gpio_init(PIN_SHUTTLE3);
    gpio_set_dir(PIN_SHUTTLE3, GPIO_IN);
    gpio_pull_up(PIN_SHUTTLE3);
}

bool Encoder::task()
{
    bool update = false;

    for (uint i = 0; i < NUM_ENCODERS; i++)
    {
        int new_value = quadrature_encoder_get_count(m_pio, m_sm[i]);
        if (new_value != m_last_values[i])
        {
            m_last_values[i] = new_value;
            update = true;
        }
    }

    uint32_t shuttle_code = (gpio_get_all() >> PIN_SHUTTLE0) & 0x0F;

    if (shuttle_code != m_last_shuttle) {
        int val = shuttle_code_lookup[shuttle_code];
        if (val != -8) {
            update = true;
        }
        m_last_shuttle = shuttle_code;
        m_last_shuttle_val = val;
    }



    return update;
}

int Encoder::value(uint num)
{
    if (num >= NUM_ENCODERS)
        return m_last_shuttle_val;
    else if (num > 0)
        return -m_last_values[num % NUM_ENCODERS];
    return m_last_values[num % NUM_ENCODERS];
}