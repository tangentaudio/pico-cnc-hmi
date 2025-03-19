#include <stdio.h>
#include "encoder.hh"
#include "quadrature.pio.h"

Encoder::Encoder() : m_pins{6, 12, 14, 16},
                     m_last_values{-1, -1, -1, -1},
                     m_last_shuttle(0xFF)
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
            printf("position #%u %8d\n", i, new_value);
            m_last_values[i] = new_value;
            update = true;
        }
    }

    uint32_t shuttle_code = (gpio_get_all() >> PIN_SHUTTLE0) & 0x0F;

    //     7    0x07
    //     6    0x03
    //     5    0x01
    //     4    0x05
    //     3    0x0D
    //     2    0x09
    //  CW 1    0x0B
    // neutral  0x0F
    // CCW-1    0x0E
    //    -2    0x0A
    //    -3    0x08
    //    -4    0x0C
    //    -5    0x04
    //    -6    0x02
    //    -7    0x06     
    //
    //  invalid 0x00
    const int code_lookup[16] = { -8, 5, -6, 6, -5, 4, -7, 7, -3, 2, -2, 1, -4, 3, -1, 0};

    if (shuttle_code != m_last_shuttle) {
        int val = code_lookup[shuttle_code];
        if (val != -8) {
            printf("shuttle=%d\n", val);
            update = true;
        }
        m_last_shuttle = shuttle_code;
    }



    return update;
}

int Encoder::value(uint num)
{
    return m_last_values[num % NUM_ENCODERS];
}