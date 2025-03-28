#include <stdio.h>
#include "encoder.hh"
#include "quadrature.pio.h"

Encoder::Encoder() : m_pins{6, 12, 14, 16},
                     m_last_values{-1, -1, -1, -1},
                     m_cur_values{0, 0, 0, 0},
                     m_minimum{0, 0, 0, 0},
                     m_maximum{100, 100, 100, 100},
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
            int diff = new_value - m_last_values[i];

            if (i != 0) { 
                // invert the value for the encoders but not the shuttle
                diff = -diff;
            }

            if (diff > 0)
            {
                if (m_cur_values[i] + diff < m_maximum[i])
                    m_cur_values[i] += diff;
                else
                    m_cur_values[i] = m_maximum[i];
            }
            else if (diff < 0)
            {
                if (m_cur_values[i] + diff > m_minimum[i])
                    m_cur_values[i] += diff;
                else
                    m_cur_values[i] = m_minimum[i];
            }

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

void Encoder::set_limits(uint8_t num, int min, int max, int div)
{
    if (num >= NUM_ENCODERS)
        return;
    m_minimum[num] = min * div;
    m_maximum[num] = max * div;
    m_divisor[num] = div;
}

void Encoder::set_value(uint8_t num, int val)
{
    if (num >= NUM_ENCODERS)
        return;
    m_cur_values[num] = val;
}

int Encoder::value(uint8_t num)
{
    if (num >= NUM_ENCODERS)
        return m_last_shuttle_val;
    
    return m_cur_values[num] / m_divisor[num];
}