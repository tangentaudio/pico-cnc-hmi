#include <stdio.h>
#include "encoder.hh"
#include "quadrature.pio.h"


Encoder::Encoder() :
    m_pins{ 6, 12, 14, 16 },
    m_last_values{ -1, -1, -1, -1}
{
}

Encoder::~Encoder()
{
}

void Encoder::init()
{
    m_pio = pio0;
    pio_add_program(m_pio, &quadrature_encoder_program);
    for (uint i=0; i<NUM_ENCODERS; i++) {
        m_sm[i] = pio_claim_unused_sm(m_pio, true);
        quadrature_encoder_program_init(m_pio, m_sm[i], m_pins[i], 0);    
    }
} 

bool Encoder::task()
{
    bool update = false;

    for (uint i=0; i<NUM_ENCODERS; i++) {
        int new_value = quadrature_encoder_get_count(m_pio, m_sm[i]);
        if (new_value != m_last_values[i]) {
            printf("position #%u %8d\n", i, new_value);
            m_last_values[i] = new_value;
            update = true;
        }
    }

    return update;
}

int Encoder::value(uint num)
{
    return m_last_values[num % NUM_ENCODERS];
}