#ifndef __encoder_hh
#define __encoder_hh

#include <hardware/pio.h>

#define NUM_ENCODERS 4

class Encoder {
public:
    Encoder();
    ~Encoder();
    void init();
    bool task();
    int value(uint num);
private:
    PIO m_pio;
    const uint m_pins[NUM_ENCODERS];
    uint m_sm[NUM_ENCODERS];
    int m_last_values[NUM_ENCODERS];
};


#endif