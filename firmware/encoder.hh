#ifndef __encoder_hh
#define __encoder_hh

#include <hardware/pio.h>

#define NUM_ENCODERS 4

#define PIN_SHUTTLE0 8
#define PIN_SHUTTLE1 9
#define PIN_SHUTTLE2 10
#define PIN_SHUTTLE3 11

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
    uint8_t m_last_shuttle;
};


#endif