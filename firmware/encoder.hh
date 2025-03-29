#ifndef __encoder_hh
#define __encoder_hh

#include <hardware/pio.h>

#define NUM_ENCODERS 4

#define PIN_SHUTTLE0 8
#define PIN_SHUTTLE1 9
#define PIN_SHUTTLE2 10
#define PIN_SHUTTLE3 11


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
const int shuttle_code_lookup[16] = { -8, 5, -6, 6, -5, 4, -7, 7, -3, 2, -2, 1, -4, 3, -1, 0};


class Encoder {
public:
    Encoder();
    ~Encoder();
    void init();
    bool task();
    int value(uint8_t num);
    void set_limits(uint8_t num, int min, int max, int div);
    void set_value(uint8_t num, int val);

    int get_min(uint8_t num) { return m_minimum[num] / m_divisor[num]; }
    int get_max(uint8_t num) { return m_maximum[num] / m_divisor[num]; }
    int get_div(uint8_t num) { return m_divisor[num]; }

private:
    PIO m_pio;
    const uint m_pins[NUM_ENCODERS];
    uint m_sm[NUM_ENCODERS];
    int m_last_values[NUM_ENCODERS];
    int m_cur_values[NUM_ENCODERS];
    int m_minimum[NUM_ENCODERS];
    int m_maximum[NUM_ENCODERS];
    int m_divisor[NUM_ENCODERS];
    uint8_t m_last_shuttle;
    int m_last_shuttle_val;
};


#endif