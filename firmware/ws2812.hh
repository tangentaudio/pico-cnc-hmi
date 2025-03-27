#ifndef __WS2812_H
#define __WS2812_H

#include "ws2812.pio.h"

#define PIN_WS2812 18
#define WS2812_FREQ 800000
#define WS2812_NUM_LEDS 45

class WS2812
{
public:
    WS2812();
    ~WS2812();

    void init();
    void update();
    void setRing(uint8_t ring, uint8_t value);
    void setLED(uint8_t num, uint32_t value, bool update_now = true);
    uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b);

protected:
    void put_pixel(PIO pio, uint sm, uint32_t pixel_grb);

private:
    PIO m_pio;
    uint m_sm;
    uint m_offset;
    uint32_t m_leds[WS2812_NUM_LEDS];
};

#endif

