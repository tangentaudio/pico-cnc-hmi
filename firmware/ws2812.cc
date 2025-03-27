#include "ws2812.hh"

WS2812::WS2812()
{
    for (int i = 0; i < WS2812_NUM_LEDS; i++) {
        m_leds[i] = 0;
    }
}

WS2812::~WS2812() 
{
}

void WS2812::init() 
{
    bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&ws2812_program, &m_pio, &m_sm, &m_offset, PIN_WS2812, 1, true);
    hard_assert(success);

    ws2812_program_init(m_pio, m_sm, m_offset, PIN_WS2812, WS2812_FREQ, false);
}

void WS2812::setLED(uint8_t num, uint32_t value, bool update_now) 
{
    m_leds[num % WS2812_NUM_LEDS] = value;
    if (update_now)
        update();
}

void WS2812::update() 
{
    for (uint8_t i = 0; i < WS2812_NUM_LEDS; i++) {
        put_pixel(m_pio, m_sm, m_leds[i]);
    }
}

void WS2812::setRing(uint8_t ring, uint8_t value)
{
    if (ring >= 3) return;

    uint8_t ofs = ring * 15;

    for (uint8_t i = 0; i < 15; i++) {
        if (i < value) {
            m_leds[(i + ofs)] = urgb_u32(0x0f, 0x0f, 0x0f);
        } else if (i == value) {
            m_leds[(i + ofs)] = urgb_u32(0, 0x7f, 0);
        } else {
            m_leds[(i + ofs)] = 0;
        }
    }
    update();
}

void WS2812::put_pixel(PIO pio, uint sm, uint32_t pixel_grb) {
    pio_sm_put_blocking(pio, sm, pixel_grb << 8u);
}

uint32_t WS2812::urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t) (r) << 8) |
           ((uint32_t) (g) << 16) |
            (uint32_t) (b);
}

