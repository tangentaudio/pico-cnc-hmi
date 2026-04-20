#include <stdio.h>
#include "pico/time.h"
#include <FreeRTOS.h>
#include <task.h>
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
    // Enforce minimum inter-frame gap using absolute time tracking.
    // At the start of each update(), wait until the previous frame's transmission
    // and WS2812 reset period are guaranteed complete, then record the deadline for
    // the *next* call before clocking any pixels.
    //
    // 45 pixels × 30µs/pixel (800kHz, 24-bit) = 1350µs frame time
    // + 100µs reset margin (WS2812 needs >50µs; +30µs for OSR drain after FIFO empty)
    // = 1450µs minimum between frame starts.
    //
    // This avoids the race in the FIFO-drain + sleep approach where the OSR still
    // has one pixel in-flight after the FIFO empties, causing partial-frame corruption.
    static absolute_time_t next_update_at = nil_time;
    busy_wait_until(next_update_at);

    // Disable FreeRTOS task preemption for the duration of the pixel write.
    // Higher-priority encoder/matrix tasks preempting mid-loop stall the PIO,
    // causing the strip to latch a partial frame — wrong colors on wrong rings.
    // 45 pixels at 800kHz takes ~1.35ms; this is the critical window.
    taskENTER_CRITICAL();
    next_update_at = make_timeout_time_us(1450);
    for (uint8_t i = 0; i < WS2812_NUM_LEDS; i++) {
        put_pixel(m_pio, m_sm, m_leds[i]);
    }
    taskEXIT_CRITICAL();
}

void WS2812::setRing(uint8_t ring, uint8_t value, uint32_t ball_color, bool update_now)
{
    printf("setRing %d %d %x %d\n", ring, value, ball_color, update_now);
    if (ring >= 3) return;

    uint8_t ofs = ring * 15;

    for (uint8_t i = 0; i < 15; i++) {
        if (i == value) {
            m_leds[(i + ofs)] = ball_color;
        } else {
            m_leds[(i + ofs)] = 0;
        }
    }
    if (update_now)
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

