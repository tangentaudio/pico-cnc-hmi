/**
 * adapted from:
 * I2C Driver for the Adafruit TCA8418 Keypad Matrix / GPIO Expander
 *
 * 	This is a library for the Adafruit TCA8418 breakout:
 * 	https://www.adafruit.com/products/4918
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *	BSD license (see license.txt)
 */

/*
 * REV0 hardware map:
 *
 * ROW0..6 are the rows of the keypad matrix
 * COL0..7 are the columns of the keypad matrix
 * ROW7 is encoder 0 pushbutton GPIO
 * COL8 is encoder 1 pushbutton GPIO
 * COL9 is encoder 2 pushbutton GPIO
 */

#include <stdio.h>
#include "bsp/board_api.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "tca8418.hh"


TCA8418::TCA8418(uint8_t address) :
 m_address(address)
{
}

TCA8418::~TCA8418()
{
}

bool TCA8418::init(I2C *i2cbus)
{
    m_i2c = i2cbus;

    //  GPIO
    //  set default all GIO pins to INPUT
    writeRegister(TCA8418_REG_GPIO_DIR_1, 0x00);
    writeRegister(TCA8418_REG_GPIO_DIR_2, 0x00);
    writeRegister(TCA8418_REG_GPIO_DIR_3, 0x00);

    //  add all pins to key events
    writeRegister(TCA8418_REG_GPI_EM_1, 0xFF);
    writeRegister(TCA8418_REG_GPI_EM_2, 0xFF);
    writeRegister(TCA8418_REG_GPI_EM_3, 0xFF);

    //  set all pins to FALLING interrupts
    writeRegister(TCA8418_REG_GPIO_INT_LVL_1, 0x00);
    writeRegister(TCA8418_REG_GPIO_INT_LVL_2, 0x00);
    writeRegister(TCA8418_REG_GPIO_INT_LVL_3, 0x00);

    //  add all pins to interrupts
    writeRegister(TCA8418_REG_GPIO_INT_EN_1, 0xFF);
    writeRegister(TCA8418_REG_GPIO_INT_EN_2, 0xFF);
    writeRegister(TCA8418_REG_GPIO_INT_EN_3, 0xFF);

    // set up interrupt pin
    gpio_init(PIN_KEY_INT);
    gpio_set_dir(PIN_KEY_INT, GPIO_IN);
    gpio_pull_up(PIN_KEY_INT);

    // configure matrix and gpio
    matrix(7, 8);
    enableDebounce();
    enableInterrupts();

    return true;
}

/**
 * @brief configures the size of the keypad matrix.
 *
 * @param [in] rows    number of rows, should be <= 8
 * @param [in] columns number of columns, should be <= 10
 * @return true is rows and columns have valid values.
 *
 * @details will always use the lowest pins for rows and columns.
 *          0..rows-1  and  0..columns-1
 */
bool TCA8418::matrix(uint8_t rows, uint8_t columns)
{
    if ((rows > 8) || (columns > 10))
        return false;

    //  MATRIX
    //  skip zero size matrix
    if ((rows != 0) && (columns != 0))
    {
        // setup the keypad matrix.
        uint8_t mask = 0x00;
        for (int r = 0; r < rows; r++)
        {
            mask <<= 1;
            mask |= 1;
        }
        writeRegister(TCA8418_REG_KP_GPIO_1, mask);

        mask = 0x00;
        for (int c = 0; c < columns && c < 8; c++)
        {
            mask <<= 1;
            mask |= 1;
        }
        writeRegister(TCA8418_REG_KP_GPIO_2, mask);

        if (columns > 8)
        {
            if (columns == 9)
                mask = 0x01;
            else
                mask = 0x03;
            writeRegister(TCA8418_REG_KP_GPIO_3, mask);
        }
    }

    return true;
}

/////////////////////////////////////////////////////////////////////////////
//
//  KEY EVENTS
//

/**
 * @brief checks if key events are available in the internal buffer
 *
 * @return number of key events in the buffer
 *         0x80 is ORed in if there's an overflow condition
 */
uint8_t TCA8418::available()
{
    if (gpio_get(PIN_KEY_INT) == 0)
    {
        // clear interrupt
        uint8_t r = readRegister(TCA8418_REG_INT_STAT);
        writeRegister(TCA8418_REG_INT_STAT, 0x1F);
        if (r & 0x02)
        {
            uint8_t g1 = readRegister(TCA8418_REG_GPIO_INT_STAT_1);
            uint8_t g2 = readRegister(TCA8418_REG_GPIO_INT_STAT_2);
            uint8_t g3 = readRegister(TCA8418_REG_GPIO_INT_STAT_3);
        }
        bool ovf = 0;
        if (r & 0x08)
        {
            ovf = 0x80;
        }

        uint8_t eventCount = readRegister(TCA8418_REG_KEY_LCK_EC);
        eventCount &= 0x0F; //  lower 4 bits only
        return eventCount | ovf;
    }
    return 0;
}

/**
 * @brief gets first event from the internal buffer
 *
 * @return key event or 0 if none available
 *
 * @details
 *     key event 0x00        no event
 *               0x01..0x50  key  press
 *               0x81..0xD0  key  release
 *               0x5B..0x72  GPIO press
 *               0xDB..0xF2  GPIO release
 */
uint8_t TCA8418::getEvent()
{
    uint8_t event = readRegister(TCA8418_REG_KEY_EVENT_A);
    return event;
}

/**
 * @brief flushes the internal buffer of key events
 *        and cleans the GPIO status registers.
 *
 * @return number of keys flushed.
 */
uint8_t TCA8418::flush()
{
    //  flush key events
    uint8_t count = 0;
    while (getEvent() != 0)
        count++;
    //  flush gpio events
    readRegister(TCA8418_REG_GPIO_INT_STAT_1);
    readRegister(TCA8418_REG_GPIO_INT_STAT_2);
    readRegister(TCA8418_REG_GPIO_INT_STAT_3);
    //  clear INT_STAT register
    writeRegister(TCA8418_REG_INT_STAT, 3);
    return count;
}

/////////////////////////////////////////////////////////////////////////////
//
//  GPIO
//

/**
 * @brief read GPIO
 *
 * @param [in] pinnum Pin name between TCA8418_ROW0 and TCA8418_COL9  0..17
 * @return 0 = LOW, 1 = HIGH, 0xFF = pinnum out of range
 */
uint8_t TCA8418::digitalRead(uint8_t pinnum)
{
    if (pinnum > TCA8418_COL9)
        return 0xFF;

    uint8_t reg = TCA8418_REG_GPIO_DAT_STAT_1 + pinnum / 8;
    uint8_t mask = (1 << (pinnum % 8));

    // LEVEL  0 = LOW  other = HIGH
    uint8_t value = readRegister(reg);
    if (value & mask)
        return HIGH;
    return LOW;
}

/**
 * @brief set GPIO pin to LOW or HIGH
 *
 * @param [in] pinnum  Pin name between TCA8418_ROW0 and TCA8418_COL9  0..17
 * @param [in] level   0 = LOW, all other are HIGH
 * @return true if successful
 */
bool TCA8418::digitalWrite(uint8_t pinnum, uint8_t level)
{
    if (pinnum > TCA8418_COL9)
        return false;

    uint8_t reg = TCA8418_REG_GPIO_DAT_OUT_1 + pinnum / 8;
    uint8_t mask = (1 << (pinnum % 8));

    // LEVEL  0 = LOW  other = HIGH
    uint8_t value = readRegister(reg);
    if (level == LOW)
        value &= ~mask;
    else
        value |= mask;
    writeRegister(reg, value);
    return true;
}

/**
 * @brief set mode of GPIO pin to INPUT, INPUT_PULLUP or OUTPUT
 *
 * @param [in] pinnum Pin name between TCA8418_ROW0 and TCA8418_COL9  0..17
 * @param [in] mode   INPUT, INPUT_PULLUP or OUTPUT
 * @return  false if failed.
 */
bool TCA8418::pinMode(uint8_t pinnum, uint8_t mode)
{
    if (pinnum > TCA8418_COL9)
        return false;
    // if (mode > INPUT_PULLUP) return false; ?s

    uint8_t idx = pinnum / 8;
    uint8_t reg = TCA8418_REG_GPIO_DIR_1 + idx;
    uint8_t mask = (1 << (pinnum % 8));

    // MODE  0 = INPUT   1 = OUTPUT
    uint8_t value = readRegister(reg);
    if (mode == OUTPUT)
        value |= mask;
    else
        value &= ~mask;
    writeRegister(reg, value);

    // PULLUP  0 = enabled   1 = disabled
    reg = TCA8418_REG_GPIO_PULL_1 + idx;
    value = readRegister(reg);
    if (mode == INPUT_PULLUP)
        value &= ~mask;
    else
        value |= mask;
    writeRegister(reg, value);

    return true;
}

/**
 * @brief set IRQ mode of GPIO pin to FALLING RISING
 *
 * @param [in] pinnum  Pin name between TCA8418_ROW0 and TCA8418_COL9  0..17
 * @param [in] mode    IRQ mode   FALLING RISING
 * @return  false if failed.
 */
bool TCA8418::pinIRQMode(uint8_t pinnum, uint8_t mode)
{
    if (pinnum > TCA8418_COL9)
        return false;
    if ((mode != RISING) && (mode != FALLING))
        return false;

    //  MODE  0 = FALLING   1 = RISING
    uint8_t idx = pinnum / 8;
    uint8_t reg = TCA8418_REG_GPIO_INT_LVL_1 + idx;
    uint8_t mask = (1 << (pinnum % 8));

    uint8_t value = readRegister(reg);
    if (mode == RISING)
        value |= mask;
    else
        value &= ~mask;
    writeRegister(reg, value);

    // ENABLE INTERRUPT
    reg = TCA8418_REG_GPIO_INT_EN_1 + idx;
    value = readRegister(reg);
    value |= mask;
    writeRegister(reg, value);

    return true;
}

/////////////////////////////////////////////////////////////////////////////
//
//  CONFIGURATION
//

/**
 * @brief enables key event + GPIO interrupts.
 */
void TCA8418::enableInterrupts()
{
    uint8_t value = readRegister(TCA8418_REG_CFG);
    value |= (TCA8418_REG_CFG_GPI_IEN | TCA8418_REG_CFG_KE_IEN);
    writeRegister(TCA8418_REG_CFG, value);
};

/**
 * @brief disables key events + GPIO interrupts.
 */
void TCA8418::disableInterrupts()
{
    uint8_t value = readRegister(TCA8418_REG_CFG);
    value &= ~(TCA8418_REG_CFG_GPI_IEN | TCA8418_REG_CFG_KE_IEN);
    writeRegister(TCA8418_REG_CFG, value);
};

/**
 * @brief enables matrix overflow interrupt.
 */
void TCA8418::enableMatrixOverflow()
{
    uint8_t value = readRegister(TCA8418_REG_CFG);
    value |= TCA8418_REG_CFG_OVR_FLOW_M;
    writeRegister(TCA8418_REG_CFG, value);
};

/**
 * @brief disables matrix overflow interrupt.
 */
void TCA8418::disableMatrixOverflow()
{
    uint8_t value = readRegister(TCA8418_REG_CFG);
    value &= ~TCA8418_REG_CFG_OVR_FLOW_M;
    writeRegister(TCA8418_REG_CFG, value);
};

/**
 * @brief enables key debounce.
 */
void TCA8418::enableDebounce()
{
    writeRegister(TCA8418_REG_DEBOUNCE_DIS_1, 0x00);
    writeRegister(TCA8418_REG_DEBOUNCE_DIS_2, 0x00);
    writeRegister(TCA8418_REG_DEBOUNCE_DIS_3, 0x00);
}

/**
 * @brief disables key debounce.
 */
void TCA8418::disableDebounce()
{
    writeRegister(TCA8418_REG_DEBOUNCE_DIS_1, 0xFF);
    writeRegister(TCA8418_REG_DEBOUNCE_DIS_2, 0xFF);
    writeRegister(TCA8418_REG_DEBOUNCE_DIS_3, 0xFF);
}

/////////////////////////////////////////////////////////////////////////////
//
//  LOW LEVEL
//

/**
 * @brief reads byte value from register
 *
 * @param [in] reg register address
 * @return value from register
 */
uint8_t TCA8418::readRegister(uint8_t reg)
{
    uint8_t val;

    m_i2c->readRegister(m_address, reg, val);
    return val;
}

/**
 * @brief write byte value to register
 *
 * @param [in] reg register address
 * @param [in] value
 */
void TCA8418::writeRegister(uint8_t reg, uint8_t value)
{
    m_i2c->writeRegister(m_address, reg, value);
}