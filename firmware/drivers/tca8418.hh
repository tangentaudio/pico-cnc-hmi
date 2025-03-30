#ifndef __TCA8418_HH
#define __TCA8418_HH

#include "i2c.hh"

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

//  REGISTERS

// #define TCA8418_REG_RESERVED          0x00
#define TCA8418_REG_CFG 0x01             ///< Configuration register
#define TCA8418_REG_INT_STAT 0x02        ///< Interrupt status
#define TCA8418_REG_KEY_LCK_EC 0x03      ///< Key lock and event counter
#define TCA8418_REG_KEY_EVENT_A 0x04     ///< Key event register A
#define TCA8418_REG_KEY_EVENT_B 0x05     ///< Key event register B
#define TCA8418_REG_KEY_EVENT_C 0x06     ///< Key event register C
#define TCA8418_REG_KEY_EVENT_D 0x07     ///< Key event register D
#define TCA8418_REG_KEY_EVENT_E 0x08     ///< Key event register E
#define TCA8418_REG_KEY_EVENT_F 0x09     ///< Key event register F
#define TCA8418_REG_KEY_EVENT_G 0x0A     ///< Key event register G
#define TCA8418_REG_KEY_EVENT_H 0x0B     ///< Key event register H
#define TCA8418_REG_KEY_EVENT_I 0x0C     ///< Key event register I
#define TCA8418_REG_KEY_EVENT_J 0x0D     ///< Key event register J
#define TCA8418_REG_KP_LCK_TIMER 0x0E    ///< Keypad lock1 to lock2 timer
#define TCA8418_REG_UNLOCK_1 0x0F        ///< Unlock register 1
#define TCA8418_REG_UNLOCK_2 0x10        ///< Unlock register 2
#define TCA8418_REG_GPIO_INT_STAT_1 0x11 ///< GPIO interrupt status 1
#define TCA8418_REG_GPIO_INT_STAT_2 0x12 ///< GPIO interrupt status 2
#define TCA8418_REG_GPIO_INT_STAT_3 0x13 ///< GPIO interrupt status 3
#define TCA8418_REG_GPIO_DAT_STAT_1 0x14 ///< GPIO data status 1
#define TCA8418_REG_GPIO_DAT_STAT_2 0x15 ///< GPIO data status 2
#define TCA8418_REG_GPIO_DAT_STAT_3 0x16 ///< GPIO data status 3
#define TCA8418_REG_GPIO_DAT_OUT_1 0x17  ///< GPIO data out 1
#define TCA8418_REG_GPIO_DAT_OUT_2 0x18  ///< GPIO data out 2
#define TCA8418_REG_GPIO_DAT_OUT_3 0x19  ///< GPIO data out 3
#define TCA8418_REG_GPIO_INT_EN_1 0x1A   ///< GPIO interrupt enable 1
#define TCA8418_REG_GPIO_INT_EN_2 0x1B   ///< GPIO interrupt enable 2
#define TCA8418_REG_GPIO_INT_EN_3 0x1C   ///< GPIO interrupt enable 3
#define TCA8418_REG_KP_GPIO_1 0x1D       ///< Keypad/GPIO select 1
#define TCA8418_REG_KP_GPIO_2 0x1E       ///< Keypad/GPIO select 2
#define TCA8418_REG_KP_GPIO_3 0x1F       ///< Keypad/GPIO select 3
#define TCA8418_REG_GPI_EM_1 0x20        ///< GPI event mode 1
#define TCA8418_REG_GPI_EM_2 0x21        ///< GPI event mode 2
#define TCA8418_REG_GPI_EM_3 0x22        ///< GPI event mode 3
#define TCA8418_REG_GPIO_DIR_1 0x23      ///< GPIO data direction 1
#define TCA8418_REG_GPIO_DIR_2 0x24      ///< GPIO data direction 2
#define TCA8418_REG_GPIO_DIR_3 0x25      ///< GPIO data direction 3
#define TCA8418_REG_GPIO_INT_LVL_1 0x26  ///< GPIO edge/level detect 1
#define TCA8418_REG_GPIO_INT_LVL_2 0x27  ///< GPIO edge/level detect 2
#define TCA8418_REG_GPIO_INT_LVL_3 0x28  ///< GPIO edge/level detect 3
#define TCA8418_REG_DEBOUNCE_DIS_1 0x29  ///< Debounce disable 1
#define TCA8418_REG_DEBOUNCE_DIS_2 0x2A  ///< Debounce disable 2
#define TCA8418_REG_DEBOUNCE_DIS_3 0x2B  ///< Debounce disable 3
#define TCA8418_REG_GPIO_PULL_1 0x2C     ///< GPIO pull-up disable 1
#define TCA8418_REG_GPIO_PULL_2 0x2D     ///< GPIO pull-up disable 2
#define TCA8418_REG_GPIO_PULL_3 0x2E     ///< GPIO pull-up disable 3
// #define TCA8418_REG_RESERVED          0x2F

//  FIELDS CONFIG REGISTER  1
#define TCA8418_REG_CFG_AI 0x80           ///< Auto-increment for read/write
#define TCA8418_REG_CFG_GPI_E_CGF 0x40    ///< Event mode config
#define TCA8418_REG_CFG_OVR_FLOW_M 0x20   ///< Overflow mode enable
#define TCA8418_REG_CFG_INT_CFG 0x10      ///< Interrupt config
#define TCA8418_REG_CFG_OVR_FLOW_IEN 0x08 ///< Overflow interrupt enable
#define TCA8418_REG_CFG_K_LCK_IEN 0x04    ///< Keypad lock interrupt enable
#define TCA8418_REG_CFG_GPI_IEN 0x02      ///< GPI interrupt enable
#define TCA8418_REG_CFG_KE_IEN 0x01       ///< Key events interrupt enable

//  FIELDS INT_STAT REGISTER  2
#define TCA8418_REG_STAT_CAD_INT 0x10      ///< Ctrl-alt-del seq status
#define TCA8418_REG_STAT_OVR_FLOW_INT 0x08 ///< Overflow interrupt status
#define TCA8418_REG_STAT_K_LCK_INT 0x04    ///< Key lock interrupt status
#define TCA8418_REG_STAT_GPI_INT 0x02      ///< GPI interrupt status
#define TCA8418_REG_STAT_K_INT 0x01        ///< Key events interrupt status

//  FIELDS  KEY_LCK_EC REGISTER 3
#define TCA8418_REG_LCK_EC_K_LCK_EN 0x40 ///< Key lock enable
#define TCA8418_REG_LCK_EC_LCK_2 0x20    ///< Keypad lock status 2
#define TCA8418_REG_LCK_EC_LCK_1 0x10    ///< Keypad lock status 1
#define TCA8418_REG_LCK_EC_KLEC_3 0x08   ///< Key event count bit 3
#define TCA8418_REG_LCK_EC_KLEC_2 0x04   ///< Key event count bit 2
#define TCA8418_REG_LCK_EC_KLEC_1 0x02   ///< Key event count bit 1
#define TCA8418_REG_LCK_EC_KLEC_0 0x01   ///< Key event count bit 0

#define TCA8418_DEFAULT_ADDR 0x34 ///< The default I2C address for our breakout

/** Pin IDs for matrix rows/columns */
enum
{
    TCA8418_ROW0, // Pin ID for row 0
    TCA8418_ROW1, // Pin ID for row 1
    TCA8418_ROW2, // Pin ID for row 2
    TCA8418_ROW3, // Pin ID for row 3
    TCA8418_ROW4, // Pin ID for row 4
    TCA8418_ROW5, // Pin ID for row 5
    TCA8418_ROW6, // Pin ID for row 6
    TCA8418_ROW7, // Pin ID for row 7
    TCA8418_COL0, // Pin ID for column 0
    TCA8418_COL1, // Pin ID for column 1
    TCA8418_COL2, // Pin ID for column 2
    TCA8418_COL3, // Pin ID for column 3
    TCA8418_COL4, // Pin ID for column 4
    TCA8418_COL5, // Pin ID for column 5
    TCA8418_COL6, // Pin ID for column 6
    TCA8418_COL7, // Pin ID for column 7
    TCA8418_COL8, // Pin ID for column 8
    TCA8418_COL9  // Pin ID for column 9
};

#define PIN_KEY_INT 19

// these are needed because it's an Arduino library port
#define LOW 0
#define HIGH 1
#define INPUT 0
#define INPUT_PULLUP 1
#define OUTPUT 2
#define FALLING 0
#define RISING 1

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the TCA8418 I2C GPIO expander
 */
class TCA8418
{
public:
    TCA8418(uint8_t address = TCA8418_DEFAULT_ADDR);
    ~TCA8418();

    //  initialize the TCA8418
    bool init(I2C* i2cbus);

    //  KEY EVENTS
    //  configure the size of the keypad.
    //  all other rows and columns are set as inputs.
    bool matrix(uint8_t rows, uint8_t columns);

    //  key events available in the internal FIFO buffer
    uint8_t available();

    //  get one event from the FIFO buffer
    //  bit 7 indicates  press = 0  or  release == 1 (mask 0x80)
    uint8_t getEvent();

    //  flush all events in the FIFO buffer + GPIO events
    uint8_t flush();

    //  GPIO
    uint8_t digitalRead(uint8_t pinnum);
    bool digitalWrite(uint8_t pinnum, uint8_t level);
    bool pinMode(uint8_t pinnum, uint8_t mode);
    bool pinIRQMode(uint8_t pinnum, uint8_t mode); // MODE  FALLING or RISING

    //  CONFIGURATION
    //  enable / disable interrupts for matrix and GPI pins
    void enableInterrupts();
    void disableInterrupts();

    //  ignore key events when FIFO buffer is full or not.
    void enableMatrixOverflow();
    void disableMatrixOverflow();

    //  debounce keys.
    void enableDebounce();
    void disableDebounce();

    // for expert mode
    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t value);

protected:
    I2C* m_i2c;
    uint8_t m_address;
};

#endif