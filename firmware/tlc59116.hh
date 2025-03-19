#ifndef __tlc59116_hh
#define __tlc59116_hh

#include "i2c.hh"

// I2C registers and other definitions for TI LED driver TLC59116
// see: http://www.ti.com/product/tlc59116


enum tlc59116_reg {
	REG_MODE1 = 0x00,
	REG_MODE2 = 0x01,
	REG_PWM0 = 0x02,
	REG_PWM1 = 0x03,
	REG_PWM2 = 0x04,
	REG_PWM3 = 0x05,
	REG_PWM4 = 0x06,
	REG_PWM5 = 0x07,
	REG_PWM6 = 0x08,
	REG_PWM7 = 0x09,
	REG_PWM8 = 0x0A,
	REG_PWM9 = 0x0B,
	REG_PWM10 = 0x0C,
	REG_PWM11 = 0x0D,
	REG_PWM12 = 0x0E,
	REG_PWM13 = 0x0F,
	REG_PWM14 = 0x10,
	REG_PWM15 = 0x11,
	REG_GRPPWM = 0x12,
	REG_GRPFREQ = 0x13,
	REG_LEDOUT0 = 0x14,
	REG_LEDOUT1 = 0x15,
	REG_LEDOUT2 = 0x16,
	REG_LEDOUT3 = 0x17,
	REG_SUBADR1 = 0x18,
	REG_SUBADR2 = 0x19,
	REG_SUBADR3 = 0x1A,
	REG_ALLCALLADR = 0x1B,
	REG_IREF = 0x1C,
	REG_EFLAG1 = 0x1D,
	REG_EFLAG2 = 0x1E
};

enum tlc59116_mode1 {
	MODE1_AI2 = 0x80,
	MODE1_AI1 = 0x40,
	MODE1_AI0 = 0x20,
	MODE1_SLEEP = 0x10,
	MODE1_SUB1 = 0x08,
	MODE1_SUB2 = 0x04,
	MODE1_SUB3 = 0x02,
	MODE1_ALLCALL = 0x01
};

enum tlc59116_mode2 {
	MODE2_EFCLR = 0x80,
	MODE2_DMBLNK = 0x20,
	MODE2_OCH = 0x08
};


enum tlc59116_ledlevel {
	LEDOUT_OFF = 0x00,
	LEDOUT_FULLON = 0x55,
	LEDOUT_BRIGHT = 0xAA,
	LEDOUT_BRIGHTBLINK = 0xFF
};

enum tlc59116_addr {
	LED_ADDR0 = 0x60,
	LED_ADDR1 = 0x31,
	LED_ADDR2 = 0x32,
	LED_ADDR3 = 0x33,
	LED_ADDR4 = 0x34,
	LED_ADDR5 = 0x35,
	LED_ADDR6 = 0x36,
	LED_ADDR7 = 0x37,
	LED_ADDR8 = 0x38,
	LED_ADDR9 = 0x39,
	LED_ADDR10 = 0x3a,
	LED_ADDR_RESET_ALL = 0x3b,
	LED_ADDR11 = 0x3c
};


class TLC59116 {
public:
    TLC59116(I2C& i2c, uint8_t address = LED_ADDR0);
    ~TLC59116();

    void init();
    bool update();
    void setLED(uint8_t num, uint8_t value);
    uint8_t getLED(uint8_t num) { return m_led_buf[num % 16]; }

protected:
  I2C& m_i2c;
  uint8_t m_address;
  uint8_t m_led_buf[16];

};

#endif
