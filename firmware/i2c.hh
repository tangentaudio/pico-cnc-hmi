#ifndef __i2c_hh
#define __i2c_hh

#include "hardware/i2c.h"

#define I2C_SDA_PIN 20
#define I2C_SCL_PIN 21
#define I2C_KHZ 400

class I2C
{
public:
    I2C();
    ~I2C();

    void init();
    bool writeRegister(uint8_t address, uint8_t reg, uint8_t value);
    bool readRegister(uint8_t address, uint8_t reg, uint8_t &value);
    bool writeBuffer(uint8_t address, uint8_t *buf, uint8_t len);

private:
};

#endif