#ifndef __i2c_hh
#define __i2c_hh

#include "hardware/i2c.h"

#define I2C_SDA_PIN 20
#define I2C_SCL_PIN 21
#define I2C_KHZ 400



class I2C {
public:
    I2C();
    ~I2C();

    void init();
private:
};


#endif