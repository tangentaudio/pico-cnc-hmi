#include <stdio.h>
#include "i2c.hh"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "tca8418.hh"

I2C::I2C()
{

}

I2C::~I2C()
{
    
}

void I2C::init()
{
    i2c_init(i2c0, I2C_KHZ*1000);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(I2C_SDA_PIN, I2C_SCL_PIN, GPIO_FUNC_I2C));    
}

bool I2C::writeRegister(uint8_t address, uint8_t reg, uint8_t value)
{
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = value;

    int r1 = i2c_write_blocking(i2c0, address, buf, 2, false);

    if (r1 == PICO_ERROR_GENERIC)
        printf("i2c write error addr %X\n", address);
    else
        printf("i2c write %X: reg=%x val=%x\n", address, reg, value);

    return r1 != PICO_ERROR_GENERIC;
}

bool I2C::writeBuffer(uint8_t address, uint8_t* buf, uint8_t len)
{
    int r1 = i2c_write_blocking(i2c0, address, buf, len, false);

    if (r1 == PICO_ERROR_GENERIC)
        printf("i2c write error addr %X\n", address);
    else {
        printf("i2c write %X: %d bytes:\n", address, len);
        for (uint8_t i=0; i<len; i++)
            printf("%2.2X ", buf[i]);
        printf("\n");
    }


    return r1 != PICO_ERROR_GENERIC;
}