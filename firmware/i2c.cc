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