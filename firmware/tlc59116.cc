#include "tlc59116.hh"
#include "i2c.hh"

TLC59116::TLC59116(I2C& i2cbus, uint8_t address) :
    m_i2c(i2cbus),
    m_address(address),
    m_led_buf{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
{
}

TLC59116::~TLC59116()
{
}

void TLC59116::init()
{
    // reset chip
    //m_i2c.writeRegister(m_address, 0xA5, 0x5A);

    m_i2c.writeRegister(m_address, REG_MODE1, 0x00);

    m_i2c.writeRegister(m_address, REG_LEDOUT0, LEDOUT_BRIGHT);
    m_i2c.writeRegister(m_address, REG_LEDOUT1, LEDOUT_BRIGHT);
    m_i2c.writeRegister(m_address, REG_LEDOUT2, LEDOUT_BRIGHT);
    m_i2c.writeRegister(m_address, REG_LEDOUT3, LEDOUT_BRIGHT);

    update();
}


bool TLC59116::update()
{
    uint8_t buf[17];
    unsigned int bidx = 0;

    buf[0] = REG_PWM0 | MODE1_AI2 | MODE1_AI0;
    
    for (unsigned int j=0; j<16; j++) {
        buf[j + 1] = m_led_buf[bidx++];
    }
    
    if (!m_i2c.writeBuffer(m_address, buf, 17)) {
        return false;
    }

    return true;
}


void TLC59116::setLED(uint8_t num, uint8_t value)
{
    m_led_buf[num % 16] = value;
    update();
}