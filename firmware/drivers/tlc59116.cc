#include "tlc59116.hh"
#include "i2c.hh"

TLC59116::TLC59116(uint8_t address) : m_address(address),
                                      m_led_buf{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
{
}

TLC59116::~TLC59116()
{
}

void TLC59116::init(I2C *i2cbus)
{
    m_i2c = i2cbus;

    // reset chip
    // m_i2c->writeRegister(m_address, 0xA5, 0x5A);

    m_i2c->writeRegister(m_address, REG_MODE1, 0x00);
    m_i2c->writeRegister(m_address, REG_MODE2, MODE2_DMBLNK);


    m_i2c->writeRegister(m_address, REG_GRPPWM, 0x7F); // 50% duty cycle
    m_i2c->writeRegister(m_address, REG_GRPFREQ, 12);  // ~0.5sec

    update();
}

bool TLC59116::update()
{
    uint8_t buf[17];
    unsigned int bidx = 0;

    buf[0] = REG_PWM0 | MODE1_AI2 | MODE1_AI0;

    for (unsigned int j = 0; j < 16; j++)
    {
        buf[j + 1] = m_led_buf[bidx++];

    
    }

    m_i2c->writeRegister(m_address, REG_LEDOUT0, (m_led_mode[3]&0x03)<<6 | (m_led_mode[2]&0x03)<<4 | (m_led_mode[1]&0x03)<<2 | (m_led_mode[0]&0x03));
    m_i2c->writeRegister(m_address, REG_LEDOUT1, (m_led_mode[7]&0x03)<<6 | (m_led_mode[6]&0x03)<<4 | (m_led_mode[5]&0x03)<<2 | (m_led_mode[4]&0x03));
    m_i2c->writeRegister(m_address, REG_LEDOUT2, (m_led_mode[11]&0x03)<<6 | (m_led_mode[10]&0x03)<<4 | (m_led_mode[9]&0x03)<<2 | (m_led_mode[8]&0x03));
    m_i2c->writeRegister(m_address, REG_LEDOUT3, (m_led_mode[15]&0x03)<<6 | (m_led_mode[14]&0x03)<<4 | (m_led_mode[13]&0x03)<<2 | (m_led_mode[12]&0x03));


    if (!m_i2c->writeBuffer(m_address, buf, 17))
    {
        return false;
    }

    return true;
}

void TLC59116::setLED(uint8_t num, uint8_t value, uint8_t mode, bool update_now)
{
    m_led_buf[num % 16] = value;
    m_led_mode[num % 16] = mode;
    if (update_now)
        update();
}