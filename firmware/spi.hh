#ifndef __spi_hh
#define __spi_hh

#include "hardware/spi.h"

#define SPI_TX_PIN 3
#define SPI_SCK_PIN 2
#define SPI_CSN_PIN 5
#define SPI_MHZ 1

class SPI
{
public:
    SPI();
    ~SPI();

    void init();

private:
};

#endif