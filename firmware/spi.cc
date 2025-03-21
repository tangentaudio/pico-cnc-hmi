#include <stdio.h>
#include "spi.hh"
#include "pico/stdlib.h"
#include "pico/binary_info.h"

SPI::SPI()
{
}

SPI::~SPI()
{
}

void SPI::init()
{
    // Enable SPI 0 and connect to GPIOs
    spi_init(spi0, SPI_MHZ * 1000 * 1000);

    spi_set_format(spi0, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function(SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(SPI_CSN_PIN, GPIO_FUNC_SPI);
    bi_decl(bi_3pins_with_func(SPI_TX_PIN, SPI_SCK_PIN, SPI_CSN_PIN, GPIO_FUNC_SPI));    
}

