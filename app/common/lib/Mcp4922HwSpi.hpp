/*!
 * Mcp4922HwSpi
 * Copyright 2026 marksard
 * This software is released under the MIT license.
 * see https://opensource.org/licenses/MIT
 */

#pragma once

#include <Arduino.h>
#include "hardware/spi.h"

class Mcp4922HwSpi
{
public:
    Mcp4922HwSpi() {}

    void init(uint pinMosi, uint pinSck, uint pinCs, spi_inst_t *spi = spi1, uint32_t baudRate = 20000000)
    {
        _spi = spi;
        _pinCs = pinCs;
        spi_init(_spi, baudRate);
        gpio_set_function(pinSck, GPIO_FUNC_SPI);
        gpio_set_function(pinMosi, GPIO_FUNC_SPI);

        gpio_init(_pinCs);
        gpio_set_dir(_pinCs, GPIO_OUT);
        gpio_put(_pinCs, 1);

        spi_set_format(_spi, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    }

    inline void out1(uint16_t value)
    {
        if (value > 4095)
            value = 4095;

        gpio_put(_pinCs, 0);
        spi_get_hw(_spi)->dr = (confDacA | value); // FIFOへ書き込み

        while (spi_is_busy(_spi))
        {
            tight_loop_contents();
        }
        gpio_put(_pinCs, 1);
    }

    inline void out2(uint16_t value)
    {
        if (value > 4095)
            value = 4095;

        gpio_put(_pinCs, 0);
        spi_get_hw(_spi)->dr = (confDacB | value); // FIFOへ書き込み

        while (spi_is_busy(_spi))
        {
            tight_loop_contents();
        }
        gpio_put(_pinCs, 1);
    }

private:
    spi_inst_t *_spi;
    uint _pinCs;

    const uint16_t confDacA = 0x3000;
    const uint16_t confDacB = 0xB000;
};
