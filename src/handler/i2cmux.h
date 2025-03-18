#ifndef I2CMUX_H
#define I2CMUX_H

#include <Wire.h>

#define TCAADDR 0x70

namespace i2cmux
{
    void tcaselect(uint8_t i);
}

#endif