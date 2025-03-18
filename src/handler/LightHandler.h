#ifndef LIGHTHANDLER_H
#define LIGHTHANDLER_H

#include <Wire.h>
#include <BH1750.h>
#include "i2cmux.h"

class LightHandler
{
public:
    LightHandler(int cLight);
    void Begin();
    void Update();
    void PrintInfo(Print &output, bool printConfig = false) const;
    float GetLightLevel() const;

private:
    int cLight;
    BH1750 lightMeter;
    float lightLevel;
};

// Overload the << operator for Print class
Print &operator<<(Print &output, const LightHandler &handler);

#endif // LIGHTHANDLER_H