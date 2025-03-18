#include "LightHandler.h"

LightHandler::LightHandler(int cLight) : cLight(cLight), lightMeter(), lightLevel(0.0)
{
}

void LightHandler::Begin()
{
    i2cmux::tcaselect(cLight);

    if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire1))
    {
        Serial.println("Light sensor initialized");
    }
    else
    {
        Serial.println("Error initializing light sensor");
    }
}

void LightHandler::Update()
{
    i2cmux::tcaselect(cLight);
    lightLevel = lightMeter.readLightLevel();
}

void LightHandler::PrintInfo(Print &output, bool printConfig) const
{
    output.println("Light Sensor Info:");
    output.print("Light Level: ");
    output.print(lightLevel);
    output.println(" lx");

    if (printConfig)
    {
        output.println("Configuration:");
        output.println("Mode: Continuous High Resolution");
    }
}

float LightHandler::GetLightLevel() const
{
    return lightLevel;
}

Print &operator<<(Print &output, const LightHandler &handler)
{
    handler.PrintInfo(output);
    return output;
}