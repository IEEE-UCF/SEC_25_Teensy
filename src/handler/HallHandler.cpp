#include "HallHandler.h"
#include <Arduino.h>

HallHandler::HallHandler(int *kPins, int numPins)
{
    this->kPins = kPins;
    this->numPins = numPins;
    analogValues = new int[numPins];
}

HallHandler::~HallHandler()
{
    delete[] analogValues;
}

void HallHandler::Begin()
{
    analogReadResolution(8);
    analogReadAveraging(32);
    for (int i = 0; i < numPins; i++)
    {
        pinMode(kPins[i], INPUT);
    }
}

void HallHandler::Update()
{
    for (int i = 0; i < numPins; i++)
    {
        analogValues[i] = analogRead(kPins[i]);
    }
}

void HallHandler::PrintInfo(Print &output, bool printConfig) const
{
    if (printConfig)
    {
        output.print(F("HallHandler Configuration: "));
        output.print(F("Number of Pins: "));
        output.println(numPins);
        output.print(F("Pins: "));
        for (int i = 0; i < numPins; i++)
        {
            output.print(kPins[i]);
            if (i < numPins - 1)
            {
                output.print(F(", "));
            }
        }
        output.println();
    }
    else
    {
        output.print(F("Analog Values: "));
        for (int i = 0; i < numPins; i++)
        {
            output.print(analogValues[i]);
            if (i < numPins - 1)
            {
                output.print(F(", "));
            }
        }
        output.println();
    }
}

Print &operator<<(Print &output, const HallHandler &handler)
{
    handler.PrintInfo(output, false);
    return output;
}