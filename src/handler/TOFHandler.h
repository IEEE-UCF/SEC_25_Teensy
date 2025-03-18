#ifndef TOFHANDLER_H
#define TOFHANDLER_H

#include <Arduino.h>
#include <Print.h>
#include <Wire.h>
#include <VL53L0X.h>
#include "i2cmux.h"

class TOFHandler
{
public:
    TOFHandler(int *cToFs, int numChannels);
    ~TOFHandler();
    void Begin();
    void Update();
    const int *Get() const;
    int GetIndex(int index) const;
    void PrintInfo(Print &output, bool printConfig) const;
    friend Print &operator<<(Print &output, const TOFHandler &handler);

private:
    int *cToFs;
    int numChannels;
    VL53L0X *sensors;
    int *distances;
};

#endif