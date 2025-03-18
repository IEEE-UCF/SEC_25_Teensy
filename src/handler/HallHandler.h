#ifndef HALLHANDLER_H
#define HALLHANDLER_H

#include <Arduino.h>
#include <Print.h>

class HallHandler
{
public:
    HallHandler(int *kPins, int numPins);

    int *getReadings() const { return analogValues; }
    ~HallHandler();
    void Begin();
    void Update();
    void PrintInfo(Print &output, bool printConfig) const;
    friend Print &operator<<(Print &output, const HallHandler &handler);

private:
    int *kPins;
    int numPins;
    int *analogValues;
};

#endif