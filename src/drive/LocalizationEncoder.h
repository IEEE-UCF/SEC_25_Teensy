#ifndef LOCALIZATIONENCODER_H
#define LOCALIZATIONENCODER_H

#include "math/Pose2D.h"
#include <Arduino.h>

class LocalizationEncoder {
public:
    LocalizationEncoder(Print &output);
    void updatePosition(const long encoderCounts[3]);
    Pose2D getPosition() const; // Added const
    void setPosition(const Pose2D &transform);
    void PrintInfo(Print &output) const; // Added const
    friend Print &operator<<(Print &output, const LocalizationEncoder &transform);

private:
    Print& output;
    Pose2D transform;
    long previousLeftTicks = 0;
    long previousBackTicks = 0;
    long previousRightTicks = 0;
};
#endif
