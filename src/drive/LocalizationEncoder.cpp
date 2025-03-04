#include "LocalizationEncoder.h"
#include "MOTORCONFIG.h"
#include <Arduino.h>

LocalizationEncoder::LocalizationEncoder(Print &output)
    : output(output), transform(0, 0, 0) {}

void LocalizationEncoder::updatePosition(const long encoderCounts[3]) {
    const float trackWidth = MotorConstants::TRACK_WIDTH;
    if (fabsf(trackWidth) < 1e-6f) {
        output.println(F("Error: TRACK_WIDTH invalid!"));
        return;
    }

    const long leftChangeTicks = encoderCounts[0] - previousLeftTicks;
    const long backChangeTicks = encoderCounts[1] - previousBackTicks;
    const long rightChangeTicks = encoderCounts[2] - previousRightTicks;

    previousLeftTicks = encoderCounts[0];
    previousBackTicks = encoderCounts[1];
    previousRightTicks = encoderCounts[2];

    const float leftDistance = leftChangeTicks * MotorConstants::IN_PER_TICK;
    const float backDistance = backChangeTicks * MotorConstants::IN_PER_TICK;
    const float rightDistance = rightChangeTicks * MotorConstants::IN_PER_TICK;

    const float deltaTheta = (rightDistance - leftDistance) / trackWidth;
    
    const float cosTheta = cosf(transform.getTheta());
    const float sinTheta = sinf(transform.getTheta());
    
    const float deltaX = ((leftDistance + rightDistance) * 0.5f * cosTheta) + 
                        (backDistance * sinTheta) - 
                        (MotorConstants::WHEEL_OFFSET_Y * deltaTheta * sinTheta);
                        
    const float deltaY = ((leftDistance + rightDistance) * 0.5f * sinTheta) - 
                        (backDistance * cosTheta) + 
                        (MotorConstants::WHEEL_OFFSET_Y * deltaTheta * cosTheta) + 
                        (MotorConstants::BACK_OFFSET_F * deltaTheta * cosTheta);

    transform.add(Pose2D(deltaX, deltaY, deltaTheta));
}

Pose2D LocalizationEncoder::getPosition() const {
    return transform;
}

void LocalizationEncoder::PrintInfo(Print &output) const {
    output.println(F("Localization Encoder Information:"));
    output.print(F("Translation X: "));
    output.println(transform.getX());
    output.print(F("Translation Y: "));
    output.println(transform.getY());
    output.print(F("Translation Theta: "));
    output.println(transform.getTheta());
    output.println(F("Encoder Counts:"));
}

Print &operator<<(Print &output, const LocalizationEncoder &encoder) {
    encoder.PrintInfo(output);
    return output;
}