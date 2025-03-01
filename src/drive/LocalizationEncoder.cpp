#include "LocalizationEncoder.h"
#include "MOTORCONFIG.h"
#include <Arduino.h>

LocalizationEncoder::LocalizationEncoder() : transform(0, 0, 0)
{
}

/**
 * Update position of the encoder.
 *
 * @param encoderCounts Array of encoder counts, {left, back, right}
 */
void LocalizationEncoder::updatePosition(const long encoderCounts[3])
{
    static long previousLeftTicks = 0;
    static long previousBackTicks = 0;
    static long previousRightTicks = 0;

    long leftChangeTicks = encoderCounts[0] - previousLeftTicks;
    long backChangeTicks = encoderCounts[1] - previousBackTicks;
    long rightChangeTicks = encoderCounts[2] - previousRightTicks;

    previousLeftTicks = encoderCounts[0];
    previousBackTicks = encoderCounts[1];
    previousRightTicks = encoderCounts[2];

    // Calculate the distance each wheel has moved
    float leftDistance = leftChangeTicks * IN_PER_TICK; // all in inches
    float backDistance = backChangeTicks * IN_PER_TICK;
    float rightDistance = rightChangeTicks * IN_PER_TICK;

    // Calculate the change in orientation (theta)
    float deltaTheta = (rightDistance - leftDistance) / TRACK_WIDTH; // get angular change

    // Calculate the change in position
    // float deltaX = ((leftDistance + rightDistance) / 2) * cos(transform.theta) + backDistance * sin(transform.theta) - WHEEL_OFFSET_Y * deltaTheta * sin(transform.theta);
    // float deltaY = ((leftDistance + rightDistance) / 2) * sin(transform.theta) - backDistance * cos(transform.theta) + WHEEL_OFFSET_Y * deltaTheta * cos(transform.theta) + BACK_OFFSET_F * deltaTheta * cos(transform.theta);

    // The second term of deltaXRobotFrame is the comensation for what the back wheel will experience when the robot is rotating.
    // This must also be accounted for in the kinematics, or else the motor will slip
    float deltaYRobotFrame = (leftDistance + rightDistance) / 2;
    float deltaXRobotFrame = ((leftDistance - rightDistance) * BACK_OFFSET_F / TRACK_WIDTH) + backDistance;

    float deltaX = deltaXRobotFrame * cos(transform.theta) + deltaYRobotFrame * sin(transform.theta);
    float deltaY = deltaXRobotFrame * sin(transform.theta) + deltaYRobotFrame * cos(transform.theta);

    // Update the robot's position
    Pose2D delta(deltaX, deltaY, deltaTheta);
    transform.add(delta);
    transform.fixTheta();
}

/**
 * Return transform of the robot.
 *
 * @return Transform of robot.
 */
Pose2D LocalizationEncoder::getPosition() const
{
    return transform;
}

/**
 * Override and set position of robot.
 *
 * @param transform Pose2D of new position
 */
void LocalizationEncoder::setPosition(const Pose2D &transform)
{
    this->transform = transform;
    this->transform.fixTheta();
}

void LocalizationEncoder::PrintInfo(Print &output) const
{
    output.println(F("Localization Encoder Information:"));

    output.print(F("Translation X: "));
    output.println(transform.x);

    output.print(F("Translation Y: "));
    output.println(transform.y);

    output.print(F("Translation Theta: "));
    output.println(transform.theta);

    output.println(F("Encoder Counts:"));
}

Print &operator<<(Print &output, const LocalizationEncoder &encoder)
{
    encoder.PrintInfo(output);
    return output;
}