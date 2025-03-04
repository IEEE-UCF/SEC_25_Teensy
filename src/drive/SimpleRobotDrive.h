#ifndef SIMPLEROBOTDRIVE_H
#define SIMPLEROBOTDRIVE_H

#include "DriveMotor.h"
#include "LocalizationEncoder.h"
#include <Arduino.h>
#include <Print.h>
#include <vector>
#include <memory>

class SimpleRobotDrive {
public:
    SimpleRobotDrive(const MotorSetup motorSetups[], int numMotors, Print &output);
    void Begin();
    void Set(const int motorDirectSpeed[]);
    void SetIndex(int motorDirectSpeed, int index);
    void ReadAll();
    void Write();
    void PrintInfo(Print &output, bool printConfig = false) const;
    void PrintLocal(Print &output) const;
    Pose2D GetPosition() const;

protected:
    const int numMotors;
    Print& output;
    std::unique_ptr<long[]> enc;
    std::vector<std::unique_ptr<DriveMotor>> motors;
    LocalizationEncoder localization;
    
    void ReadEnc();
    const long* GetEnc() const;

    friend Print &operator<<(Print &output, const SimpleRobotDrive &drive);
};

Print &operator<<(Print &output, const SimpleRobotDrive &drive);

#endif
