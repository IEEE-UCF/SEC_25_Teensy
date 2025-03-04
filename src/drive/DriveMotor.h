#ifndef DriveMotor_h
#define DriveMotor_h

#include <Arduino.h>
#include <Print.h>
#include <memory>
#include "QuadEncoder.h"

#define SPEED_MAX 255
#define PWM_MAX 255

struct MotorSetup {
    int kPWM;
    int kCW;
    int kENCA;
    int kENCB;
    bool rev;
};

class DriveMotor {
public:
    explicit DriveMotor(const MotorSetup &motorSetup, Print &output);
    virtual ~DriveMotor() = default;

    void Begin();
    void Set(int speed);
    void ReadEnc();
    long GetEnc() const;
    void Write();
    void PrintInfo(Print &output, bool printConfig = false) const;

    friend Print &operator<<(Print &output, const DriveMotor &motor);

private:
    MotorSetup motorSetup;
    Print& output;
    int pwmout;
    bool cwout;
    long enc;
    std::unique_ptr<QuadEncoder> encoder;
    static int encoderNum;
};

#endif