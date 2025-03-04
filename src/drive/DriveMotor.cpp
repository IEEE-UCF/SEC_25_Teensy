#include "DriveMotor.h"

int DriveMotor::encoderNum = 1;

DriveMotor::DriveMotor(const MotorSetup &motorSetup, Print &output)
    : motorSetup(motorSetup), output(output), pwmout(0), cwout(true), enc(0)
{
    if (encoderNum <= 4 && motorSetup.kENCA != -1 && motorSetup.kENCB != -1)
    {
        encoder = std::make_unique<QuadEncoder>(encoderNum, motorSetup.kENCA, motorSetup.kENCB);
        encoderNum++; // Only increment if encoder is created
    }
    else if (encoderNum > 4)
    {
        output.println(F("WARNING: Max encoder limit (4) reached!!!"));
    }
}

void DriveMotor::Begin()
{
    if (motorSetup.kCW >= 0)
        pinMode(motorSetup.kCW, OUTPUT);
    if (motorSetup.kPWM >= 0)
    {
        analogWriteFrequency(motorSetup.kPWM, 36621.09);
        pinMode(motorSetup.kPWM, OUTPUT);
    }
    if (encoder)
    {
        encoder->setInitConfig();
        encoder->EncConfig.decoderWorkMode = 1;
        encoder->init();
    }
}

void DriveMotor::Set(int speed)
{
    speed = motorSetup.rev ? -speed : speed;
    pwmout = map(abs(speed), 0, SPEED_MAX, PWM_MAX, 0);
    cwout = (speed >= 0);
}

void DriveMotor::ReadEnc()
{
    if (encoder)
        enc = encoder->read();
}

long DriveMotor::GetEnc() const
{
    return (motorSetup.kENCA == -1 || motorSetup.kENCB == -1) ? 0 : enc;
}

void DriveMotor::Write()
{
    if (motorSetup.kPWM >= 0)
        analogWrite(motorSetup.kPWM, pwmout);
    if (motorSetup.kCW >= 0)
        digitalWrite(motorSetup.kCW, cwout);
}

void DriveMotor::PrintInfo(Print &output, bool printConfig) const
{
    if (printConfig)
    {
        output.print(F("DriveMotor Configuration: "));
        output.print(F("kPWM: "));
        output.print(motorSetup.kPWM);
        output.print(F(", kCW: "));
        output.print(motorSetup.kCW);
        output.print(F(", kENC_A: "));
        output.print(motorSetup.kENCA);
        output.print(F(", kENC_B: "));
        output.print(motorSetup.kENCB);
        output.print(F(", kRev: "));
        output.println(motorSetup.rev ? F("True") : F("False"));
    }
    else
    {
        output.print(F("DriveMotor PWM Output: "));
        output.print(pwmout);
        output.print(F(", CW Output: "));
        output.print(cwout ? F("True") : F("False"));
        output.print(F(", Encoder: "));
        output.println(enc);
    }
}

Print &operator<<(Print &output, const DriveMotor &motor)
{
    motor.PrintInfo(output, false);
    return output;
}
