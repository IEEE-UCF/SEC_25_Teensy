#include "DriveMotor.h"
#include "Arduino.h"
#include "Print.h"

int DriveMotor::encoderNum = 1;

/**
 * Initializes the DriveMotor object. Note that encoders will cap out at 4
 *
 * @param motorSetup The motor pin configuration.
 */
DriveMotor::DriveMotor(const MotorSetup &motorSetup)
    : motorSetup(motorSetup), pwmout(0), cwout(true), enc(0)
{
  if (encoderNum <= 4)
  {
    encoder = new QuadEncoder(encoderNum, motorSetup.kENCA, motorSetup.kENCB);
  }
  else
  {
    encoder = NULL;
  }
  encoderNum++;
}

/**
 * Setup function for the motor
 */
void DriveMotor::Begin()
{
  pinMode(motorSetup.kCW, OUTPUT);
  analogWriteFrequency(motorSetup.kPWM, 36621.09);
  if (encoder)
  {
    encoder->setInitConfig();
    encoder->EncConfig.decoderWorkMode = 1;
    encoder->init();
  }
}

/**
 * Set speed of motor
 *
 * @param speed value from 0 to SPEED_MAX, int
 */
void DriveMotor::Set(int speed)
{
  if (motorSetup.rev)
  {
    speed = -speed;
  }
  pwmout = map(abs(speed), 0, SPEED_MAX, PWM_MAX, 0);
  cwout = (speed >= 0);
}

/**
 * Read encoder values
 */
void DriveMotor::ReadEnc()
{
  if (encoder)
  {
    enc = encoder->read();
  }
}

/**
 * Return encoder values
 *
 * @return encoder tick count
 */
long DriveMotor::GetEnc()
{
  if (motorSetup.kENCA == -1 || motorSetup.kENCB == -1)
  {
    return 0;
  }
  return enc;
}

/**
 * Write values to the motor
 */
void DriveMotor::Write()
{
  analogWrite(motorSetup.kPWM, pwmout);
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
    output.print(map(pwmout, 0, 255, 255, 0));
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