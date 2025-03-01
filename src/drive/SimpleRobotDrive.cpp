#include "SimpleRobotDrive.h"
#include <Arduino.h>
#include <Print.h>

/**
 * Initializes a robot drive.
 *
 * @param motorSetups array of motors for setup
 * @param numMotors number of motors
 */
SimpleRobotDrive::SimpleRobotDrive(const MotorSetup motorSetups[], int numMotors)
    : numMotors(numMotors), localization()
{
  enc = new long[numMotors];
  motors = new DriveMotor *[numMotors]; // Allocate memory for motor pointers
  for (int i = 0; i < numMotors; i++)
  {
    enc[i] = 0;
    motors[i] = new DriveMotor(motorSetups[i]);
  }
}

/**
 * Runs begin method for each motor
 */
void SimpleRobotDrive::Begin()
{
  for (int i = 0; i < numMotors; i++)
  {
    motors[i]->Begin();
  }
}

/**
 * Sets value for each motor
 *
 * @param motorDirectSpeed array of speeds
 */
void SimpleRobotDrive::Set(int motorDirectSpeed[])
{
  for (int i = 0; i < numMotors; i++)
  {
    motors[i]->Set(constrain(motorDirectSpeed[i], -100, 100));
  }
}

/**
 * Sets value for a motor with index
 * @param motorDirectSpeed speed
 * @param index index
 */
void SimpleRobotDrive::SetIndex(int motorDirectSpeed, int index)
{
  if (index >= numMotors)
  {
    Serial.println(F("Motor out of index"));
    return;
  }
  motors[index]->Set(motorDirectSpeed);
}

/**
 * Read each encoder and update the values
 */
void SimpleRobotDrive::ReadEnc()
{
  for (int i = 0; i < numMotors; i++)
  {
    motors[i]->ReadEnc();
    enc[i] = motors[i]->GetEnc();
  }
}

/**
 * Read both encoder and update localization
 */
void SimpleRobotDrive::ReadAll()
{
  ReadEnc();                        // Update encoder counts
  localization.updatePosition(enc); // Update localization using encoder counts
  localization.transform.fixTheta();
}

long *SimpleRobotDrive::GetEnc()
{
  return enc;
}

/**
 * Write stored speed values to motors
 */
void SimpleRobotDrive::Write()
{
  for (int i = 0; i < numMotors; i++)
  {
    motors[i]->Write();
  }
}

/**
 * Get transform of robot
 *
 * @return transform
 */
Pose2D SimpleRobotDrive::GetPosition() const
{
  return localization.getPosition();
}

void SimpleRobotDrive::PrintInfo(Print &output, bool printConfig) const
{
  if (printConfig)
  {
    output.print(F("SimpleRobotDrive Configuration: "));
    output.print(F("Number of Motors: "));
    output.println(numMotors);
    for (int i = 0; i < numMotors; i++)
    {
      motors[i]->PrintInfo(output, true);
    }
  }
  else
  {
    for (int i = 0; i < numMotors; i++)
    {
      output.print(F("Motor "));
      output.print(i);
      output.print(F(": "));
      motors[i]->PrintInfo(output, false);
    }
  }
}

void SimpleRobotDrive::PrintLocal(Print &output) const
{
  localization.PrintInfo(output);
}

Print &operator<<(Print &output, const SimpleRobotDrive &drive)
{
  drive.PrintInfo(output, false);
  drive.PrintLocal(output);
  return output;
}
