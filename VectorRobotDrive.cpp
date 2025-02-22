#include "VectorRobotDrive.h"
#include <Arduino.h>
#include <Print.h>


VectorRobotDrive::VectorRobotDrive(int kPWM[], int kCW[], int kENC[], bool rev[], int numMotors)
  : speedPose(0, 0, 0), kPWM(kPWM), kCW(kCW), kENC(kENC), rev(rev), numMotors(numMotors) {
  enc = new int[numMotors];
  motors = new DriveMotor*[numMotors]; // Allocate memory for motor pointers
  for (int i = 0; i < numMotors; i++) {
    enc[i] = 0;
    motors[i] = new DriveMotor(kPWM[i], kCW[i], kENC[i], rev[i]);
  }
}

void VectorRobotDrive::Set(const Pose2D &speedPose) {
  this->speedPose = speedPose;
}

void VectorRobotDrive::Set(int motorDirectSpeed[]) {
  for (int i = 0; i < numMotors; i++) {
    motors[i]->Set(motorDirectSpeed[i]);
  }
}

void VectorRobotDrive::SetIndex(int motorDirectSpeed, int index) {
  if(index >= numMotors) {
    Serial.println(F("Motor out of index"));
    return;
  }
  motors[index]->Set(motorDirectSpeed);
}

void VectorRobotDrive::ReadEnc() {
  for (int i = 0; i < numMotors; i++) {
    motors[i]->ReadEnc();
    enc[i] = motors[i]->GetEnc();
  }
}

Pose2D VectorRobotDrive::GetPose() {
  return speedPose;
}

int *VectorRobotDrive::GetEnc() {
  return enc;
}

void VectorRobotDrive::Write() {
  for (int i = 0; i < numMotors; i++) {
    motors[i]->Write();
  }
}

void VectorRobotDrive::PrintInfo(Print &output, bool printConfig) const {
  if (printConfig) {
    output.print(F("VectorRobotDrive Configuration: "));
    output.print(F("Number of Motors: "));
    output.println(numMotors);
    for (int i = 0; i < numMotors; i++) {
      output.print(F("Motor "));
      output.print(i);
      output.print(F(" - kPWM: "));
      output.print(kPWM[i]);
      output.print(F(", kCW: "));
      output.print(kCW[i]);
      output.print(F(", kENC: "));
      output.print(kENC[i]);
      output.print(F(", kRev: "));
      output.println(rev[i] ? F("True") : F("False"));
    }
  } else {
    for (int i = 0; i < numMotors; i++) {
      output.print(F("Motor "));
      output.print(i);
      output.println(F(":"));
      motors[i]->PrintInfo(output, false);
    }
  }
}

Print &operator<<(Print &output, const VectorRobotDrive &drive) {
  drive.PrintInfo(output, false);
  return output;
}
