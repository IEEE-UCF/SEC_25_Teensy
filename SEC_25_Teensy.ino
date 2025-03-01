#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "src/handler/GyroHandler.h"
#include "src/handler/LineHandler.h"
#include "src/handler/ButtonHandler.h"
#include "src/handler/ServoHandler.h"
#include "src/handler/HallHandler.h"
#include "src/handler/RCHandler.h"
// #include "TOFHandler.h"

#include "src/drive/math/Pose2D.h"

#include "src/drive/VectorRobotDrive.h"
#include "src/drive/SimpleRobotDrive.h"
#include "src/drive/DriveMotor.h"

// Constants
#define DRIVEMOTOR_COUNT 3
#define NONDRIVEMOTOR_COUNT 2
#define SERVO_COUNT 5
#define BUTTON_COUNT 2
#define HALL_COUNT 2
// #define TOF_COUNT 2
#define LINE_COUNT 3

/**
 *  kPWM, kCW, kENCA, kENCB, rev
 */
MotorSetup driveMotors[DRIVEMOTOR_COUNT] = {
    {10, 24, 3, 4, true},  // left
    {12, 11, 5, 6, false}, // center
    {25, 9, 7, 8, false}   // right
};

MotorSetup nonDriveMotors[NONDRIVEMOTOR_COUNT] = {
    {28, 29, 30, 31, true}, // intake
    {33, 32, -1, -1, false} // aoerwe
};

// Other Input Pins
// const int kButton[BUTTON_COUNT] = {33, 33};
// const int kHall[HALL_COUNT] = {33, 33};
// const int kTOF[TOF_COUNT] = {33, 33};
// const int kLine[LINE_COUNT] = {33, 33, 33};

// Other Output Pins
const int kServo[SERVO_COUNT] = {0, 1, 2, 23, 22};

// Input Handlers
// ButtonHandler buttons(kButton, BUTTON_COUNT);
// HallHandler halls(kHall, HALL_COUNT);
// TOFHandler tofs(kTOF, TOF_COUNT);
GyroHandler gyro;
RCHandler rc;

// LineHandler lines(kLine, LINE_COUNT);

// Output Handlers
VectorRobotDrive robotDrive(driveMotors, DRIVEMOTOR_COUNT);
ServoHandler servos(kServo, SERVO_COUNT);
DriveMotor intake(nonDriveMotors[0]);
// DriveMotor sorterMotor(nkPWM[1], nkCW[1], -1, -1, nrev[1]);

void setup()
{
  delay(500);
  Serial.begin(115200);
  Serial.println();

  // Initialize Handlers
  // buttons.Setup();
  // halls.Setup();
  // tofs.Begin();
  rc.Begin(Serial8);
  gyro.Setup();
  // lines.Setup();
  // servos.Setup();
  robotDrive.Begin();
  intake.Begin();

  // Set everything to 0
  robotDrive.Set(Pose2D(0, 0, 0));
  intake.Set(0);
  robotDrive.Write();
  intake.Write();

  robotDrive.PrintInfo(Serial, true);
  // servos.PrintInfo(Serial, true);
  intake.PrintInfo(Serial, true);
  // sorterMotor.PrintInfo(Serial, true);
  static long temp = millis();
  while (millis() - temp < 3000)
  {
    delay(100);
  }
}

/*
0 - startup
1 - running, controlled by Pi
2 - running, controlled by RC
*/

void loop()
{

  // Read/Update
  gyro.Read();
  rc.Read();
  robotDrive.ReadAll();
  intake.ReadEnc();

  // State Logic
  static int programState = 0;
  if (rc.Get(6) == 255)
  {
    programState = 1; // Down, turn RC off
  }
  else if (rc.Get(6) == -255)
  {
    programState = 2; // Up, turn RC on
  }

  Pose2D toWrite(0, 0, 0);
  // Main logic
  if (programState == 1)
  {
    Pose2D hi(0, 0, 0);
    robotDrive.Set(hi);
    intake.Set(0);
  }
  else if (programState == 2)
  {
    // Flysky inputs. Note that "?" is -255 and "?" is 255
    float x = map((float)constrain(rc.Get(0), -255, 255), -255, 255, -MAX_VELOCITY * 2, MAX_VELOCITY * 2);
    float y = map((float)constrain(rc.Get(1), -255, 255), -255, 255, -MAX_VELOCITY * 2, MAX_VELOCITY * 2);
    float theta = map((float)constrain(rc.Get(3), -255, 255), -255, 255, -MAX_ANGULAR_VELOCITY * 2, MAX_ANGULAR_VELOCITY * 2); // RPot X
    float angleOffset = -gyro.GetGyroData()[2];
    toWrite = Pose2D(x, y, theta);
    // Serial << toWrite;
    robotDrive.Set(toWrite) /*.rotateVector(angleOffset)*/;
    intake.Set(rc.Get(5));
  }

  intake.Write();
  robotDrive.Write();

  // Debug
  static elapsedMillis printTimer = 0;
  if (printTimer >= 100)
  {
    Serial.print("State: ");
    Serial.println(programState);

    printTimer -= 100;
    // Print Info
    Serial << robotDrive << intake;

    /*
    for (int i = 0; i < 10; i++)
    {
      Serial.print("channel: ");
      Serial.println(rc.Get(i));
    }*/

    Serial.print("Robot velocity pose ");
    Serial << robotDrive.constrainedSpeedPose;
    /*
    robotDrive.PrintInfo(Serial, false);
    robotDrive.PrintLocal(Serial);
    intake.PrintInfo(Serial, false);*/
    // sorterMotor.PrintInfo(Serial, false);
    // servos.PrintInfo(Serial, false);
    // halls.PrintInfo(Serial, false);
    // tofs.PrintInfo(Serial, false);
    // gyro.PrintInfo(Serial, false);
    // lines.PrintInfo(Serial, false);
  }
}
