#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "src/handler/GyroHandler.h"
#include "src/handler/LineHandler.h"
#include "src/handler/ButtonHandler.h"
// #include "ServoHandler.h"
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
// #define SERVO_COUNT 3
#define BUTTON_COUNT 2
#define HALL_COUNT 2
// #define TOF_COUNT 2
#define LINE_COUNT 3

/**
 *  kPWM, kCW, kENCA, kENCB, rev
 */
MotorSetup driveMotors[DRIVEMOTOR_COUNT] = {
    {9, 32, 7, 8, true},   // left
    {11, 27, 5, 6, false}, // center
    {10, 26, 3, 4, false}  // right
};

MotorSetup nonDriveMotors[NONDRIVEMOTOR_COUNT] = {
    {29, 30, 31, 30, true}, // intake
    {-1, -1, -1, -1, false} // empty placeholder
};

// Other Input Pins
// const int kButton[BUTTON_COUNT] = {33, 33};
// const int kHall[HALL_COUNT] = {33, 33};
// const int kTOF[TOF_COUNT] = {33, 33};
// const int kLine[LINE_COUNT] = {33, 33, 33};

// Other Output Pins
// const int kServo[SERVO_COUNT] = {33, 33, 33};

// Input Handlers
// ButtonHandler buttons(kButton, BUTTON_COUNT);
// HallHandler halls(kHall, HALL_COUNT);
// TOFHandler tofs(kTOF, TOF_COUNT);
GyroHandler gyro;
RCHandler rc;

// LineHandler lines(kLine, LINE_COUNT);

// Output Handlers
VectorRobotDrive robotDrive(driveMotors, DRIVEMOTOR_COUNT, Serial);
// ServoHandler servos(kServo, SERVO_COUNT);
DriveMotor intake(nonDriveMotors[0], Serial);
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
  rc.Begin(Serial1);
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
  gyro.Update();
  rc.Update();
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
    float y = map((float)constrain(rc.Get(2), -255, 255), -255, 255, -1, 1);               // LPot Y
    float x = map((float)constrain(rc.Get(3), -255, 255), -255, 255, -1, 1);               // LPot X
    float theta = map((float)constrain(rc.Get(0), -255, 255), -255, 255, -2 * PI, 2 * PI); // RPot X
    float angleOffset = -gyro.GetGyroData()[2];
    Pose2D toWrite(x, y, theta);
    toWrite.normalize();
    Serial << toWrite;
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
