

// Include Libraries
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include "src/handler/ButtonHandler.h"
#include "src/handler/GyroHandler.h"
#include "src/handler/LightHandler.h"
#include "src/handler/HallHandler.h"
#include "src/handler/LineHandler.h"
#include "src/handler/RCHandler.h"
#include "src/handler/RGBHandler.h"
#include "src/handler/ROSHandler.h"
#include "src/handler/ServoHandler.h"
#include "src/handler/TOFHandler.h"
#include "src/subsystem/SorterSubsystem.h"

#include "src/drive/math/Pose2D.h"

#include "src/drive/VectorRobotDrive.h"
#include "src/drive/SimpleRobotDrive.h"
#include "src/drive/DriveMotor.h"

/*
--- Board Setup ---
*/

// Counts
#define DRIVEMOTOR_COUNT 3
#define NONDRIVEMOTOR_COUNT 2
#define SERVO_COUNT 4
#define TOF_COUNT 4
#define HALL_COUNT 3
#define BUTTON_COUNT 4

// Motor pins
/**
 * kPwm, kCW, kENCA, kENCB, rev
 */
MotorSetup driveMotors[DRIVEMOTOR_COUNT] = {
    {10, 24, 3, 4, true},  // left
    {12, 11, 5, 6, false}, // center
    {25, 9, 7, 8, false}   // right
};

MotorSetup nonDriveMotors[NONDRIVEMOTOR_COUNT] = {
    {28, 29, 31, 30, false}, // intake
    {33, 32, -1, -1, false}  // transfer
};

/**
 * Servo pins: 23, 22, 0, 1, 2
 * TOF channels: 1-7
 * Hall pins: 41, 15, 21, 20
 * Button/Dip pins: 40, 39, 38, 37
 * LED pin: 14
 * RC Rx: Serial8
 * BNO I2C: SDA/SCL
 * TOF/Light I2C: SDA1/SCL1
 */
int kServo[SERVO_COUNT] = {23, 22, 0, 1};
int cTOF[TOF_COUNT] = {1, 2, 3, 4};
int kHall[HALL_COUNT] = {41, 15, 21};
int kButton[BUTTON_COUNT] = {40, 39, 38, 37};
int kLED = 14;
int cLight = 0;

/*
--- Handlers ---
*/
TOFHandler tofs(cTOF, TOF_COUNT);
GyroHandler gyro;
LightHandler light(cLight);
HallHandler halls(kHall, HALL_COUNT);
ButtonHandler buttons(kButton, BUTTON_COUNT);
RGBHandler rgb(kLED);

/*
--- Subsystems ---
*/

/*
--- Program Control ---
*/
int state = 0;

void setup()
{
  delay(500);
  Serial.begin(115200);
  while(!Serial) {
    delay(1);
  }
  Serial.println("Hello");

  Wire.setClock(400000);
  Wire1.setClock(1000000);
  Wire.begin();
  Wire1.begin();

  // Initialize Handlers
  tofs.Begin();
  gyro.Begin();
  light.Begin();
  halls.Begin();
  buttons.Begin();
  //rgb.Begin();
  tofs.PrintInfo(Serial, true);
  gyro.PrintInfo(Serial, true);
  light.PrintInfo(Serial, true);
  halls.PrintInfo(Serial, true);
  buttons.PrintInfo(Serial, true);

  // Initialize Subsystems
  // Initialize Program Control
  state = 0;
  delay(3000);
}

void loop()
{
  // Read/Update
  static elapsedMillis read = 0;
  if (read > 100)
  {
    read = 0;
    tofs.Update();
    light.Update();
    halls.Update();
    buttons.Update();
  }
  static elapsedMillis fastRead = 0;
  if(fastRead > 20) {
    fastRead = 0;
    gyro.Update();
  }

  // Print
  static elapsedMillis printTimer = 0;
  if (printTimer > 100)
  {
    printTimer = 0;
    Serial << tofs << gyro << light << halls << buttons;
    Serial.println();
  }

  // Statistics
  static elapsedMillis fpsTimer = 0;
  static long cycles = 0;
  cycles++;
  if(fpsTimer >= 1000) {
    Serial.print("Cycles: ");
    Serial.println(cycles);
    fpsTimer = 0;
    cycles = 0;
  }
}