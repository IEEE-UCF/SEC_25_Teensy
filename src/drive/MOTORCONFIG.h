#ifndef MOTORCONFIG_H
#define MOTORCONFIG_H

/*
 Translation Constants
 */
#define TRACK_WIDTH 10       // inches, distance from encoder wheel to encoder wheel
#define WHEEL_DIAMETER 3.25f // inches
constexpr float WHEEL_RADIUS = WHEEL_DIAMETER * 0.5;
constexpr float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

// Wheel Offsets
#define WHEEL_OFFSET_Y 2 // inches, offset of the left and right wheels from the center
#define BACK_OFFSET_F 4  // inches, offset of the back wheel from the center

/*
Motor Constants
*/

#define RAW_MOTOR_RPM_NOLOAD 2700  // rpm, no gears
#define GEAR_RATIO 34              // gear ratio
#define RAW_TICKS_PER_REVOLUTION 3 // encoder ticks per wheel revolution
constexpr long TICKS_PER_REVOLUTION = RAW_TICKS_PER_REVOLUTION * GEAR_RATIO;
constexpr long MOTOR_RPM_NOLOAD = RAW_MOTOR_RPM_NOLOAD / GEAR_RATIO;
constexpr float IN_PER_TICK = WHEEL_CIRCUMFERENCE / TICKS_PER_REVOLUTION;

#define MAX_VELOCITY 10    // inches per second
#define MAX_ACCELERATION 5 // inches per second^2

#endif
