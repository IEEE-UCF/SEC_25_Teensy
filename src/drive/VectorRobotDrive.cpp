#include "VectorRobotDrive.h"

/**
 * Initializes a robot drive, where the drive is based on a vector Pose2D input
 */
VectorRobotDrive::VectorRobotDrive(const MotorSetup motorSetups[], int numMotors)
    : SimpleRobotDrive(motorSetups, numMotors), lastVelocityPose(0, 0, 0)
{
}

/**
 * Set motor values based on velocities.
 *
 * @param velocityPose motor velocities in inches per second
 */
void VectorRobotDrive::Set(const Pose2D &velocityPose)
{
    Constrain(velocityPose);
    // lastVelocityPose = velocityPose;
    int i = 0;
    while (i < numMotors)
    {
        switch (i)
        {
            /*The robot would have a hard time going max speed while turning due to constrainSpeedPose + lastVelocityPose not actually being
            constrained to (-1, 1) naturally. However, this shouldn't be an issue as we are NOT running the robot THAT fast.

            Also. We have to account for the middle motor being not completely in the center, so theta also applies there. Middle wheel
            also being multiplied by a constant to make up for only having one motor
            */

        case 0:
            motors[i]->Set((lastVelocityPose.y - lastVelocityPose.theta * TRACK_WIDTH / 2) / WHEEL_CIRCUMFERENCE / MOTOR_RPS_NOLOAD * 255);
            break;

        case 1:
            motors[i]->Set(1.2f * (lastVelocityPose.x + lastVelocityPose.theta * BACK_OFFSET_F) / WHEEL_CIRCUMFERENCE / MOTOR_RPS_NOLOAD * 255);
            break;
        case 2:
            motors[i]->Set((lastVelocityPose.y + lastVelocityPose.theta * TRACK_WIDTH / 2) / WHEEL_CIRCUMFERENCE / MOTOR_RPS_NOLOAD * 255);

            break;
        }
        i++;
    }
}

Pose2D VectorRobotDrive::Constrain(const Pose2D &speed)
{
    static elapsedMicros timer = 0;
    if (timer > 1000)
    {
        Pose2D velocityPose = speed;

        // Velocity constraints
        velocityPose.normalize();
        velocityPose.xymag = constrain(velocityPose.xymag, 0, MAX_VELOCITY);
        velocityPose.unnormalize();
        velocityPose.theta = constrain(velocityPose.theta, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

        // Acceleration constraints
        float dt = timer / 1000000;
        Pose2D accel = velocityPose;
        accel.subtract(lastVelocityPose).multConstant(1 / dt); // Get theta
        accel.normalize();
        accel.xymag = constrain(accel.xymag, 0, MAX_ACCELERATION);
        accel.unnormalize();
        accel.theta = constrain(accel.theta, -MAX_ANGULAR_ACCELERATION, MAX_ANGULAR_ACCELERATION);

        lastVelocityPose = lastVelocityPose.add(accel.multConstant(dt));

        timer = 0;
    }
    return lastVelocityPose;
}
/**
 * Return speedPos
 *
 * @return Stored write values for motor velocities
 */
Pose2D VectorRobotDrive::GetVelocity()
{
    return lastVelocityPose;
}