#include "VectorRobotDrive.h"

/**
 * Initializes a robot drive, where the drive is based on a NORMALIZED Pose2D Input
 */
VectorRobotDrive::VectorRobotDrive(const MotorSetup motorSetups[], int numMotors)
    : SimpleRobotDrive(motorSetups, numMotors), constrainedSpeedPose(0, 0, 0)
{
}

/**
 * Set motor values based on velocities. Input should be normalized
 *
 * @param speedPose motor velocities in inches per second
 */
void VectorRobotDrive::Set(const Pose2D &speedPose)
{
    Constrain(speedPose);
    int i = 0;
    while (i < numMotors)
    {
        switch (i)
        {
            /*The robot would have a hard time going max speed while turning due to constrainSpeedPose + constrainedSpeedPose not actually being
            constrained to (-1, 1) naturally. However, this shouldn't be an issue as we are NOT running the robot THAT fast.

            Also. We have to account for the middle motor being not completely in the center, so theta also applies there. Middle wheel
            also being multiplied by a constant to make up for only having one motor
            */
        case 0:
            motors[i]->Set(constrain((constrainedSpeedPose.y - constrainedSpeedPose.theta * TRACK_WIDTH) / WHEEL_CIRCUMFERENCE / MOTOR_RPS_NOLOAD * 255, -255, 255));
            break;
        case 1:
            motors[i]->Set(constrain(1.2 * (constrainedSpeedPose.x + constrainedSpeedPose * BACK_OFFSET_F) / WHEEL_CIRCUMFERENCE / MOTOR_RPS_NOLOAD * 255, -255, 255));
            break;
        case 2:
            motors[i]->Set(constrain((constrainedSpeedPose.y + constrainedSpeedPose.theta * TRACK_WIDTH) / WHEEL_CIRCUMFERENCE / MOTOR_RPS_NOLOAD * 255, -255, 255));
            break;
        }
        i++;
    }
}

Pose2D VectorRobotDrive::Constrain(const Pose2D &speedPose)
{
    static elapsedMillis timer = 0;
    static Pose2D previousCommand(0, 0, 0);
    if (timer > 10)
    {
        Pose2D command(speedPose.x, speedPose.y, speedPose.theta, speedPose.xymag);
        // Velocity constraints
        if (command.magnitude() > MAX_VELOCITY)
        {
            command.normalize();
            command.xymag = MAX_VELOCITY;
            command.unnormalize();
        }
        command.theta = constrain(command.theta, -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

        // Acceleration constraints
        Pose2D change = command;
        change.subtract(previousCommand);
        float dv = change.magnitude() / (timer / 1000);
        float adv = change.theta;
        if (dv > MAX_ACCELERATION)
        {
            command.normalize();
            command.xymag = previousCommand.magnitude() + MAX_ACCELERATION;
            command.unnormalize();
        }
        if (abs(adv) > MAX_ANGULAR_ACCELERATION)
        {
            command.theta = previousCommand.theta + (abs(adv) / adv) * MAX_ANGULAR_ACCELERATION;
        }
        previousCommand = command;
    }
    constrainedSpeedPose = previousCommand;
    return previousCommand;
}

/**
 * Return speedPose
 *
 * @return Stored write values for motor velocities
 */
Pose2D VectorRobotDrive::GetVelocity()
{
    return constrainedSpeedPose;
}