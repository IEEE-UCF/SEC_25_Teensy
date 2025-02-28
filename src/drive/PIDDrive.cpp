#include "PIDDrive.h"

PIDDrive::PIDDrive(const PIDConfig &xConfig, const PIDConfig &yConfig, const PIDConfig &thetaConfig, Print &serialOutput)
    : xPID(xConfig), yPID(yConfig), thetaPID(thetaConfig), serialOutput(serialOutput) {}

/**
 * Update the PID. Automatically integrates max acceleration and velocity acceleration constraints.
 *
 * @param currentPose current position of the robot in inches.
 * @param targetPose set position of the robot in inches.
 * @return velocity Pose2D in inches per second.
 */
Pose2D PIDDrive::Step(const Pose2D &currentPose, const Pose2D &targetPose)
{
    double xCommand = xPID.Step(currentPose.x, targetPose.x);
    double yCommand = yPID.Step(currentPose.y, targetPose.y);
    double thetaCommand = thetaPID.Step(currentPose.theta, targetPose.theta);
    return Pose2D(xCommand, yCommand, thetaCommand);
}