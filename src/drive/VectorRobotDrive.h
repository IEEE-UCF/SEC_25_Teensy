#ifndef VectorRobotDrive_h
#define VectorRobotDrive_h

#include "SimpleRobotDrive.h"
#include "MOTORCONFIG.h"
#include <elapsedMillis.h>
#include <Arduino.h>

class VectorRobotDrive : public SimpleRobotDrive
{
public:
    VectorRobotDrive(const MotorSetup motorSetups[], int numMotors);
    void Set(const Pose2D &velocityPose);
    Pose2D GetVelocity();
    Pose2D Constrain(const Pose2D &velocityPose);
    Pose2D lastVelocityPose;
};

#endif
