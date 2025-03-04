#ifndef VectorRobotDrive_h
#define VectorRobotDrive_h

#include "SimpleRobotDrive.h"
#include "MOTORCONFIG.h"

class VectorRobotDrive : public SimpleRobotDrive {
public:
    VectorRobotDrive(const MotorSetup motorSetups[], int numMotors, Print &output);
    void Set(const Pose2D &speedPose);
    Pose2D GetVelocity() const;

private:
    Pose2D speedPose;
};
#endif