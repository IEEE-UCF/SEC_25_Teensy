#include "VectorRobotDrive.h"

VectorRobotDrive::VectorRobotDrive(const MotorSetup motorSetups[], int numMotors, Print &output)
    : SimpleRobotDrive(motorSetups, numMotors, output),
      speedPose(0, 0, 0) {}

void VectorRobotDrive::Set(const Pose2D &speedPose) {
    this->speedPose = speedPose;
    
    using namespace MotorConstants;
    // Validate TRACK_WIDTH before calculations
    if (TRACK_WIDTH < 1e-6f) {
        output.println(F("Error: Invalid TRACK_WIDTH!"));
        return;
    }

    // Compile-time constant coefficients [X, Y, Theta]
    static constexpr float motorCoeffs[3][3] = {
        {0.0f, 1.0f, -1.0f}, // Left motor
        {1.0f, 0.0f, 0.0f},  // Back motor
        {0.0f, 1.0f, 1.0f}   // Right motor
    };

    for (size_t i = 0; i < static_cast<size_t>(numMotors); ++i) {
        if (i >= motors.size()) {
            output.println(F("Error: Motor index out of bounds"));
            continue;
        }

        const float xTerm = motorCoeffs[i][0] * speedPose.getX();
        const float yTerm = motorCoeffs[i][1] * speedPose.getY();
        const float thetaTerm = motorCoeffs[i][2] * speedPose.getTheta() * TRACK_WIDTH;
        
        const float motorSpeed = (xTerm + yTerm + thetaTerm) 
                               / WHEEL_CIRCUMFERENCE 
                               / MOTOR_RPS_NOLOAD 
                               * 255.0f;

        motors[i]->Set(static_cast<int>(constrain(motorSpeed, -255, 255)));
    }
}

Pose2D VectorRobotDrive::GetVelocity() const {
    return speedPose;