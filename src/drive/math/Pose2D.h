#ifndef Pose2D_H
#define Pose2D_H

#include <Arduino.h>

class Pose2D
{
public:
    float x, y, theta, xymag;

    Pose2D(float x, float y, float theta, float xymag = 1);

    Pose2D &normalize(Print &output);
    Pose2D &normalize();
    Pose2D &unnormalize(Print &output);
    Pose2D &unnormalize();
    float magnitude();
    Pose2D &add(const Pose2D &pose);
    Pose2D &subtract(const Pose2D &pose);
    Pose2D &multElement(const Pose2D &pose);
    Pose2D &multConstant(float scalar);
    Pose2D &reset();
    Pose2D &translate(float dx, float dy);
    Pose2D &rotate(float dtheta);
    Pose2D &fixTheta();
    virtual Pose2D &rotateVector(float angle);

    friend Print &operator<<(Print &output, const Pose2D &pose); // ChatGPT generated
protected:
    bool normalized;
    void normalizeError();
};

#endif