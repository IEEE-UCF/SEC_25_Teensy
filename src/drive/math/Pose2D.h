#ifndef Pose2D_H
#define Pose2D_H

#include <Arduino.h>
#include <cmath>

class Pose2D {
public:
    Pose2D(float x = 0, float y = 0, float theta = 0, float xymag = 1, bool debug = false);

    float getX() const { return x; }
    float getY() const { return y; }
    float getTheta() const { return theta; }
    float getXyMag() const { return xymag; }

    void setX(float val) { x = val; normalized = false; }
    void setY(float val) { y = val; normalized = false; }
    void setThetaRadians(float val) { theta = val; fixTheta(); }
    void setThetaDegrees(float deg) { theta = radians(deg); fixTheta(); }

    Pose2D &normalize();
    Pose2D &fixTheta();
    Pose2D &fullNormalize();
    Pose2D &unnormalize(bool scaleTheta = false);

    Pose2D &rotateVector(float angle);
    Pose2D &rotateVectorCached(float cosAngle, float sinAngle);

    Pose2D &add(const Pose2D &pose);
    Pose2D &subtract(const Pose2D &pose);
    Pose2D &multElement(const Pose2D &pose);
    Pose2D &multConstant(float scalar);
    Pose2D &reset();
    Pose2D &translate(float dx, float dy);
    Pose2D &rotate(float dtheta);

    friend Print &operator<<(Print &output, const Pose2D &pose);

private:
    float x, y, theta;
    float xymag;
    bool normalized;
    bool debugMode;
};
#endif