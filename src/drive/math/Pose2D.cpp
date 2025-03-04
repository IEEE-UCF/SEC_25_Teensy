#include "Pose2D.h"
#include <Arduino.h>
#include <math.h>

Pose2D::Pose2D(float x, float y, float theta, float xymag, bool debug)
    : x(x), y(y), theta(theta), xymag(xymag), normalized(false), debugMode(debug) {
    if (this->xymag < 1e-6f) this->xymag = 1.0f;
    fixTheta();
}

Pose2D &Pose2D::normalize() {
    if (!normalized) {
        xymag = hypotf(x, y);
        constexpr float EPSILON = 1e-6f;
        if (xymag < EPSILON) {
            xymag = 1.0f;
            x = 1.0f;
            y = 0.0f;
        } else {
            x /= xymag;
            y /= xymag;
        }
        normalized = true;
    }
    return *this;
}

Pose2D &Pose2D::fixTheta() {
    theta = fmod(theta, 2 * PI);
    while (theta >= PI) theta -= 2 * PI;
    while (theta < -PI) theta += 2 * PI;
    return *this;
}

Pose2D &Pose2D::fullNormalize() {
    normalize();
    fixTheta();
    return *this;
}

Pose2D &Pose2D::unnormalize(bool scaleTheta) {
    if (normalized) {
        constexpr float MIN_XYMAG = 1e-3f;
        if (xymag < MIN_XYMAG) xymag = MIN_XYMAG;
        
        x *= xymag;
        y *= xymag;
        if (scaleTheta) theta *= xymag;
        
        xymag = 1.0f;
        normalized = false;
    }
    return *this;
}

Pose2D &Pose2D::rotateVector(float angle) {
    const float cosA = cosf(angle);
    const float sinA = sinf(angle);
    return rotateVectorCached(cosA, sinA);
}

Pose2D &Pose2D::rotateVectorCached(float cosAngle, float sinAngle) {
    const float newX = x * cosAngle - y * sinAngle;
    const float newY = x * sinAngle + y * cosAngle;
    x = newX;
    y = newY;
    return *this;
}

Pose2D &Pose2D::add(const Pose2D &pose) {
    x += pose.x;
    y += pose.y;
    theta += pose.theta;
    fixTheta();
    return *this;
}

Pose2D &Pose2D::subtract(const Pose2D &pose) {
    x -= pose.x;
    y -= pose.y;
    theta -= pose.theta;
    fixTheta();
    return *this;
}

Pose2D &Pose2D::multElement(const Pose2D &pose) {
    x *= pose.x;
    y *= pose.y;
    theta *= pose.theta;
    return *this;
}

Pose2D &Pose2D::multConstant(float scalar) {
    x *= scalar;
    y *= scalar;
    theta *= scalar;
    return *this;
}

Pose2D &Pose2D::reset() {
    x = 0;
    y = 0;
    theta = 0;
    return *this;
}

Pose2D &Pose2D::translate(float dx, float dy) {
    x += dx;
    y += dy;
    return *this;
}

Pose2D &Pose2D::rotate(float dtheta) {
    theta += dtheta;
    fixTheta();
    return *this;
}

Print &operator<<(Print &output, const Pose2D &pose) {
    float thetaDeg = degrees(pose.theta);
    output.print(F("Pose: ("));
    output.print(pose.x);
    output.print(F(", "));
    output.print(pose.y);
    output.print(F(", "));
    output.print(thetaDeg);
    output.println(F("°)"));
    return output;
}
