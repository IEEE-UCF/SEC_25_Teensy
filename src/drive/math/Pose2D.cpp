#include "Pose2D.h"
#include "Arduino.h"

/**
 * Creates a Pose2D. Has an x, y, and theta. Use however you'd like
 */
Pose2D::Pose2D(float x, float y, float theta, float xymag) : x(x), y(y), theta(theta), xymag(xymag), normalized(false) {}

/**
 * Normalize magnitude of x and y to 1, and theta from -1 to 1. Stores magnitude in xymag
 *
 */

Pose2D &Pose2D::normalize()
{
    if (!normalized)
    {
        xymag = sqrt(x * x + y * y);
        if (xymag == 0)
        {
            x = 0;
            y = 0;
        }
        else
        {
            x = x / xymag;
            y = y / xymag;
        }
        theta = theta / (2 * PI);
        normalized = true;
    }
    return *this;
}

Pose2D &Pose2D::unnormalize()
{
    if (normalized)
    {
        x *= xymag;
        y *= xymag;
        xymag = 1;
        theta = theta * (2 * PI);
        normalized = false;
    }
    else
    {
        Serial.println("Not already normalized!");
    }
    return *this;
}

/**
 * Get magnitude of vector component
 */
float Pose2D::magnitude()
{
    return sqrt(x * x + y * y) * xymag;
}
/**
 * Adds a pose. FixTheta must be manually called
 *
 * @param pose Pose2D to add.
 */
Pose2D &Pose2D::add(const Pose2D &pose)
{
    x += pose.x;
    y += pose.y;
    theta += pose.theta;
    return *this; // Return a reference to the current object
}

/**
 * Subtracts a pose. FixTheta must be manually called
 *
 * @param pose Pose2D to subtract.
 */
Pose2D &Pose2D::subtract(const Pose2D &pose)
{
    x -= pose.x;
    y -= pose.y;
    theta -= pose.theta;
    return *this; // Return a reference to the current object
}

/**
 * Multiplies 2 poses element by element. FixTheta must be maually called
 *
 * @param pose Pose2D to multiply element by element.
 */
Pose2D &Pose2D::multElement(const Pose2D &pose)
{
    x *= pose.x;
    y *= pose.y;
    theta *= pose.theta;
    return *this;
}

/**
 * Multiplies pose by a scalar. FixTheta must be manualley called
 *
 * @param pose Pose2D to multiply by scalar.
 */
Pose2D &Pose2D::multConstant(float scalar)
{
    x *= scalar;
    y *= scalar;
    theta *= scalar;
    return *this;
}

/**
 * Reset pose to (0,0,0).
 */
Pose2D &Pose2D::reset()
{
    x = 0;
    y = 0;
    theta = 0;
    return *this; // Return a reference to the current object
}

/**
 * Translate vector
 *
 * @param dx Amount to change x by.
 * @param dy Amount to change y by.
 */
Pose2D &Pose2D::translate(float dx, float dy)
{
    x += dx;
    y += dy;
    return *this; // Return a reference to the current object
}

/**
 * Add to theta.
 *
 * @param dtheta Amount to change theta by.
 */
Pose2D &Pose2D::rotate(float dtheta)
{
    theta += dtheta;
    fixTheta();
    return *this; // Return a reference to the current object
}

/**
 * Rotate the pose around (0,0) as if it were a vector. Theta stays the same
 *
 * @param angle Angle to rotate by
 */
Pose2D &Pose2D::rotateVector(float angle)
{
    float rad = radians(angle); // Convert angle to radians
    float cosA = cos(rad);
    float sinA = sin(rad);

    float newX = x * cosA - y * sinA;
    float newY = x * sinA + y * cosA;

    x = newX;
    y = newY;

    return *this; // Return a reference to the current object for chaining
}

/**
 * Check if theta overflows, and fix
 */

Pose2D &Pose2D::fixTheta()
{
    while (theta > PI)
    {
        theta -= 2 * PI;
    }
    while (theta < -PI)
    {
        theta += 2 * PI;
    }
    return *this;
}

Print &operator<<(Print &output, const Pose2D &pose)
{
    output.print(F("Pose: ("));
    output.print(pose.x);
    output.print(F(", "));
    output.print(pose.y);
    output.print(F(", "));
    output.print(pose.theta);
    output.println(F(")"));
    return output;
}
