#include "GyroHandler.h"

GyroHandler::GyroHandler() : bno08x(Adafruit_BNO08x(-1)) {}

/**
 * Setup function for the Gyro. Fails sometimes
 */
void GyroHandler::Begin()
{
    if (!bno08x.begin_I2C(0x4B))
    {
        // if (!bno08x.begin_UART(&Serial1)) {  // Requires a device with > 300 byte
        // UART buffer! if (!bno08x.begin_SPI(BNO08X_CS, BNO08X_INT)) {
        Serial.println("Failed to find BNO08x chip");
        while (1)
        {
            delay(10);
        }
    }
    Serial.println("BNO08x Found!");
    if (!bno08x.enableReport(SH2_ROTATION_VECTOR))
    {
        Serial.println(F("Could not enable rotation vector"));
    }
}

/**
 * Reads the gyro data
 */
void GyroHandler::Update()
{
    if (bno08x.getSensorEvent(&gyroEvent))
    {
        // Gyro data is now stored in gyroEvent
    }
    gyroData[0] = gyroEvent.un.rotationVector.i;
    gyroData[1] = gyroEvent.un.rotationVector.j;
    gyroData[2] = gyroEvent.un.rotationVector.k;
}

/**
 * Returns the gyro data
 *
 * @return float array, {x, y, z}
 */
float *GyroHandler::GetGyroData()
{
    return gyroData;
}

void GyroHandler::PrintInfo(Print &output, bool printConfig) const
{
    if (printConfig)
    {
        output.println(F("BNO08x Configuration:"));
        output.println(F("I2C Address: 0x55"));
    }
    else
    {
        output.print(F("Gyroscope: "));
        output.print(gyroData[0]);
        output.print(F(", "));
        output.print(gyroData[1]);
        output.print(F(", "));
        output.print(gyroData[2]);
        output.println();
    }
}

Print &operator<<(Print &output, const GyroHandler &handler)
{
    handler.PrintInfo(output, false);
    return output;
}