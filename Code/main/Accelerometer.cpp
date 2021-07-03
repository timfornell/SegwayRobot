/* External libraries */
#include <Wire.h> // Must include Wire library for I2C
#include <SparkFun_MMA8452Q.h> // Includes the SFE_MMA8452Q library
#include <DebugLog.h>
#include <Arduino.h>

/* Local libraries */
#include "Accelerometer.hpp"

/* Static variables */
static MMA8452Q accel; // Accelerometer

/* Function definitions */
void setupAccelerometer(void)
{
    accel.init(SCALE_2G);

    LOG_VERBOSE("Accelerometer setup finished.");
}

AccelerometerData getAccelerometerData(void)
{
    AccelerometerData accData;

    accel.read();
    accData.ax = accel.cx;
    accData.ay = accel.cy;
    accData.az = accel.cz;

    // Serial.print("X: "); Serial.print(accData.ax); Serial.print(" ");
    // Serial.print("Y: "); Serial.print(accData.ay); Serial.print(" ");
    // Serial.print("Z: "); Serial.print(accData.az); Serial.print(" ");

    return accData;
}
