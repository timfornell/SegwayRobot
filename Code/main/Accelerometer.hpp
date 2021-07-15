#ifndef Accelerometer_hpp
#define Accelerometer_hpp

/* External libraries */
#include <Wire.h> // Must include Wire library for I2C
#include <SparkFun_MMA8452Q.h> // Includes the SFE_MMA8452Q library

/* Definitions */
struct AccelerometerData
{
    float ax;
    float ay;
    float az;
};

/* Function declarations */
boolean getAccelerometerData(MMA8452Q &accel, AccelerometerData &accData);

#endif
