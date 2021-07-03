#ifndef Accelerometer_hpp
#define Accelerometer_hpp

/* Definitions */
struct AccelerometerData
{
    float ax;
    float ay;
    float az;
};

/* Function declarations */
void setupAccelerometer(void);
AccelerometerData getAccelerometerData(void);

#endif
