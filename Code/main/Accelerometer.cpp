/* Local libraries */
#include "Accelerometer.hpp"

boolean getAccelerometerData(MMA8452Q &accel, AccelerometerData &accData)
{
    const boolean dataAvailable = accel.available();
    if (dataAvailable)
    {
        accel.read();
        accData.ax = accel.cx;
        accData.ay = accel.cy;
        accData.az = accel.cz;
    }

    return dataAvailable;
}
