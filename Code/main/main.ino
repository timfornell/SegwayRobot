/* External libraries */
#include <Arduino.h>
#include <Wire.h> // Must include Wire library for I2C
#include <SparkFun_MMA8452Q.h> // Includes the SFE_MMA8452Q library

/* Local libraries */
#include "MotorController.hpp"
#include "CommandHandling.hpp"
#include "ekf.hpp"

/* Static variables */
static MMA8452Q accel; // Accelerometer
static const float referenceAngle = 21;
static boolean TUNING_MODE = true;

/* Function definitions */
void setup(void)
{
    Serial.begin(9600);

    /* Accelerometer */
    Wire.begin();
    accel.init(SCALE_2G);

    initializeEkfFilter(referenceAngle);
    setupMotorController();
    setupCommandHandler();
}

void loop(void)
{
    ekfTimeUpdate();
    
    AccelerometerData accData;
    const boolean accDataAvailable = getAccelerometerData(accel, accData);
    
    if(TUNING_MODE && Serial.available())
    {
        String string = Serial.readStringUntil('\n');
        parseCommandLine(string);
        Serial.println("Parsing complete.");
    }
    else if (accDataAvailable)
    {
        ekfMeasurementUpdate(accData.ax, accData.az);
        motorController(referenceAngle, accData);
    }
}
