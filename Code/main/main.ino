/* External libraries */
#include <Arduino.h>
#include <Wire.h> // Must include Wire library for I2C
#include <SparkFun_MMA8452Q.h> // Includes the SFE_MMA8452Q library

/* Local libraries */
#include "MotorController.hpp"
#include "CommandHandling.hpp"

static boolean TUNING_MODE = true;

/* External libraries */

/* Local libraries */

/* Static variables */
static MMA8452Q accel; // Accelerometer

/* Function definitions */
void setup(void)
{
    Serial.begin(9600);

    /* Accelerometer */
    Wire.begin();
    accel.init(SCALE_2G);

    Serial.println("Main starting...");
    setupMotorController();
    setupCommandHandler();
    Serial.println("Main setup finished!");
}

void loop(void)
{
    AccelerometerData accData;
    const boolean accDataAvailable = getAccelerometerData(accel, accData);
    
    // if (accDataAvailable)
    // {
    //     motorController(0, accData);
    // }
    
    if(TUNING_MODE && Serial.available())
    {
        String string = Serial.readStringUntil('\n');
        Serial.print("Data received: ");
        Serial.println(string);
        parseCommandLine(string);
        Serial.println("Parsing complete.");
    }
}
