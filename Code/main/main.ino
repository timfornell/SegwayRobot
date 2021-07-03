/* External libraries */
#include <DebugLog.h>

/* Local libraries */
#include "MotorController.hpp"
#include "Accelerometer.hpp"
#include "CommandHandling.hpp"


void setup(void)
{
    // Setup debug level
    const DebugLogLevel debugLogLevel = DebugLogLevel::VERBOSE;
    // LOG_SET_LEVEL(DebugLogLevel::ERRORS); // only ERROR log is printed
    // LOG_SET_LEVEL(DebugLogLevel::WARNINGS); // ERROR and WARNING is printed
    // LOG_SET_LEVEL(DebugLogLevel::VERBOSE); // all log is printed
    LOG_SET_LEVEL(debugLogLevel);

    LOG_VERBOSE("Main starting...");
    Serial.begin(9600);
    setupAccelerometer();
    setupMotorController();
    setupCommandHandler(debugLogLevel);
    LOG_VERBOSE("Main setup finished!");
}

void loop(void)
{
    const AccelerometerData accData = getAccelerometerData();
    motorController(0, accData);

    if(Serial.available())
    {
        LOG_VERBOSE("Data received.");
        parseCommandLine();
        LOG_VERBOSE("Parsing complete.");
    }
}
