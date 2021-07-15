#ifndef MotorController_hpp
#define MotorController_hpp

/* External libraries */

/* Local libraries */
#include "Accelerometer.hpp"
#include "CommandHandling.hpp"

/* Definitions */
#define BUFFER_SIZE (10)
#define ERROR_BUFFER_SIZE (1)
#define dT (11e-6) // 10 microseconds

struct ControllerSavedValues
{
    float integralValue;
    float previousError;
};

struct ControllerValues
{
    float K;
    float Ti;
    float Td;
};

void setupMotorController(void);
void motorController(const float referenceValue, const AccelerometerData accData);
void setControllerParameter_K(const String commandParameters[], const int numParameters);
void setControllerParameter_Ti(const String commandParameters[], const int numParameters);
void setControllerParameter_Td(const String commandParameters[], const int numParameters);
void resetIntergralPart(const String commandParameters[], const int numParameters);
void printPidValues(const String commandParameters[], const int numParameters);

#endif
