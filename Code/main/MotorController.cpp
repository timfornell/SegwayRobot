// External libraries
#include <RedBot.h>
#include <DebugLog.h>
#include <Arduino.h>
#include <Vector.h>

/* Local libraries */
#include "MotorController.hpp"

/* Static variables */
static RedBotMotors motors;
static float accArray[BUFFER_SIZE];
Vector<float> accelBuffer(accArray);

static ControllerValues pidValues;
static ControllerSavedValues previousTimeInstance;

/* Static function declarations */
static float simpleMovingAverage();
static void addNewValueToAccelBuffer(const float value);

/* Static function definitions */
static float simpleMovingAverage()
{
    // Serial.print("Avg: ");
    float sum = 0;
    for (int i = 0; i < accelBuffer.size(); i++)
    {
        sum += accelBuffer[i];
    }
    // Serial.print("Return: "); Serial.println(sum / accelBuffer.size());
    return sum / accelBuffer.size();
}

static void addNewValueToAccelBuffer(const float value)
{
    if (accelBuffer.size() == accelBuffer.max_size())
    {
        accelBuffer.remove(0);
    }
    
    accelBuffer.push_back(value);
}

/* Function definitions */
void setupMotorController(void)
{
    // Setup debug level
    LOG_SET_LEVEL(DebugLogLevel::ERRORS); // only ERROR log is printed
    // LOG_SET_LEVEL(DebugLogLevel::WARNINGS); // ERROR and WARNING is printed
    // LOG_SET_LEVEL(DebugLogLevel::VERBOSE); // all log is printed
    
    Serial.begin(9600);
    pidValues.K = 1e1;
    pidValues.Ti = 1e-2;
    pidValues.Td = 1e0;
    previousTimeInstance.integralValue = 0;
    previousTimeInstance.previousError = 0;
    
    LOG_VERBOSE("MotorController.ino setup finished.");
}

void motorController(const float referenceValue, const AccelerometerData accData)
{
    const float measuredValue = accData.ax;
    addNewValueToAccelBuffer(measuredValue);
    const float smoothedValue = simpleMovingAverage();
    const float previousError = previousTimeInstance.previousError;
    const float currentError = referenceValue - smoothedValue;

    /*
     * Controller is on the following form:
     *    I_k = I_{k-1} + e_k * (K * dT / T_i)
     *    v_k = K * e_k + I_k + (e_k - e_{k-1}) * (K * T_d / dT)
     *    u_k = constrain(v_k, -255, 255)
     */

    const float I_previous = previousTimeInstance.integralValue;
    float I_current = I_previous + currentError * (pidValues.K * dT / pidValues.Ti);
    const float deltaError = currentError - previousError;
    float controlSignal = pidValues.K * currentError + I_current + deltaError * (pidValues.K * pidValues.Td / dT);
    
    if (abs(controlSignal) > 255)
    {
        I_current = I_previous;
        controlSignal = pidValues.K * currentError + I_current + deltaError * (pidValues.K * pidValues.Td / dT);
    }

    const float constrainedControlSignal = constrain(controlSignal, -255, 255);
    
    Serial.print("smooth: "); Serial.print(smoothedValue); Serial.print(", ");
    Serial.print("e: "); Serial.print(currentError); Serial.print(", ");
    Serial.print("I: "); Serial.print(I_current); Serial.print(", ");
    Serial.print("v: "); Serial.print(controlSignal); Serial.print(", ");
    Serial.print("u: "); Serial.print(constrainedControlSignal); Serial.println();

    motors.leftMotor(- constrainedControlSignal);
    motors.rightMotor(constrainedControlSignal);
    
    previousTimeInstance.previousError = currentError;
    previousTimeInstance.integralValue = I_current;
}