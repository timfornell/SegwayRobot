/* External libraries */
#include <RedBot.h>
#include <Vector.h>

/* Local libraries */
#include "MotorController.hpp"
#include "CommandHandling.hpp"

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
    float sum = 0;
    for (int i = 0; i < accelBuffer.size(); i++)
    {
        sum += accelBuffer[i];
    }

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
    pidValues.K = 5e1;
    pidValues.Ti = 1e-5; // Lower exponent => Bigger changes
    pidValues.Td = 1e-5; 
    previousTimeInstance.integralValue = 0;
    previousTimeInstance.previousError = 0;

    Serial.println("MotorController setup finished.");
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
    const float deltaError = currentError - previousError;
    const float P_current = pidValues.K * currentError;
    float I_current = I_previous + currentError * (pidValues.K * dT / pidValues.Ti);
    const float D_current = deltaError * (pidValues.K * pidValues.Td / dT);
    float controlSignal = P_current + I_current + D_current;

    if (abs(controlSignal) > 255)
    {
        I_current = I_previous; // Only done so that correct value is used in next iteration
        controlSignal = P_current + I_previous + D_current;
    }

    const float constrainedControlSignal = constrain(controlSignal, -255, 255);

    Serial.print("smooth: "); Serial.print(smoothedValue); Serial.print(", ");
    Serial.print("P: "); Serial.print(P_current); Serial.print(", ");
    Serial.print("I: "); Serial.print(I_current); Serial.print(", ");
    Serial.print("D: "); Serial.print(D_current); Serial.print(", ");
    Serial.print("v: "); Serial.print(controlSignal); Serial.print(", ");
    Serial.print("u: "); Serial.print(constrainedControlSignal); Serial.println();

    motors.leftMotor(- constrainedControlSignal);
    motors.rightMotor(constrainedControlSignal);

    previousTimeInstance.previousError = currentError;
    previousTimeInstance.integralValue = I_current;
}

void setControllerParameter_K(const String commandParameters[], const int numParameters)
{
    Serial.println("K parameter");
    for (int i = 0; i < numParameters; i++)
    {
        Serial.print(i);
        Serial.print(": ");
        Serial.println(commandParameters[i]);
    }
}

void setControllerParameter_Ti(const String commandParameters[], const int numParameters)
{
    Serial.println("Ti parameter");
    for (int i = 0; i < numParameters; i++)
    {
        Serial.print(i);
        Serial.print(": ");
        Serial.println(commandParameters[i]);
    }
}

void setControllerParameter_Td(const String commandParameters[], const int numParameters)
{
    Serial.println("Td parameter");
    for (int i = 0; i < numParameters; i++)
    {
        Serial.print(i);
        Serial.print(": ");
        Serial.println(commandParameters[i]);
    }
}