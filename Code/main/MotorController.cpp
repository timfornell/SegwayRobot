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
    pidValues.K = 5e2;
    pidValues.Ti = 1e-3; // Lower exponent => Bigger changes
    pidValues.Td = 1e-4; 
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
    Serial.println("Setting K to first parameter...");
    if (numParameters > 0)
    {
        const float new_K = commandParameters[0].toFloat();
        Serial.print("New K: ");
        Serial.println(new_K);
        Serial.print("old K: ");
        Serial.println(pidValues.K);
        pidValues.K = new_K;
    }
}

void setControllerParameter_Ti(const String commandParameters[], const int numParameters)
{
    Serial.println("Ti parameter");
    Serial.println("Setting Ti to first parameter...");
    if (numParameters > 0)
    {
        const float new_Ti = commandParameters[0].toFloat();
        Serial.print("New Ti: ");
        Serial.println(new_Ti);
        Serial.print("old Ti: ");
        Serial.println(pidValues.Ti);
        pidValues.Ti = new_Ti;
    }
}

void setControllerParameter_Td(const String commandParameters[], const int numParameters)
{
    Serial.println("Td parameter");
    Serial.println("Setting Td to first parameter...");
    if (numParameters > 0)
    {
        const float new_Td = commandParameters[0].toFloat();
        Serial.print("New Td: ");
        Serial.println(new_Td);
        Serial.print("old Td: ");
        Serial.println(pidValues.Td);
        pidValues.Td = new_Td;
    }
}

void resetIntergralPart(const String commandParameters[], const int numParameters)
{
    Serial.println("Reset Integral part");
    previousTimeInstance.integralValue = 0;
}

void printPidValues(const String commandParameters[], const int numParameters)
{
    Serial.print("K: ");
    Serial.printn(pidValues.K);
    Serial.print("Ti: ");
    Serial.printn(pidValues.Ti);
    Serial.print("Td: ");
    Serial.printn(pidValues.Td);
}