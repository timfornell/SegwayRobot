/* External libraries */
#include <BasicLinearAlgebra.h>

/* Local libraries */
#include "CommonDefinitions.hpp"
#include "ekf.hpp"

using namespace BLA;

/* Static variables */
static EkfState state;
/* Noises and uncertainties are specified in standard deviation */
static float measurementNoise = 1e1;
static float processNoise = 1e3;

/* Function definitions */
void initializeEkfFilter(const float referenceAngle)
{
    const float initialAngle = referenceAngle;
    const float initialAngleVel = 0;
    /* Noises and uncertainties are specified in standard deviation */
    const float initialAngleCov = 2.0;
    const float initialAngleVelCov = 1.0;
    
    /* Initial state guess */
    state.state << initialAngle,
                   initialAngleVel;
    state.covariance << pow(initialAngleCov, 2), 0, 
                        0,                       pow(initialAngleVelCov, 2);
}

EkfParameters getEkfParameters()
{
    EkfParameters params;

    /* Constant "velocity" model */
    params.F << 1.0,  dT,
                0.0, 1.0;
    params.L << 0.5 * pow(dT, 2),
                dT;
    params.Q = pow(processNoise, 2);
        
    /* The angle is measured directly from the accelerometer by theta = arctan(x / z) */
    params.H << 1.0, 0.0;
    params.R = pow(measurementNoise, 2);

    return params;
}

Matrix<numStates, numStates> getIdentityMatrix(void)
{
    const Matrix<numStates, numStates> identityMatrix = {1, 0, 0, 1};
    return identityMatrix;
}

void ekfTimeUpdate(void)
{
    EkfParameters p = getEkfParameters();
    state.state = p.F * state.state;
    state.covariance = p.F * state.covariance * ~p.F.Ref() + p.L * p.Q * ~p.L.Ref();

    // Serial.print("TimeUpdate: "); Serial.print(state(0)); Serial.print(", ");Serial.print(state(1));
}

void ekfMeasurementUpdate(const float ax, const float az)
{
    EkfParameters p = getEkfParameters();
    // Serial.println("Measurement update");
    const float measuredAngle = 180 * atan2(ax, az) / PI;
    // Serial.print("Angle: "); Serial.println(measuredAngle);
    Matrix<numMeasurements, 1> error = measuredAngle - state.state(0);
    // Serial.print("Error: "); Serial.println(error(0));

    Matrix<numMeasurements, numMeasurements> S = p.H * state.covariance * ~p.H.Ref() + p.R;
    // Serial.print("S (");
    // Serial.print(S.GetRowCount()); Serial.print(", ");
    // Serial.print(S.GetColCount()); Serial.print("): ");
    // Serial.println(S(0));
    
    Matrix<numStates, numMeasurements> K = state.covariance * ~p.H.Ref() * S.Ref().Inverse();
    // Serial.print("K (");
    // Serial.print(K.GetRowCount()); Serial.print(", ");
    // Serial.print(K.GetColCount()); Serial.print("): ");
    // Serial.print(K(0)); Serial.print(", "); Serial.println(K(1));

    state.state = state.state + K * error;
    state.covariance = (getIdentityMatrix() - K *p.H) * state.covariance;

    // Serial.print("MeasUpdate: "); Serial.print(state(0)); Serial.print(", ");Serial.print(state(1));
}

float ekfGetAngle(void)
{
    return state.state(0);
}

float ekfGetAngleVelocity(void)
{
    return state.state(1);
}

void ekfSetQValue(const String commandParameters[], const int numParameters)
{
    Serial.println("Q value");
    Serial.println("Setting Q to first parameters...");
    if (numParameters > 0)
    {
        const float new_Q = commandParameters[0].toFloat();
        Serial.print("New Q: "); 
        Serial.println(new_Q);
        Serial.print("Old Q: "); 
        Serial.println(processNoise);
        processNoise = new_Q;
    }
}

void ekfSetRValue(const String commandParameters[], const int numParameters)
{
    Serial.println("R value");
    Serial.println("Setting R to first parameters...");
    if (numParameters > 0)
    {
        const float new_R = commandParameters[0].toFloat();
        Serial.print("New R: "); 
        Serial.println(new_R);
        Serial.print("Old R: "); 
        Serial.println(measurementNoise);
        measurementNoise = new_R;
    }
}