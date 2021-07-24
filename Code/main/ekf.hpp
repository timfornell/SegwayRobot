#ifndef ekf_hpp
#define ekf_hpp

#include <BasicLinearAlgebra.h>

using namespace BLA;

/* Definitions */
#define numStates (2)
#define numMeasurements (1)

struct EkfParameters
{
    Matrix<numStates, numStates> F; // Process model
    Matrix<numStates, 1> L; // Process noise model
    Matrix<numMeasurements, numStates> H; // Measurement model
    float R; // Measurement noise
    float Q; // Process noise
};

struct EkfState
{
    Matrix<numStates, 1> state;
    Matrix<numStates, numStates> covariance;
};

void initializeEkfFilter(const float referenceAngle);
void ekfTimeUpdate(void);
void ekfMeasurementUpdate(const float ax, const float az);
float ekfGetAngle(void);
float ekfGetAngleVelocity(void);
void ekfSetQValue(const String commandParameters[], const int numParameters);
void ekfSetRValue(const String commandParameters[], const int numParameters);

#endif
