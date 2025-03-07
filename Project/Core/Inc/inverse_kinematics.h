#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H
#include "stm32f1xx_hal.h"

typedef struct {
    float d1;
    float a2;
    float a3;
    float a4;
    float d5;
} InverseKinematics;
void MatMultiply(const float* _matrix1, const float* _matrix2, float* _matrixOut, const int _m, const int _l, const int _n);
void InverseKinematics_Init(InverseKinematics* ik, float d1, float a2, float a3, float a4, float d5);
void InverseKinematics_Calculate(InverseKinematics* ik, float object[3], float theta[5]);

#endif // INVERSE_KINEMATICS_H
