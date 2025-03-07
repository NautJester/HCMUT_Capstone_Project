#ifndef JACOBIAN_H
#define JACOBIAN_H

typedef struct {
    float data[4][4];
} Matrix;

typedef struct {
    float linear[3];
    float angular[3];
} Velocity;

Matrix DHMatrix(float a, float alpha, float d, float theta);
Matrix MatMultiply(Matrix A, Matrix B);
void ComputeJacobian(float theta1, float theta2, float theta3, float theta4, float theta5, float Jacobian[6][5]);
int CheckSingularity(float Jacobian[6][5]);
void ComputeVelocities(float Jacobian[6][5], float joint_velocities[5], Velocity* end_effector_velocity);

#endif // JACOBIAN_H
