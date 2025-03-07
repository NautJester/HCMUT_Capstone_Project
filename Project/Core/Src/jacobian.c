#include "jacobian.h"
#include <math.h>

#define EPSILON 1e-6

Matrix DHMatrix(float a, float alpha, float d, float theta) {
    Matrix T;
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);
    float cos_alpha = cosf(alpha);
    float sin_alpha = sinf(alpha);

    T.data[0][0] = cos_theta; T.data[0][1] = -sin_theta * cos_alpha; T.data[0][2] = sin_theta * sin_alpha; T.data[0][3] = a * cos_theta;
    T.data[1][0] = sin_theta; T.data[1][1] = cos_theta * cos_alpha;  T.data[1][2] = -cos_theta * sin_alpha; T.data[1][3] = a * sin_theta;
    T.data[2][0] = 0.0f;      T.data[2][1] = sin_alpha;              T.data[2][2] = cos_alpha;            T.data[2][3] = d;
    T.data[3][0] = 0.0f;      T.data[3][1] = 0.0f;                   T.data[3][2] = 0.0f;                 T.data[3][3] = 1.0f;

    return T;
}

Matrix MatMultiply_1(Matrix A, Matrix B) {
    Matrix C;
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            C.data[i][j] = 0.0f;
            for (int k = 0; k < 4; ++k) {
                C.data[i][j] += A.data[i][k] * B.data[k][j];
            }
        }
    }
    return C;
}

void ComputeJacobian(float theta1, float theta2, float theta3, float theta4, float theta5, float Jacobian[6][5]) {
    // Define the DH parameters
    float a[5] = {0.0f, 10.5f, 13.0f, 0.0f, 0.0f};
    float alpha[5] = {M_PI_2, 0.0f, 0.0f, M_PI_2, 0.0f};
    float d[5] = {5.5f, 0.0f, 0.0f, 0.0f, 11.0f};
    float theta[5] = {theta1, theta2, theta3, theta4, theta5};

    // Compute the transformation matrices
    Matrix T01 = DHMatrix(a[0], alpha[0], d[0], theta[0]);
    Matrix T12 = DHMatrix(a[1], alpha[1], d[1], theta[1]);
    Matrix T23 = DHMatrix(a[2], alpha[2], d[2], theta[2]);
    Matrix T34 = DHMatrix(a[3], alpha[3], d[3], theta[3]);
    Matrix T45 = DHMatrix(a[4], alpha[4], d[4], theta[4]);

    Matrix T02 = MatMultiply_1(T01, T12);
    Matrix T03 = MatMultiply_1(T02, T23);
    Matrix T04 = MatMultiply_1(T03, T34);
    Matrix T05 = MatMultiply_1(T04, T45);

    // Extract positions (origins) from the transformation matrices
    float p0[3] = {0.0f, 0.0f, 0.0f}; // Base
    float p1[3] = {T01.data[0][3], T01.data[1][3], T01.data[2][3]};
    float p2[3] = {T02.data[0][3], T02.data[1][3], T02.data[2][3]};
    float p3[3] = {T03.data[0][3], T03.data[1][3], T03.data[2][3]};
    float p4[3] = {T04.data[0][3], T04.data[1][3], T04.data[2][3]};
    float p5[3] = {T05.data[0][3], T05.data[1][3], T05.data[2][3]}; // End-effector

    // Z-axes (unit vectors) for each joint in the base frame
    float z0[3] = {0.0f, 0.0f, 1.0f};
    float z1[3] = {T01.data[0][2], T01.data[1][2], T01.data[2][2]};
    float z2[3] = {T02.data[0][2], T02.data[1][2], T02.data[2][2]};
    float z3[3] = {T03.data[0][2], T03.data[1][2], T03.data[2][2]};
    float z4[3] = {T04.data[0][2], T04.data[1][2], T04.data[2][2]};

    // Compute the linear velocity part of the Jacobian (Jv)
    for (int i = 0; i < 3; ++i) {
        Jacobian[i][0] = z0[i] * (p5[i] - p0[i]); // Jv1
        Jacobian[i][1] = z1[i] * (p5[i] - p1[i]); // Jv2
        Jacobian[i][2] = z2[i] * (p5[i] - p2[i]); // Jv3
        Jacobian[i][3] = z3[i] * (p5[i] - p3[i]); // Jv4
        Jacobian[i][4] = z4[i] * (p5[i] - p4[i]); // Jv5
    }

    // Compute the angular velocity part of the Jacobian (Jw)
    for (int i = 0; i < 3; ++i) {
        Jacobian[i + 3][0] = z0[i]; // Jw1
        Jacobian[i + 3][1] = z1[i]; // Jw2
        Jacobian[i + 3][2] = z2[i]; // Jw3
        Jacobian[i + 3][3] = z3[i]; // Jw4
        Jacobian[i + 3][4] = z4[i]; // Jw5
    }
}

int CheckSingularity(float Jacobian[6][5]) {
    // Check if the Jacobian matrix has any zero rows or columns,
    // or if the rank of the Jacobian is less than the number of degrees of freedom (5).
    int singular = 0;

    // Check for zero rows
    for (int i = 0; i < 6; ++i) {
        int is_zero_row = 1;
        for (int j = 0; j < 5; ++j) {
            if (fabs(Jacobian[i][j]) > EPSILON) {
                is_zero_row = 0;
                break;
            }
        }
        if (is_zero_row) {
            singular = 1;
            break;
        }
    }

    // Check for zero columns
    if (!singular) {
        for (int j = 0; j < 5; ++j) {
            int is_zero_col = 1;
            for (int i = 0; i < 6; ++i) {
                if (fabs(Jacobian[i][j]) > EPSILON) {
                    is_zero_col = 0;
                    break;
                }
            }
            if (is_zero_col) {
                singular = 1;
                break;
            }
        }
    }

    return singular;
}

void ComputeVelocities(float Jacobian[6][5], float joint_velocities[5], Velocity* end_effector_velocity) {
    for (int i = 0; i < 3; ++i) {
        end_effector_velocity->linear[i] = 0.0f;
        end_effector_velocity->angular[i] = 0.0f;
        for (int j = 0; j < 5; ++j) {
            end_effector_velocity->linear[i] += Jacobian[i][j] * joint_velocities[j];
            end_effector_velocity->angular[i] += Jacobian[i + 3][j] * joint_velocities[j];
        }
    }
}


