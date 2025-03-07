#include "inverse_kinematics.h"
#include <math.h>
#include <stdio.h>
#define PI 3.14159265358979323846

float cosf(float x)
{
	return cos(x);
}

float sinf(float x)
{
	return sin(x);
}

void MatMultiply(const float* _matrix1, const float* _matrix2, float* _matrixOut,

                        const int _m, const int _l, const int _n)
{
    float tmp;
    int i, j, k;
    for (i = 0; i < _m; i++)
    {
        for (j = 0; j < _n; j++)
        {
            tmp = 0.0f;
            for (k = 0; k < _l; k++)
            {
                tmp += _matrix1[_l * i + k] * _matrix2[_n * k + j];
            }
            _matrixOut[_n * i + j] = tmp;
        }
    }
}
// Function to compute the rotation matrix R0_3
void computeR0_3(float theta1, float theta2, float theta3, float R0_3[3][3]) {
    R0_3[0][0] = cos(theta1) * cos(theta2 + theta3);
    R0_3[0][1] = -cos(theta1) * sin(theta2 + theta3);
    R0_3[0][2] = sin(theta1);

    R0_3[1][0] = sin(theta1) * cos(theta2 + theta3);
    R0_3[1][1] = -sin(theta1) * sin(theta2 + theta3);
    R0_3[1][2] = -cos(theta1);

    R0_3[2][0] = sin(theta2 + theta3);
    R0_3[2][1] = cos(theta2 + theta3);
    R0_3[2][2] = 0;
}

// Function to compute the inverse of a 3x3 rotation matrix
float determinant3x3(float R[3][3]) {
    // Tính định thức của ma trận 3x3
    return R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1]) -
           R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0]) +
           R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]);
}

void invertMatrix3x3(float R[3][3], float R_inv[3][3]) {
    float det = determinant3x3(R);

    // Kiểm tra xem định thức có bằng 0 không
    if (det == 0) {
        printf("Matrix is singular and cannot be inverted.\n");
        return;
    }

    float invDet = 1.0 / det;

    // Tính ma trận phụ đại số (adjugate matrix) và nhân với 1/det
    R_inv[0][0] = invDet * (R[1][1] * R[2][2] - R[1][2] * R[2][1]);
    R_inv[0][1] = invDet * (R[0][2] * R[2][1] - R[0][1] * R[2][2]);
    R_inv[0][2] = invDet * (R[0][1] * R[1][2] - R[0][2] * R[1][1]);

    R_inv[1][0] = invDet * (R[1][2] * R[2][0] - R[1][0] * R[2][2]);
    R_inv[1][1] = invDet * (R[0][0] * R[2][2] - R[0][2] * R[2][0]);
    R_inv[1][2] = invDet * (R[0][2] * R[1][0] - R[0][0] * R[1][2]);

    R_inv[2][0] = invDet * (R[1][0] * R[2][1] - R[1][1] * R[2][0]);
    R_inv[2][1] = invDet * (R[0][1] * R[2][0] - R[0][0] * R[2][1]);
    R_inv[2][2] = invDet * (R[0][0] * R[1][1] - R[0][1] * R[1][0]);
}

void invertRotationMatrix(float R[3][3], float R_inv[3][3]) {
    // Tính định thức của ma trận quay
    float det = determinant3x3(R);

    // Nếu det ≈ 1, coi nó là ma trận quay và dùng chuyển vị
    if (det > 0.999 && det < 1.001) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                R_inv[i][j] = R[j][i];  // Ma trận nghịch đảo của ma trận quay bằng ma trận chuyển vị
            }
        }
    } else {
        // Nếu det khác 1, sử dụng công thức tính ma trận nghịch đảo thông thường
        invertMatrix3x3(R, R_inv);
    }
}

// Function to compute theta4 and theta5 from the rotation matrix R3_5
void computeTheta4Theta5(float R3_5[3][3], float *theta4, float *theta5) {
    // R3_5 is the rotation matrix from the 3rd joint to the end-effector

	float r11 = R3_5[0][0];
    float r12 = R3_5[0][1];
    float r13 = R3_5[0][2];

    float r21 = R3_5[1][0];
    float r22 = R3_5[1][1];
    float r23 = R3_5[1][2];

    float r31 = R3_5[2][0];
    float r32 = R3_5[2][1];
    float r33 = R3_5[2][2];


    // Calculate theta5
    *theta4 = atan2(r13, -r23);

    // Calculate theta4
    //*theta5 = atan2(-r23 / sin(*theta4), r13 / sin(*theta4));
    *theta5 = atan2(r31, r32);

    // Ensure the angles are in the range [-PI / 2, PI / 2]
    if (*theta4 < - PI ) *theta4 += PI ;
    if (*theta4 >= PI ) *theta4 -= PI ;

    if (*theta5 < - PI / 2) *theta5 += PI / 2;
    if (*theta5 >= PI / 2) *theta5 -= PI / 2;
}

void InverseKinematics_Init(InverseKinematics* ik, float d1, float a2, float a3, float a4, float d5) {
    ik->d1 = d1;
    ik->a2 = a2;
    ik->a3 = a3;
    ik->a4 = a4;
    ik->d5 = d5;
}


void InverseKinematics_Calculate(InverseKinematics* ik, float object[3], float theta[5])
{
	float x, y, z;

	x = object[0];
	y = object[1];

    // Compute theta1
	theta[0] = atan2(y, x);

	z = object[2] + (ik->a4 + ik->d5);

    // Compute r và s
    float r = hypot(x,y);
    float s = z - ik->d1;

    // Compute theta2 and theta3
    float D = (r * r + s * s - ik->a2 * ik->a2 - ik->a3 * ik->a3) / (- 2 * ik->a2 * ik->a3);
    if (D < -1 || D > 1)
    {
        // Unreachable target
        return;
    }

    theta[2] = atan2(sqrt(1 - D * D), D) + PI / 36;

    theta[1] = PI - atan2(s, r) - atan2(ik->a3 * sinf(PI - theta[2]), ik->a2 + ik->a3 * cosf(PI - theta[2]));


    theta[3] = PI + theta[2] - theta[1] - PI / 18;     //theta[3] = PI - (2 * PI - PI / 2 - theta[2] - PI + theta[1] - PI / 2)

    theta[4] =  theta[0] + PI / 36;

//float x = object[0] - (ik->d5)  * R0_5[0][2];
//float x = object[0] - (ik->a4 + ik->d5);
//float y = object[1] - (ik->d5)  * R0_5[1][2];
//float y = object[1] - (ik->a4 + ik->d5);

//	if (xc < 0.0)
//	{
//		x = -(fabs(xc) - (ik->a4 + ik->d5) * fabs(cos(theta[0])));
//	}
//	else if(xc >= 0.0)
//	{
//		x = (fabs(xc) - (ik->a4 + ik->d5) * fabs(cos(theta[0])));
//	}
//
//	if (yc < 0.0)
//	{
//		y = -(fabs(yc) - (ik->a4 + ik->d5) * fabs(sin(theta[0])));
//	}
//	else if(yc >= 0.0)
//	{
//		y = (fabs(yc) - (ik->a4 + ik->d5) * fabs(sin(theta[0])));
//	}

//    	// Compute rotation matrix R0_3
//       float R0_3[3][3];
//       computeR0_3(theta[0], theta[1], theta[2], R0_3);
//
//       // Compute the inverse of R0_3
//       float R0_3_inv[3][3];
//       invertRotationMatrix(R0_3, R0_3_inv);
//
//       float R3_5[3][3];
//       MatMultiply((const float*)R0_3_inv, (const float*)R0_5, (float*)R3_5, 3, 3, 3);
//
//       // Compute theta4 and theta5
//       computeTheta4Theta5(R3_5, &theta[3], &theta[4]);

}






