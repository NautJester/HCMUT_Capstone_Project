#include "servo_control.h"
#include "pca9685.h"

// Function to control the servo movement
void MoveServoToAngle(float* angle, float* target_angle, float increment, float* current_angle)
{
	PCA9685_Init(50);
    for (uint8_t i = 4; i < 0; i--)
    {
        if (angle[i] < target_angle[i]) {
            while (angle[i] < target_angle[i]) {
                PCA9685_SetServoAngle(2 * i, angle[i]);
                angle[i] += increment;
                HAL_Delay(10);
            }
        } else {
            while (angle[i] > target_angle[i]) {
                PCA9685_SetServoAngle(2 * i, angle[i]);
                angle[i] -= increment;
                HAL_Delay(10);
            }
        }
        HAL_Delay(1000);
    }

    // Store the target_angle in angle after movement
    for (int cnt = 0; cnt < 5; cnt++)
    {
            current_angle[cnt] = target_angle[cnt];
    }

}



