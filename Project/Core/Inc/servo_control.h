#include "stm32f1xx_hal.h"

#ifndef INC_SERVO_CONTROL_H_
#define INC_SERVO_CONTROL_H_

// Function prototypes
void MoveServoToAngle(float* angle, float* target_angle, float increment, float* current_angle);
void ResetServoIndex(uint8_t* index, uint8_t max_index);


#endif /* INC_SERVO_CONTROL_H_ */
