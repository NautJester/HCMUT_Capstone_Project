#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float setpoint;
    float prev_error;
    float integral;
    float output;
    float dt;

} PID_Controller;

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float setpoint, float dt);
float PID_Compute(PID_Controller *pid, float current_value);


#endif /* INC_PID_H_ */
