#include "pid.h"

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd, float setpoint, float dt)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = setpoint;
    pid->prev_error = 0;
    pid->integral = 0;
    pid->dt = dt;
    pid->output = 0;
}

float PID_Compute(PID_Controller *pid, float current_value)
{
	// Calculate error
    float error = pid->setpoint - current_value;

    // I_error
    pid->integral += ((error + pid->prev_error) / 2) * pid->dt;

    // D_error
    float derivative = (error - pid->prev_error) / pid->dt;

    // Output
    pid->output = pid->Kp * error + pid->Ki * pid->integral + pid->Kd * derivative;

    //Check range of output
//    if(pid->output < 102.375)
//    	pid->output = 102.375;
//    if(pid->output > 511.875)
//    	pid->output = 511.875;

    // Update prev_error
    pid->prev_error = error;

    return pid->output;
}


