#include "pid.h"
#include <math.h>
#include <stdlib.h>

void PID_Init(PID *pid, float Kp, float Ki, float Kd, float setpoint) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->setpoint = setpoint;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = -100.0f;  // PWM limits
    pid->output_max = 100.0f;
}

void PID_Update(PID *pid, float setpoint, float measured_value, float dt) {
    if (dt < 1e-6f) dt = 1e-6f;
    float error = setpoint - measured_value;

    pid->integral += error * dt;
    float derivative = (error - pid->prev_error) / dt;

    pid->output =
        pid->Kp * error +
        pid->Ki * pid->integral +
        pid->Kd * derivative;

    if (pid->output > pid->output_max) pid->output = pid->output_max;
    else if (pid->output < pid->output_min) pid->output = pid->output_min;

    pid->prev_error = error;
    pid->setpoint = setpoint;
}

void Motor_SetPWM(TIM_HandleTypeDef *htim, uint32_t channel, float duty_percent)
{
	float duty_abs = fabsf(duty_percent);

	uint32_t compare_val = (uint32_t)((duty_abs / 100.0f) * htim->Init.Period);
	__HAL_TIM_SET_COMPARE(htim, channel, compare_val);
}



/**
  * @brief  Drive a motor with two opposite PWM signals.
  *         pid_output in [-100..100]. Positive => forward, negative => reverse.
  */
void Motor_SetMotorPWM(TIM_HandleTypeDef *htim,
                       uint32_t forward_channel,
                       uint32_t reverse_channel,
                       float pid_output)
{
    if (pid_output > 0.0f) {
        Motor_SetPWM(htim, forward_channel, pid_output);
        Motor_SetPWM(htim, reverse_channel, 0.0f);
    }
    else if (pid_output < 0.0f) {
        Motor_SetPWM(htim, forward_channel, 0.0f);
        Motor_SetPWM(htim, reverse_channel, pid_output);
    }
    else {
        Motor_SetPWM(htim, forward_channel, 0.0f);
        Motor_SetPWM(htim, reverse_channel, 0.0f);
    }
}



