#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include <math.h>
#include "lpf.h"

typedef struct {
    float Kp, Ki, Kd;
    float setpoint;
    float integral;
    float prev_error;
    float output;
    float output_min, output_max;
    LPF_State lpf_state;
    LPF_Coef  lpf_coeffs;      // computed output (e.g. -100..100)

} PID;

/* PID function prototypes */
void PID_Init  (PID *pid, float Kp, float Ki, float Kd, float setpoint);
void PID_Update(PID *pid, float setpoint, float measured_value, float dt);

void Motor_SetPWM      (TIM_HandleTypeDef *htim, uint32_t channel, float duty_percent);
void Motor_SetMotorPWM (TIM_HandleTypeDef *htim,
                        uint32_t forward_channel,
                        uint32_t reverse_channel,
                        float pid_output);

#ifdef __cplusplus
}
#endif

#endif
