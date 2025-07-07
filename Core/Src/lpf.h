#ifndef INC_LPF_H_
#define INC_LPF_H_
#include "stdint.h"
#include "main.h"



typedef struct {
    float x1, x2;  // Previous inputs
    float y1, y2;  // Previous outputs
} LPF_State;

typedef struct {
    float b0, b1, b2;  // Numerator coefficients
    float a1, a2;       // Denominator coefficients (a0 is normalized to 1)
} LPF_Coef;


LPF_Coef calculate_lpf_coefficients(float fc, float fs, float Q);
void moving_average_filter(float *input, float *output, int length);
float lpf_process(float input, LPF_State* state, float b0, float b1, float b2, float a1, float a2);


#endif

