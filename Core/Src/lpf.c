#include "lpf.h"
#include <math.h>

LPF_Coef calculate_lpf_coefficients(float fc, float fs, float Q){

    LPF_Coef coef;

    // Pre-warping for bilinear transform
    float omega0 = 2 * M_PI * fc / fs;  // Digital cutoff frequency
    float alpha = sin(omega0) / (2 * Q);

    // Compute raw coefficients
    float b0_raw = (1 - cos(omega0)) / 2;
    float b1_raw = 1 - cos(omega0);
    float b2_raw = (1 - cos(omega0)) / 2;
    float a0 = 1 + alpha;
    float a1_raw = -2 * cos(omega0);
    float a2_raw = 1 - alpha;

    // Normalize coefficients by a0
    coef.b0 = b0_raw / a0;
    coef.b1 = b1_raw / a0;
    coef.b2 = b2_raw / a0;
    coef.a1 = a1_raw / a0;
    coef.a2 = a2_raw / a0;

    return coef;
}


//float moving_average_filter(float *input, int length) {
//    for (int i = 0; i < length; i++) {
//        float sum = 0;
//        int count = 0;
//
//        // Compute the average over the window
//        for (int j = i - (WINDOW_SIZE / 2); j <= i + (WINDOW_SIZE / 2); j++) {
//            if (j >= 0 && j < length) { // Ensure index is within bounds
//                sum += input[j];
//                count++;
//            }
//        }
//        return output[i] = sum / count;  // Compute the moving average
//    }
//}
//


float lpf_process(float input, LPF_State* state, float b0, float b1, float b2, float a1, float a2){
    float output =
        b0 * input +
        b1 * state->x1 +
        b2 * state->x2 -
        a1 * state->y1 -
        a2 * state->y2;

    state->x2 = state->x1;   // x[n-2] = x[n-1]
    state->x1 = input;       // x[n-1] = x[n]
    state->y2 = state->y1;   // y[n-2] = y[n-1]
    state->y1 = output;      // y[n-1] = y[n]

    return output;
}

