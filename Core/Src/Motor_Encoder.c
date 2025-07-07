#include "Motor_Encoder.h"
#include "stm32f4xx_hal.h"
#include <stdatomic.h>



void Encoder_Reset(encoder *enc) {
    enc->position = 0;
    enc->last_count = 0;
    enc->rpm = 0.0f;
    atomic_flag_clear(&enc->is_initialized);
}

void Encoder_Init(encoder *enc, TIM_HandleTypeDef *htim) {
    atomic_init(&enc->max_count, htim->Instance->ARR); // Set once here
    atomic_flag_clear(&enc->is_initialized);
}

void Encoder_Update(encoder *enc, TIM_HandleTypeDef *htim, float dt, float counts_per_rev, int8_t dir) {
    uint32_t current_count = __HAL_TIM_GET_COUNTER(htim);

    if (!atomic_flag_test_and_set(&enc->is_initialized)) {
        enc->last_count = current_count;
        return;
    }

    int32_t delta = (int32_t)(current_count - enc->last_count);
    delta *= dir;
    enc->position += delta;
    enc->last_count = current_count;
    enc->rpm = (delta * 60.0f) / (counts_per_rev * dt);
}

