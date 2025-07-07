#ifndef ENCODER_H
#define ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>
#include <stdatomic.h>


/* Simple encoder data structure */
typedef struct  {
    int32_t position;
    uint32_t last_count;
    float rpm;
    atomic_uint max_count;      // Use atomic type
    atomic_flag is_initialized; // Simpler for flags
} encoder;

/* Encoder function prototypes */
void    Encoder_Reset    (encoder *enc);
void    Encoder_Update   (encoder *enc, TIM_HandleTypeDef *htim, float dt, float counts_per_rev,int8_t dir);
uint8_t Encoder_GetDirection(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif

