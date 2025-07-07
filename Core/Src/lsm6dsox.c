/*
 * lsm6dsox.c
 *
 *  Created on: Mar 15, 2023
 *      Author: jed
 */

#include "lsm6dsox.h"

#define TIMEOUT_MS      (50)





float heading = 0.0f;
uint32_t previous_time = 0;
float gyro_bias = 0.0f; // Calibrated gyroscope bias


HAL_StatusTypeDef lsm6dsox_init(lsm6dsox_t *lsm6dsox, I2C_HandleTypeDef *i2c, uint8_t *buf, uint8_t bufsize) {
	HAL_StatusTypeDef status;

	lsm6dsox->i2c_addr_r = (LSM6DSOX_ADDR_BASE << 1) | LSM6DSOX_ADDR_READ_BIT;
	lsm6dsox->i2c_addr_w = (LSM6DSOX_ADDR_BASE << 1) | LSM6DSOX_ADDR_WRITE_BIT;
	lsm6dsox->i2c = i2c;
	lsm6dsox->buf = buf;
	lsm6dsox->bufsize = bufsize;

	// Wake up.
	HAL_Delay(LSM6DSOX_POWER_UP_MS);

	status = HAL_I2C_IsDeviceReady(lsm6dsox->i2c, LSM6DSOX_ADDR_BASE << 1, 1, TIMEOUT_MS);
	if (status != HAL_OK) return status;

	// I am who I am.
	status = lsm6dsox_read(lsm6dsox, REG_WHO_AM_I, 1);
	if (status != HAL_OK) return status;
	if (lsm6dsox->buf[0] != LSM6DSOX_DEVICE_ID) return HAL_ERROR;

	return HAL_OK;
}

bool lsm6dsox_data_available(lsm6dsox_t *lsm6dsox) {
	return false;
}

HAL_StatusTypeDef lsm6dsox_read(lsm6dsox_t *lsm6dsox, uint8_t reg, uint8_t bufsize) {
	if (bufsize > lsm6dsox->bufsize) return HAL_ERROR;

	return HAL_I2C_Mem_Read(lsm6dsox->i2c, lsm6dsox->i2c_addr_r, reg, 1, lsm6dsox->buf, bufsize, TIMEOUT_MS);
}

HAL_StatusTypeDef lsm6dsox_write(lsm6dsox_t *lsm6dsox, uint8_t reg, uint8_t data) {
	return HAL_I2C_Mem_Write(lsm6dsox->i2c, lsm6dsox->i2c_addr_w, reg, 1, &data, 1, TIMEOUT_MS);
}

//HAL_StatusTypeDef initialize(HAL_StatusTypeDef stat) {
//
//}

float getAX(uint8_t lsm6dsox_buf[6]) {
      int16_t ax_raw = (int16_t)(lsm6dsox_buf[1] << 8 | lsm6dsox_buf[0]);
      float ax_g = ax_raw / 16384.0f; // ±2g range
      float ax = ax_g * 9.80665f;  // X-axis acceleration in m/s²

      //+- 0.2m/s^2 offset

      return ax;
}

float getAY(uint8_t lsm6dsox_buf[6]){
      int16_t ay_raw = (int16_t)(lsm6dsox_buf[3] << 8 | lsm6dsox_buf[2]);
      float ay_g = ay_raw / 16384.0f;
      float ay = ay_g * 9.80665f;  // Y-axis (fixed from original z_ms2)

      //+- 0.2m/s^2 offset

      return ay;

}

float getAZ(uint8_t lsm6dsox_buf[6]) {
      int16_t az_raw = (int16_t)(lsm6dsox_buf[5] << 8 | lsm6dsox_buf[4]);
      float az_g = az_raw / 16384.0f;
      float az = az_g* 9.80665f;  // Z-axis

      //+- 0.2m/s^2 offset
      return az - 0.2;

}

float getGX_dps(uint8_t lsm6dsox_buf[6]) {
      int16_t gx_raw = (int16_t)(lsm6dsox_buf[1] << 8 | lsm6dsox_buf[0]);
      float gx_dps = gx_raw * 0.00875f; // ±250dps range

      return gx_dps;

}

float getGY_dps(uint8_t lsm6dsox_buf[6]) {
      int16_t gy_raw = (int16_t)(lsm6dsox_buf[3] << 8 | lsm6dsox_buf[2]);
      float gy_dps = gy_raw * 0.00875f; // ±250dps range

      return gy_dps;

}

float getGZ_dps(uint8_t lsm6dsox_buf[6]) {
      int16_t gz_raw = (int16_t)(lsm6dsox_buf[5] << 8 | lsm6dsox_buf[4]);
      float gz_dps = gz_raw * 0.00875f; // ±250dps range

      return gz_dps + 2.2178*0.0841;

}

float wrap_angle(float angle) {
    angle = fmod(angle, 360.0f);
    if (angle > 180.0f) angle -= 360.0f;
    else if (angle < -180.0f) angle += 360.0f;
    return angle;
}

//void updateHeading(float)

//void updateHeading(void) {
//    uint32_t current_time = HAL_GetTick(); // Get current time in milliseconds
//    float dt = (current_time - previous_time) / 1000.0f; // Convert time difference to seconds
//
//    float gz_dps = getGZ_dps() - gyro_bias; // Get gyroscope reading and remove bias
//
//    // Calculate change in heading
//    float delta_heading = gz_dps * dt;
//
//    // Update heading
//    heading += delta_heading;
//
//    // Normalize heading to 0-360 degrees
//    heading = fmodf(heading, 360.0f);
//    if (heading < 0) heading += 360.0f;
//
//    // Update previous time for next calculation
//    previous_time = current_time;
//}

