
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/i2c.h"
#include "driver/gpio.h"

typedef struct {
    int16_t raw_acce_x;
    int16_t raw_acce_y;
    int16_t raw_acce_z;
} bmi2_raw_acce_value_t;

typedef struct {
    int16_t raw_gyro_x;
    int16_t raw_gyro_y;
    int16_t raw_gyro_z;
} bmi2_raw_gyro_value_t;

typedef struct {
    float acce_x;
    float acce_y;
    float acce_z;
} bmi2_acce_value_t;

typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
} bmi2_gyro_value_t;

typedef struct {
    float temp;
} bmi2_temp_value_t;


typedef void *bmi2_handle_t;

bmi2_handle_t bmi2_create(i2c_port_t port, const uint16_t sensor_addr);

esp_err_t bmi2_begin(bmi2_handle_t sensor);

esp_err_t bmi2_get_deviceid(bmi2_handle_t sensor, uint8_t *const deviceid);

void bmi2_delete(bmi2_handle_t sensor);

esp_err_t bmi2_get_raw_acce(bmi2_handle_t sensor, bmi2_raw_acce_value_t *const raw_acce_value);
/**
 * @brief Read raw accelerometer measurements
 *
 * @param sensor object handle of bmi2
 * @param acce_value accelerometer measurements
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Fail
 */

esp_err_t bmi2_get_acce(bmi2_handle_t sensor, bmi2_acce_value_t *const acce_value);

float bmi2_get_inclination(float acce_value);

bool averaging_acce(bmi2_acce_value_t *acce_value, int size, float *return_avg);
#ifdef __cplusplus
}
#endif
