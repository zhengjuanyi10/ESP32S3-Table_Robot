#pragma once

#include "esp_err.h"

/* INA219 读取到的母线电压、电流和功率。 */
typedef struct {
    float bus_voltage_v;
    float current_a;
    float power_w;
} ina219_measurement_t;

/* INA219 默认地址 0x40；配置适用于常见 R100（0.1 欧）模块。 */
esp_err_t ina219_init(void);

/* 读取母线电压、电流和功率。 */
esp_err_t ina219_get_measurement(ina219_measurement_t *measurement);
