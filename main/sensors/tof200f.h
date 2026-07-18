/**
 * @file tof200f.h
 * @brief TOF200F 激光测距模块 UART 驱动
 *
 * 引脚: TX→GPIO42 (ESP RX), RX→GPIO21 (ESP TX), 115200bps
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"

/**
 * @brief 初始化 TOF200F (UART1, TX=21, RX=42, 115200)
 */
esp_err_t tof200f_init(void);

/**
 * @brief 测量一次距离
 * @param dist_mm 输出距离（毫米）
 * @return ESP_OK 成功
 */
esp_err_t tof200f_measure(uint16_t *dist_mm);
