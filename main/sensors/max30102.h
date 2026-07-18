/**
 * @file max30102.h
 * @brief MAX30102 血氧/心率传感器驱动
 *
 * I2C 地址: 0x57 (7-bit)
 * 使用 i2c_bus 共享总线 (GPIO8/GPIO9)
 */

#pragma once

#include <stdint.h>
#include "esp_err.h"

/** MAX30102 数据 */
typedef struct {
    uint32_t ir;    /**< 红外通道原始值 */
    uint32_t red;   /**< 红光通道原始值 */
} max30102_sample_t;

/**
 * @brief 初始化 MAX30102
 * @return ESP_OK 成功
 */
esp_err_t max30102_init(void);

/**
 * @brief 读取一组 IR/RED 原始值
 * @param sample 输出采样数据
 * @return ESP_OK 成功
 */
esp_err_t max30102_read_sample(max30102_sample_t *sample);

/**
 * @brief 读取芯片温度（摄氏度）
 * @return 温度值 (如 27.5)
 */
float max30102_read_temp(void);

/**
 * @brief 重置芯片
 */
void max30102_reset(void);

/**
 * @brief 检查是否有新数据
 * @return 可用样本数
 */
int max30102_available(void);
