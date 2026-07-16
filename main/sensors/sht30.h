/*
 * 功能：声明 SHT30 初始化和单次温湿度测量接口。
 * 配置：默认地址 0x44；ADDR 拉高的 0x45 模块需同步修改驱动地址。
 */

#pragma once

#include "esp_err.h"

/* SHT30 单次测量结果：摄氏度和相对湿度百分比。 */
typedef struct {
    float temperature_c;
    float humidity_percent;
} sht30_measurement_t;

/* 注册默认地址设备，并触发一次带 CRC 校验的高精度测量。 */
esp_err_t sht30_init(void);
esp_err_t sht30_get_measurement(sht30_measurement_t *measurement);
