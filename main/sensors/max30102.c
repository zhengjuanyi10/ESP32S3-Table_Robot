/**
 * @file max30102.c
 * @brief MAX30102 血氧传感器驱动
 *
 * 修正 FIFO 读取方式——逐字节读取，避免多字节连续读取异常
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"

#include "max30102.h"
#include "i2c_bus.h"

static const char *TAG = "MAX30102";
#define I2C_ADDR 0x57

/* 寄存器地址 */
#define REG_INTR_STATUS_1  0x00
#define REG_INTR_STATUS_2  0x01
#define REG_INTR_ENABLE_1  0x02
#define REG_INTR_ENABLE_2  0x03
#define REG_FIFO_WR_PTR    0x04
#define REG_OVF_COUNTER    0x05
#define REG_FIFO_RD_PTR    0x06
#define REG_FIFO_DATA      0x07
#define REG_FIFO_CONFIG    0x08
#define REG_MODE_CONFIG    0x09
#define REG_SPO2_CONFIG    0x0A
#define REG_LED1_PA        0x0C
#define REG_LED2_PA        0x0D
#define REG_TEMP_INTEGER   0x1F
#define REG_TEMP_FRACTION  0x20
#define REG_TEMP_CONFIG    0x21
#define REG_PART_ID        0xFF

static i2c_master_dev_handle_t s_dev = NULL;

/* ========== I2C 读写 ========== */

static esp_err_t write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_transmit(s_dev, buf, 2, -1);
}

static esp_err_t read_reg(uint8_t reg, uint8_t *val)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, val, 1, -1);
}

/* ========== 初始化 ========== */

esp_err_t max30102_init(void)
{
    i2c_master_bus_handle_t bus = i2c_bus_get_handle();
    if (!bus) return ESP_FAIL;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = I2C_ADDR,
        .scl_speed_hz = 400000,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus, &dev_cfg, &s_dev),
                        TAG, "add device failed");

    uint8_t id = 0;
    read_reg(REG_PART_ID, &id);
    ESP_LOGI(TAG, "PART_ID: 0x%02X", id);

    /* 复位 */
    write_reg(REG_MODE_CONFIG, 0x40);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* FIFO: 4 平均, 滚存使能, FIFO_A_FULL=15 */
    write_reg(REG_FIFO_CONFIG, 0x4F);

    /* 模式: SpO2 */
    write_reg(REG_MODE_CONFIG, 0x03);

    /* SpO2: 411μs, 200Hz, 16384nA */
    write_reg(REG_SPO2_CONFIG, 0x47);

    /* LED 电流: RED=25mA, IR=18mA（降低 IR 防饱和） */
    write_reg(REG_LED1_PA, 0x70);  // RED: 22.4mA
    write_reg(REG_LED2_PA, 0x5A);  // IR:  18.0mA

    /* 中断使能 */
    write_reg(REG_INTR_ENABLE_1, 0x40);
    write_reg(REG_INTR_ENABLE_2, 0x00);

    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_LOGI(TAG, "MAX30102 ready");
    return ESP_OK;
}

void max30102_reset(void)
{
    write_reg(REG_MODE_CONFIG, 0x40);
    vTaskDelay(pdMS_TO_TICKS(10));
}

/* ========== 读取温度 ========== */

float max30102_read_temp(void)
{
    uint8_t ti = 0, tf = 0;
    write_reg(REG_TEMP_CONFIG, 0x01);
    vTaskDelay(pdMS_TO_TICKS(20));
    read_reg(REG_TEMP_INTEGER, &ti);
    read_reg(REG_TEMP_FRACTION, &tf);
    return (float)ti + (float)tf * 0.0625f;
}

/* ========== 读取 FIFO 采样 ========== */

esp_err_t max30102_read_sample(max30102_sample_t *sample)
{
    if (!sample) return ESP_ERR_INVALID_ARG;

    /* 先清除中断状态 */
    uint8_t dummy;
    read_reg(REG_INTR_STATUS_1, &dummy);
    read_reg(REG_INTR_STATUS_2, &dummy);

    /* SpO2 模式 FIFO 顺序: [RED_3bytes][IR_3bytes] */
    uint8_t buf[6] = {0};
    uint8_t reg = REG_FIFO_DATA;
    esp_err_t ret = i2c_master_transmit_receive(s_dev, &reg, 1, buf, 6, -1);
    if (ret != ESP_OK) return ret;

    sample->red = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];
    sample->ir  = ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8) | buf[5];

    if (sample->red > 262143) sample->red &= 0x3FFFF;
    if (sample->ir  > 262143) sample->ir  &= 0x3FFFF;

    return ESP_OK;
}

/* ========== 检查可用样本数 ========== */

int max30102_available(void)
{
    uint8_t wr = 0, rd = 0;
    read_reg(REG_FIFO_WR_PTR, &wr);
    read_reg(REG_FIFO_RD_PTR, &rd);
    return (wr - rd) & 0x1F;
}
