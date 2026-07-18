/**
 * @file tof200f.c
 * @brief TOF200F 激光测距 MODBUS-RTU 驱动
 *
 * 读取命令: 01 03 00 10 00 01 CRC16
 * 响应:     01 03 02 [dist_H] [dist_L] CRC16
 * 距离:     (dist_H << 8 | dist_L) mm
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/uart.h"

#include "tof200f.h"

static const char *TAG = "TOF200F";
#define UART_NUM UART_NUM_1
#define TX_PIN   21
#define RX_PIN   42
#define BUF_SIZE 256

/* MODBUS CRC16 */
static uint16_t modbus_crc(const uint8_t *data, int len)
{
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

/* 测距命令 (读取寄存器 0x0010) */
static uint8_t READ_CMD[8] = {0x01, 0x03, 0x00, 0x10, 0x00, 0x01};

esp_err_t tof200f_init(void)
{
    uart_config_t cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_RETURN_ON_ERROR(uart_param_config(UART_NUM, &cfg), TAG, "param");
    ESP_RETURN_ON_ERROR(uart_set_pin(UART_NUM, TX_PIN, RX_PIN, -1, -1), TAG, "pin");
    ESP_RETURN_ON_ERROR(uart_driver_install(UART_NUM, BUF_SIZE, 0, 0, NULL, 0), TAG, "install");

    /* 预计算 CRC */
    uint16_t crc = modbus_crc(READ_CMD, 6);
    READ_CMD[6] = crc & 0xFF;
    READ_CMD[7] = (crc >> 8) & 0xFF;

    ESP_LOGI(TAG, "Init OK (TX:21 RX:42 115200 MODBUS)");
    return ESP_OK;
}

esp_err_t tof200f_measure(uint16_t *dist_mm)
{
    if (!dist_mm) return ESP_ERR_INVALID_ARG;
    *dist_mm = 0;

    uart_flush(UART_NUM);
    uart_write_bytes(UART_NUM, READ_CMD, 8);
    vTaskDelay(pdMS_TO_TICKS(50));

    uint8_t buf[16];
    int len = uart_read_bytes(UART_NUM, buf, sizeof(buf), pdMS_TO_TICKS(100));

    if (len < 5) return ESP_ERR_TIMEOUT;

    /* MODBUS 响应: [addr:1] [func:1] [len:1] [data:2] [crc:2] */
    if (buf[0] != 0x01 || buf[1] != 0x03) {
        ESP_LOGW(TAG, "bad header: %02X %02X", buf[0], buf[1]);
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (buf[2] != 2) return ESP_ERR_INVALID_RESPONSE;  // 应返回 2 字节

    *dist_mm = ((uint16_t)buf[3] << 8) | buf[4];

    /* 可选: 验证 CRC */
    uint16_t calc = modbus_crc(buf, len - 2);
    uint16_t recv = (uint16_t)buf[len-2] | ((uint16_t)buf[len-1] << 8);
    if (calc != recv) ESP_LOGW(TAG, "CRC mismatch");

    return ESP_OK;
}
