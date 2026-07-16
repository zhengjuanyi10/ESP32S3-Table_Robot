#include <stdint.h>

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "i2c_bus.h"
#include "sht30.h"

/* SHT30 默认 7 位地址；ADDR 接高电平时需改为 0x45。 */
#define SHT30_ADDRESS 0x44

/* 温湿度传感器在共享总线上的设备句柄。 */
static i2c_master_dev_handle_t s_sht30;

static uint8_t crc8(const uint8_t *data, size_t length)
{
    /* Sensirion CRC-8：初值 0xFF，多项式 0x31。 */
    uint8_t crc = 0xFF;
    while (length-- > 0) {
        crc ^= *data++;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : crc << 1;
        }
    }
    return crc;
}

esp_err_t sht30_init(void)
{
    /* 先确保共享总线已创建，再注册 SHT30。 */
    if (s_sht30 != NULL) {
        return ESP_OK;
    }
    const i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = SHT30_ADDRESS,
        .scl_speed_hz = 100000,
    };
    esp_err_t err = i2c_bus_init();
    return err == ESP_OK ? i2c_master_bus_add_device(i2c_bus_get_handle(), &config, &s_sht30) : err;
}

esp_err_t sht30_get_measurement(sht30_measurement_t *measurement)
{
    /* 每次调用都会触发一次独立测量，结果不依赖旧缓存。 */
    if (measurement == NULL || s_sht30 == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* 单次高精度测量，无 clock stretching；转换约需 15 ms。 */
    const uint8_t command[] = {0x2C, 0x06};
    uint8_t data[6];
    esp_err_t err = i2c_master_transmit(s_sht30, command, sizeof(command), 100);
    if (err != ESP_OK) {
        return err;
    }
    vTaskDelay(pdMS_TO_TICKS(15));
    err = i2c_master_receive(s_sht30, data, sizeof(data), 100);
    if (err != ESP_OK) {
        return err;
    }
    /* 温度和湿度各有两字节数据和一字节 CRC。 */
    if (crc8(data, 2) != data[2] || crc8(data + 3, 2) != data[5]) {
        return ESP_ERR_INVALID_CRC;
    }

    const uint16_t raw_temperature = ((uint16_t)data[0] << 8) | data[1];
    const uint16_t raw_humidity = ((uint16_t)data[3] << 8) | data[4];
    measurement->temperature_c = -45.0f + 175.0f * raw_temperature / 65535.0f;
    measurement->humidity_percent = 100.0f * raw_humidity / 65535.0f;
    return ESP_OK;
}
