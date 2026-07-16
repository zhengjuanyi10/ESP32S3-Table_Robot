/* INA219 电压、电流和功率监测驱动。 */

#include <stdint.h>

#include "driver/i2c_master.h"

#include "i2c_bus.h"
#include "ina219.h"

/* INA219 默认 7 位地址；A0/A1 焊盘会改变该地址。 */
#define INA219_ADDRESS 0x40
#define INA219_REG_CONFIG 0x00
#define INA219_REG_BUS_VOLTAGE 0x02
#define INA219_REG_POWER 0x03
#define INA219_REG_CURRENT 0x04
#define INA219_REG_CALIBRATION 0x05

/* 常见 INA219 R100 模块：0.1 欧分流电阻，电流分辨率 0.1 mA。 */
#define INA219_CALIBRATION 4096
#define INA219_CURRENT_LSB_A 0.0001f
#define INA219_POWER_LSB_W (20.0f * INA219_CURRENT_LSB_A)

/* 电流监测芯片在共享总线上的设备句柄。 */
static i2c_master_dev_handle_t s_ina219;

static esp_err_t write_register(uint8_t reg, uint16_t value)
{
    /* INA219 寄存器和数据均按高字节在前发送。 */
    const uint8_t data[] = {reg, (uint8_t)(value >> 8), (uint8_t)value};
    return i2c_master_transmit(s_ina219, data, sizeof(data), 100);
}

static esp_err_t read_register(uint8_t reg, uint16_t *value)
{
    /* 先写寄存器地址，再重复起始读取两个字节。 */
    uint8_t data[2];
    esp_err_t err = i2c_master_transmit_receive(s_ina219, &reg, 1, data, sizeof(data), 100);
    if (err == ESP_OK) {
        *value = ((uint16_t)data[0] << 8) | data[1];
    }
    return err;
}

esp_err_t ina219_init(void)
{
    /* 注册设备后写入测量模式和 R100 校准值。 */
    if (s_ina219 != NULL) {
        return ESP_OK;
    }

    const i2c_device_config_t config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = INA219_ADDRESS,
        .scl_speed_hz = 100000,
    };
    esp_err_t err = i2c_bus_init();
    if (err != ESP_OK) {
        return err;
    }
    err = i2c_master_bus_add_device(i2c_bus_get_handle(), &config, &s_ina219);
    if (err != ESP_OK) {
        return err;
    }

    /* 32 V / 320 mV 量程、连续测量；随后写入 R100 的校准值。 */
    err = write_register(INA219_REG_CONFIG, 0x399F);
    return err == ESP_OK ? write_register(INA219_REG_CALIBRATION, INA219_CALIBRATION) : err;
}

esp_err_t ina219_get_measurement(ina219_measurement_t *measurement)
{
    /* 依次读取电压、电流、功率，任一步失败就舍弃本次结果。 */
    if (measurement == NULL || s_ina219 == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    uint16_t bus_raw;
    uint16_t current_raw;
    uint16_t power_raw;
    esp_err_t err = read_register(INA219_REG_BUS_VOLTAGE, &bus_raw);
    if (err == ESP_OK) {
        err = read_register(INA219_REG_CURRENT, &current_raw);
    }
    if (err == ESP_OK) {
        err = read_register(INA219_REG_POWER, &power_raw);
    }
    if (err != ESP_OK) {
        return err;
    }

    measurement->bus_voltage_v = ((bus_raw >> 3) * 0.004f);
    measurement->current_a = (int16_t)current_raw * INA219_CURRENT_LSB_A;
    measurement->power_w = power_raw * INA219_POWER_LSB_W;
    return ESP_OK;
}
