#pragma once

#include "driver/i2c_master.h"

/*
 * 数据读取模块共用的 I2C0 总线。
 * SDA=GPIO8、SCL=GPIO9；DS3231、SHT30、INA219 都通过此接口获取总线句柄。
 */
esp_err_t i2c_bus_init(void);
i2c_master_bus_handle_t i2c_bus_get_handle(void);
