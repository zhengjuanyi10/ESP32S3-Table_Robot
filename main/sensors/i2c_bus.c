#include "i2c_bus.h"

/* 总线句柄只创建一次，避免多个 I2C 设备重复初始化 I2C0。 */
static i2c_master_bus_handle_t s_bus;

esp_err_t i2c_bus_init(void)
{
    /* 已创建时直接复用，允许每个传感器模块独立调用初始化。 */
    if (s_bus != NULL) {
        return ESP_OK;
    }

    /* 使用内部上拉仅方便调试；正式接线建议模块自带或外接 4.7k 上拉。 */
    const i2c_master_bus_config_t config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = 8,
        .scl_io_num = 9,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&config, &s_bus);
}

i2c_master_bus_handle_t i2c_bus_get_handle(void)
{
    /* 调用者应先检查 i2c_bus_init() 返回 ESP_OK。 */
    return s_bus;
}
