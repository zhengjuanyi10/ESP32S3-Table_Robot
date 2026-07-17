/*
 * Capacitive touch sampling through TCA9548A channel 1.
 *
 * Supported bring-up protocols:
 * - FT6x36 / FT6236 / FT6336 at 0x38
 * - GT911 at 0x5D or 0x14
 * - CST816-style register map at 0x15
 *
 * Ported from the power_station project and adapted for the table_robot
 * board (GPIO12 RST, GPIO13 INT, TCA channel 1, portrait 320x480).
 */

#include "touch_chip.h"

#include "sdkconfig.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensors/i2c_bus.h"
#include "sensors/i2c_mux.h"

static const char *TAG = "touch_chip";

/* ------------------------------------------------------------------ */
/* Internal constants                                                  */
/* ------------------------------------------------------------------ */

#define TOUCH_INIT_RETRY_MS  1000
#define TOUCH_I2C_TIMEOUT_MS 25
#define TOUCH_RESET_FAIL_COUNT 5
#define GT911_STATUS_REG      0x814E
#define GT911_POINT1_REG      0x8150

/* Portrait screen dimensions — must match the display driver. */
#define TOUCH_SCREEN_WIDTH  320
#define TOUCH_SCREEN_HEIGHT 480

/* ------------------------------------------------------------------ */
/* Static state                                                        */
/* ------------------------------------------------------------------ */

static i2c_master_dev_handle_t s_touch_dev;
static touch_controller_t     s_controller = TOUCH_CONTROLLER_NONE;
static uint8_t                s_touch_addr;
static bool                   s_touch_initialized;
static esp_err_t              s_touch_init_err = ESP_ERR_INVALID_STATE;
static TickType_t             s_last_touch_init_fail_tick;
static uint8_t                s_touch_read_fail_count;

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static const char *touch_controller_name(void)
{
    switch (s_controller) {
    case TOUCH_CONTROLLER_FT6X36: return "FT6X36";
    case TOUCH_CONTROLLER_GT911:  return "GT911";
    case TOUCH_CONTROLLER_CST816: return "CST816";
    default:                      return "NONE";
    }
}

/* ------------------------------------------------------------------ */
/* Coordinate transform                                                */
/* ------------------------------------------------------------------ */

static void touch_transform(uint16_t raw_x, uint16_t raw_y,
                            uint16_t *screen_x, uint16_t *screen_y)
{
    int x = raw_x;
    int y = raw_y;

#if CONFIG_TABLE_ROBOT_TOUCH_SWAP_XY
    int tmp = x;
    x = y;
    y = tmp;
#endif
#if CONFIG_TABLE_ROBOT_TOUCH_INVERT_X
    x = TOUCH_SCREEN_WIDTH - 1 - x;
#endif
#if CONFIG_TABLE_ROBOT_TOUCH_INVERT_Y
    y = TOUCH_SCREEN_HEIGHT - 1 - y;
#endif

    if (x < 0) {
        x = 0;
    } else if (x >= TOUCH_SCREEN_WIDTH) {
        x = TOUCH_SCREEN_WIDTH - 1;
    }
    if (y < 0) {
        y = 0;
    } else if (y >= TOUCH_SCREEN_HEIGHT) {
        y = TOUCH_SCREEN_HEIGHT - 1;
    }

    *screen_x = (uint16_t)x;
    *screen_y = (uint16_t)y;
}

/* ------------------------------------------------------------------ */
/* I2C primitives                                                      */
/* ------------------------------------------------------------------ */

static esp_err_t touch_select_channel(void)
{
#if CONFIG_TABLE_ROBOT_TOUCH_BYPASS_TCA
    return ESP_OK;
#else
    return i2c_mux_select_channel((uint8_t)CONFIG_TABLE_ROBOT_TOUCH_TCA_CHANNEL);
#endif
}

static esp_err_t touch_read_reg8(uint8_t reg, uint8_t *data, size_t len)
{
    ESP_RETURN_ON_ERROR(touch_select_channel(), TAG, "select touch channel failed");
    return i2c_master_transmit_receive(s_touch_dev, &reg, 1, data, len,
                                       TOUCH_I2C_TIMEOUT_MS);
}

static esp_err_t touch_read_reg16(uint16_t reg, uint8_t *data, size_t len)
{
    uint8_t tx[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF) };
    ESP_RETURN_ON_ERROR(touch_select_channel(), TAG, "select touch channel failed");
    return i2c_master_transmit_receive(s_touch_dev, tx, sizeof(tx), data, len,
                                       TOUCH_I2C_TIMEOUT_MS);
}

static esp_err_t touch_write_reg16_u8(uint16_t reg, uint8_t value)
{
    uint8_t tx[3] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFF), value };
    ESP_RETURN_ON_ERROR(touch_select_channel(), TAG, "select touch channel failed");
    return i2c_master_transmit(s_touch_dev, tx, sizeof(tx), TOUCH_I2C_TIMEOUT_MS);
}

/* ------------------------------------------------------------------ */
/* State management                                                    */
/* ------------------------------------------------------------------ */

static void touch_reset_state(void)
{
    if (s_touch_dev != NULL) {
        esp_err_t err = i2c_master_bus_rm_device(s_touch_dev);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "remove stale touch device failed: %s", esp_err_to_name(err));
        }
        s_touch_dev = NULL;
    }
    s_controller       = TOUCH_CONTROLLER_NONE;
    s_touch_addr       = 0;
    s_touch_initialized = false;
    s_touch_init_err   = ESP_ERR_INVALID_STATE;
    s_last_touch_init_fail_tick = 0;
    s_touch_read_fail_count     = 0;
}

static esp_err_t touch_mark_init_failed(esp_err_t err)
{
    s_touch_init_err = err;
    s_touch_initialized = false;
    s_last_touch_init_fail_tick = xTaskGetTickCount();
    return err;
}

/* ------------------------------------------------------------------ */
/* Device registration                                                 */
/* ------------------------------------------------------------------ */

static esp_err_t touch_add_device(uint8_t addr)
{
    if (s_touch_dev != NULL) {
        esp_err_t err = i2c_master_bus_rm_device(s_touch_dev);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "remove previous touch device failed: %s", esp_err_to_name(err));
        }
        s_touch_dev = NULL;
    }

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = CONFIG_TABLE_ROBOT_TOUCH_I2C_FREQ_HZ,
    };
    ESP_RETURN_ON_ERROR(
        i2c_master_bus_add_device(i2c_mux_get_bus(), &dev_config, &s_touch_dev),
        TAG, "touch add device 0x%02X failed", addr);
    s_touch_addr = addr;
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* GPIO reset                                                          */
/* ------------------------------------------------------------------ */

static void touch_gpio_init_and_reset(void)
{
    gpio_config_t rst_conf = {
        .pin_bit_mask = 1ULL << CONFIG_TABLE_ROBOT_TOUCH_RESET_GPIO,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&rst_conf);

    gpio_config_t int_conf = {
        .pin_bit_mask = 1ULL << CONFIG_TABLE_ROBOT_TOUCH_INT_GPIO,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&int_conf);

    gpio_set_level(CONFIG_TABLE_ROBOT_TOUCH_RESET_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(CONFIG_TABLE_ROBOT_TOUCH_RESET_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(150));
}

/* ------------------------------------------------------------------ */
/* Auto-detection                                                      */
/* ------------------------------------------------------------------ */

static esp_err_t touch_probe_addr(uint8_t addr)
{
#if CONFIG_TABLE_ROBOT_TOUCH_BYPASS_TCA
    return i2c_master_probe(i2c_bus_get_handle(), addr, 50);
#else
    ESP_RETURN_ON_ERROR(touch_select_channel(), TAG, "select touch channel failed");
    return i2c_mux_probe(addr, 50);
#endif
}

static esp_err_t touch_detect_one(uint8_t addr, touch_controller_t controller)
{
    esp_err_t err = touch_probe_addr(addr);
    if (err != ESP_OK) {
        return err;
    }

    ESP_RETURN_ON_ERROR(touch_add_device(addr), TAG, "attach touch device failed");
    s_controller = controller;
    ESP_LOGI(TAG, "Capacitive touch detected: %s addr=0x%02X TCA_CH=%u INT=%d RST=%d",
             touch_controller_name(), addr,
             (unsigned)CONFIG_TABLE_ROBOT_TOUCH_TCA_CHANNEL,
             CONFIG_TABLE_ROBOT_TOUCH_INT_GPIO,
             CONFIG_TABLE_ROBOT_TOUCH_RESET_GPIO);
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Init (idempotent, with retry back-off)                              */
/* ------------------------------------------------------------------ */

static esp_err_t touch_init(void)
{
    if (s_touch_initialized) {
        return ESP_OK;
    }
    if (s_last_touch_init_fail_tick != 0) {
        TickType_t now = xTaskGetTickCount();
        if (now - s_last_touch_init_fail_tick < pdMS_TO_TICKS(TOUCH_INIT_RETRY_MS)) {
            return s_touch_init_err;
        }
    }

    touch_gpio_init_and_reset();

    esp_err_t err;
#if CONFIG_TABLE_ROBOT_TOUCH_BYPASS_TCA
    /* Touch is directly on the I2C bus — just make sure the bus exists. */
    err = i2c_bus_init();
#else
    err = i2c_mux_init();
#endif
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Touch I2C init failed: %s", esp_err_to_name(err));
        return touch_mark_init_failed(err);
    }

    err = touch_select_channel();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Touch channel select failed: %s", esp_err_to_name(err));
        return touch_mark_init_failed(err);
    }

    /* Probe known capacitive-touch addresses in priority order. */
    if (touch_detect_one(CONFIG_TABLE_ROBOT_TOUCH_FT6X36_ADDR, TOUCH_CONTROLLER_FT6X36) == ESP_OK ||
        touch_detect_one(CONFIG_TABLE_ROBOT_TOUCH_GT911_ADDR_PRIMARY, TOUCH_CONTROLLER_GT911) == ESP_OK ||
        touch_detect_one(CONFIG_TABLE_ROBOT_TOUCH_GT911_ADDR_SECONDARY, TOUCH_CONTROLLER_GT911) == ESP_OK ||
        touch_detect_one(CONFIG_TABLE_ROBOT_TOUCH_CST816_ADDR, TOUCH_CONTROLLER_CST816) == ESP_OK) {
        s_touch_init_err = ESP_OK;
        s_touch_initialized = true;
        s_last_touch_init_fail_tick = 0;
        s_touch_read_fail_count = 0;
        return ESP_OK;
    }

    s_controller = TOUCH_CONTROLLER_NONE;
    ESP_LOGW(TAG, "No capacitive touch controller found on TCA channel %u; "
             "tried 0x%02X/0x%02X/0x%02X/0x%02X",
             (unsigned)CONFIG_TABLE_ROBOT_TOUCH_TCA_CHANNEL,
             (unsigned)CONFIG_TABLE_ROBOT_TOUCH_FT6X36_ADDR,
             (unsigned)CONFIG_TABLE_ROBOT_TOUCH_GT911_ADDR_PRIMARY,
             (unsigned)CONFIG_TABLE_ROBOT_TOUCH_GT911_ADDR_SECONDARY,
             (unsigned)CONFIG_TABLE_ROBOT_TOUCH_CST816_ADDR);
    return touch_mark_init_failed(ESP_ERR_NOT_FOUND);
}

/* ------------------------------------------------------------------ */
/* Protocol readers                                                    */
/* ------------------------------------------------------------------ */

static esp_err_t touch_read_ft_like(touch_sample_t *sample)
{
    uint8_t status = 0;
    ESP_RETURN_ON_ERROR(touch_read_reg8(0x02, &status, 1), TAG, "touch status read failed");

    uint8_t points = status & 0x0F;
    if (points == 0) {
        sample->pressed = false;
        return ESP_OK;
    }

    uint8_t data[4] = { 0 };
    ESP_RETURN_ON_ERROR(touch_read_reg8(0x03, data, sizeof(data)), TAG, "touch point read failed");

    uint16_t raw_x = (uint16_t)(((data[0] & 0x0F) << 8) | data[1]);
    uint16_t raw_y = (uint16_t)(((data[2] & 0x0F) << 8) | data[3]);
    touch_transform(raw_x, raw_y, &sample->raw_x, &sample->raw_y);
    sample->pressed  = true;
    sample->pressure = points;
    sample->x_rx[0]  = data[0];
    sample->x_rx[1]  = data[1];
    sample->y_rx[0]  = data[2];
    sample->y_rx[1]  = data[3];
    return ESP_OK;
}

static esp_err_t touch_read_gt911(touch_sample_t *sample)
{
    uint8_t status = 0;
    ESP_RETURN_ON_ERROR(touch_read_reg16(GT911_STATUS_REG, &status, 1),
                        TAG, "GT911 status read failed");

    uint8_t points = status & 0x0F;
    if ((status & 0x80) == 0 || points == 0) {
        sample->pressed = false;
        return ESP_OK;
    }

    uint8_t data[8] = { 0 };
    esp_err_t err = touch_read_reg16(GT911_POINT1_REG, data, sizeof(data));
    (void)touch_write_reg16_u8(GT911_STATUS_REG, 0x00);
    ESP_RETURN_ON_ERROR(err, TAG, "GT911 point read failed");

    uint16_t raw_x = (uint16_t)data[1] << 8 | data[0];
    uint16_t raw_y = (uint16_t)data[3] << 8 | data[2];
    touch_transform(raw_x, raw_y, &sample->raw_x, &sample->raw_y);
    sample->pressed  = true;
    sample->pressure = points;
    sample->x_rx[0]  = data[0];
    sample->x_rx[1]  = data[1];
    sample->y_rx[0]  = data[2];
    sample->y_rx[1]  = data[3];
    return ESP_OK;
}

/* ------------------------------------------------------------------ */
/* Public API                                                          */
/* ------------------------------------------------------------------ */

void touch_chip_boot_probe(void)
{
    ESP_LOGI(TAG, "Capacitive touch configured: SDA/SCL via TCA CH%u, INT=%d, RST=%d",
             (unsigned)CONFIG_TABLE_ROBOT_TOUCH_TCA_CHANNEL,
             CONFIG_TABLE_ROBOT_TOUCH_INT_GPIO,
             CONFIG_TABLE_ROBOT_TOUCH_RESET_GPIO);
}

esp_err_t touch_chip_read_sample(touch_sample_t *sample)
{
    memset(sample, 0, sizeof(*sample));
    sample->controller = s_controller;

    esp_err_t err = touch_init();
    if (err != ESP_OK) {
        sample->last_error = err;
        sample->online = false;
        return err;
    }

    sample->online   = true;
    sample->irq_level = gpio_get_level(CONFIG_TABLE_ROBOT_TOUCH_INT_GPIO);
    sample->i2c_addr  = s_touch_addr;

    switch (s_controller) {
    case TOUCH_CONTROLLER_FT6X36:
    case TOUCH_CONTROLLER_CST816:
        err = touch_read_ft_like(sample);
        break;
    case TOUCH_CONTROLLER_GT911:
        err = touch_read_gt911(sample);
        break;
    default:
        err = ESP_ERR_NOT_FOUND;
        break;
    }

    sample->last_error = err;
    if (err != ESP_OK) {
        sample->online  = false;
        sample->pressed = false;
        if (s_touch_read_fail_count < UINT8_MAX) {
            s_touch_read_fail_count++;
        }
        ESP_LOGD(TAG, "Touch read failed %u/%u: %s",
                 s_touch_read_fail_count, TOUCH_RESET_FAIL_COUNT,
                 esp_err_to_name(err));
        if (s_touch_read_fail_count >= TOUCH_RESET_FAIL_COUNT) {
            ESP_LOGW(TAG, "Touch read failed repeatedly, re-detect on next retry");
            touch_reset_state();
        }
    } else {
        s_touch_read_fail_count = 0;
    }
    return err;
}
