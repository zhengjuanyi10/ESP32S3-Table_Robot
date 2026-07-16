/* TOF200F UART distance sensor driver. */

#include "tof200.h"

#include <ctype.h>
#include <stdbool.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TOF200_READ_SLICE_MS 20
#define TOF200_MAX_DISTANCE_MM 2000
#define TFMINI_FRAME_LEN 9
#define MODBUS_MIN_FRAME_LEN 7

static const char *TAG = "tof200";

static tof200_config_t s_config = {
    .uart_num = TOF200_DEFAULT_UART_NUM,
    .baud_rate = TOF200_DEFAULT_BAUD_RATE,
    .rx_gpio = TOF200_DEFAULT_RX_GPIO,
    .tx_gpio = TOF200_DEFAULT_TX_GPIO,
    .rx_buffer_size = TOF200_DEFAULT_RX_BUFFER_SIZE,
};
static bool s_initialized;

static uint8_t checksum8(const uint8_t *data, size_t length)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < length; ++i) {
        sum += data[i];
    }
    return sum;
}

static uint16_t crc16_modbus(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if ((crc & 0x0001) != 0) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

static void copy_raw(tof200_measurement_t *measurement, const uint8_t *data, size_t length)
{
    if (length > sizeof(measurement->raw)) {
        length = sizeof(measurement->raw);
    }
    memcpy(measurement->raw, data, length);
    measurement->raw_length = length;
}

static esp_err_t parse_ascii_mm(const uint8_t *data, size_t length,
                                tof200_measurement_t *measurement)
{
    char digits[6] = {0};
    size_t count = 0;
    bool terminated = false;

    for (size_t i = 0; i < length; ++i) {
        if (isdigit((int)data[i])) {
            if (count >= sizeof(digits) - 1) {
                return ESP_ERR_INVALID_RESPONSE;
            }
            digits[count++] = (char)data[i];
        } else if (data[i] == '\r' || data[i] == '\n' || data[i] == ' ' || data[i] == '\t') {
            if (count > 0) {
                terminated = true;
                break;
            }
        } else {
            return ESP_ERR_INVALID_RESPONSE;
        }
    }

    if (count == 0 || !terminated) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint32_t value = 0;
    for (size_t i = 0; i < count; ++i) {
        value = value * 10 + (uint32_t)(digits[i] - '0');
    }
    if (value > TOF200_MAX_DISTANCE_MM) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    memset(measurement, 0, sizeof(*measurement));
    measurement->distance_mm = (uint16_t)value;
    measurement->frame_type = TOF200_FRAME_ASCII_MM;
    copy_raw(measurement, data, length);
    return ESP_OK;
}

static esp_err_t parse_tfmini_compatible(const uint8_t *data, size_t length,
                                         tof200_measurement_t *measurement)
{
    if (length < TFMINI_FRAME_LEN || data[0] != 0x59 || data[1] != 0x59 ||
        checksum8(data, TFMINI_FRAME_LEN - 1) != data[TFMINI_FRAME_LEN - 1]) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    const uint16_t distance = (uint16_t)data[2] | ((uint16_t)data[3] << 8);
    const uint16_t strength = (uint16_t)data[4] | ((uint16_t)data[5] << 8);
    const int16_t raw_temperature = (int16_t)((uint16_t)data[6] | ((uint16_t)data[7] << 8));

    if (distance > TOF200_MAX_DISTANCE_MM) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    memset(measurement, 0, sizeof(*measurement));
    measurement->distance_mm = distance;
    measurement->signal_strength = strength;
    measurement->temperature_centi_c = raw_temperature;
    measurement->frame_type = TOF200_FRAME_TFMINI_COMPATIBLE;
    copy_raw(measurement, data, TFMINI_FRAME_LEN);
    return ESP_OK;
}

static esp_err_t parse_modbus_response(const uint8_t *data, size_t length,
                                       tof200_measurement_t *measurement)
{
    if (length < MODBUS_MIN_FRAME_LEN || (data[1] != 0x03 && data[1] != 0x04)) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    const uint8_t byte_count = data[2];
    const size_t expected_length = (size_t)byte_count + 5;
    if (byte_count < 2 || expected_length > length) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    const uint16_t received_crc = (uint16_t)data[expected_length - 2] |
                                  ((uint16_t)data[expected_length - 1] << 8);
    if (crc16_modbus(data, expected_length - 2) != received_crc) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    const uint16_t distance = ((uint16_t)data[3] << 8) | data[4];
    if (distance > TOF200_MAX_DISTANCE_MM) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    memset(measurement, 0, sizeof(*measurement));
    measurement->distance_mm = distance;
    measurement->frame_type = TOF200_FRAME_MODBUS_RESPONSE;
    copy_raw(measurement, data, expected_length);
    return ESP_OK;
}

static esp_err_t try_parse_frame(const uint8_t *data, size_t length,
                                 tof200_measurement_t *measurement)
{
    if (parse_ascii_mm(data, length, measurement) == ESP_OK ||
        parse_tfmini_compatible(data, length, measurement) == ESP_OK ||
        parse_modbus_response(data, length, measurement) == ESP_OK) {
        return ESP_OK;
    }
    return ESP_ERR_INVALID_RESPONSE;
}

static bool deadline_reached(TickType_t deadline)
{
    return (int32_t)(xTaskGetTickCount() - deadline) >= 0;
}

static TickType_t remaining_ticks(TickType_t deadline)
{
    if (deadline_reached(deadline)) {
        return 0;
    }

    TickType_t remaining = deadline - xTaskGetTickCount();
    const TickType_t max_slice = pdMS_TO_TICKS(TOF200_READ_SLICE_MS);
    return remaining > max_slice ? max_slice : remaining;
}

esp_err_t tof200_init(void)
{
    return tof200_init_with_config(NULL);
}

esp_err_t tof200_init_with_config(const tof200_config_t *config)
{
    if (s_initialized) {
        return ESP_OK;
    }

    if (config != NULL) {
        if (config->baud_rate <= 0 || config->rx_buffer_size <= 0) {
            return ESP_ERR_INVALID_ARG;
        }
        s_config = *config;
    }

    if (s_config.uart_num == CONFIG_ESP_CONSOLE_UART_NUM) {
        ESP_LOGE(TAG, "UART%d conflicts with the serial console", s_config.uart_num);
        return ESP_ERR_INVALID_ARG;
    }

    const uart_config_t uart_config = {
        .baud_rate = s_config.baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    esp_err_t err = uart_param_config(s_config.uart_num, &uart_config);
    if (err == ESP_OK) {
        err = uart_set_pin(s_config.uart_num, s_config.tx_gpio, s_config.rx_gpio,
                           UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    }
    if (err == ESP_OK) {
        err = gpio_set_pull_mode(s_config.rx_gpio, GPIO_PULLUP_ONLY);
    }
    if (err == ESP_OK) {
        err = uart_driver_install(s_config.uart_num, s_config.rx_buffer_size, 0, 0, NULL, 0);
    }
    if (err != ESP_OK) {
        return err;
    }

    uart_flush_input(s_config.uart_num);
    s_initialized = true;
    ESP_LOGI(TAG, "TOF200 ready: UART%d RX=GPIO%d TX=GPIO%d %d 8N1",
             s_config.uart_num, s_config.rx_gpio, s_config.tx_gpio, s_config.baud_rate);
    return ESP_OK;
}

esp_err_t tof200_deinit(void)
{
    if (!s_initialized) {
        return ESP_OK;
    }

    esp_err_t err = uart_driver_delete(s_config.uart_num);
    if (err == ESP_OK) {
        s_initialized = false;
    }
    return err;
}

esp_err_t tof200_flush(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    uart_flush_input(s_config.uart_num);
    return ESP_OK;
}

esp_err_t tof200_read_raw(uint8_t *buffer, size_t buffer_size, size_t *bytes_read,
                          uint32_t timeout_ms)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (buffer == NULL || buffer_size == 0 || bytes_read == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const int received = uart_read_bytes(s_config.uart_num, buffer, buffer_size,
                                         pdMS_TO_TICKS(timeout_ms));
    if (received < 0) {
        *bytes_read = 0;
        return ESP_FAIL;
    }

    *bytes_read = (size_t)received;
    return received > 0 ? ESP_OK : ESP_ERR_TIMEOUT;
}

esp_err_t tof200_read_distance(tof200_measurement_t *measurement, uint32_t timeout_ms)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (measurement == NULL || timeout_ms == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t window[sizeof(measurement->raw)] = {0};
    size_t used = 0;
    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    while (!deadline_reached(deadline)) {
        uint8_t byte = 0;
        const int received = uart_read_bytes(s_config.uart_num, &byte, 1,
                                             remaining_ticks(deadline));
        if (received <= 0) {
            continue;
        }

        if (used < sizeof(window)) {
            window[used++] = byte;
        } else {
            memmove(window, window + 1, sizeof(window) - 1);
            window[sizeof(window) - 1] = byte;
        }

        for (size_t offset = 0; offset < used; ++offset) {
            if (try_parse_frame(window + offset, used - offset, measurement) == ESP_OK) {
                return ESP_OK;
            }
        }
    }

    return ESP_ERR_TIMEOUT;
}

esp_err_t tof200_read_distance_mm(uint16_t *distance_mm, uint32_t timeout_ms)
{
    if (distance_mm == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    tof200_measurement_t measurement;
    esp_err_t err = tof200_read_distance(&measurement, timeout_ms);
    if (err == ESP_OK) {
        *distance_mm = measurement.distance_mm;
    }
    return err;
}
