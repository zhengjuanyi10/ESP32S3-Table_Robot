/* TOF200F UART distance sensor driver interface. */

#pragma once

#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"

/*
 * Wiring is named from the TOF200F module side:
 *   TOF200F TXD -> ESP32-S3 GPIO4  (ESP UART RX)
 *   TOF200F RXD -> ESP32-S3 GPIO5  (ESP UART TX)
 */
#define TOF200_DEFAULT_UART_NUM UART_NUM_2
#define TOF200_DEFAULT_BAUD_RATE 115200
#define TOF200_DEFAULT_RX_GPIO GPIO_NUM_4
#define TOF200_DEFAULT_TX_GPIO GPIO_NUM_5
#define TOF200_DEFAULT_RX_BUFFER_SIZE 256

typedef struct {
    uart_port_t uart_num;
    int baud_rate;
    gpio_num_t rx_gpio;
    gpio_num_t tx_gpio;
    int rx_buffer_size;
} tof200_config_t;

typedef enum {
    TOF200_FRAME_UNKNOWN = 0,
    TOF200_FRAME_ASCII_MM,
    TOF200_FRAME_TFMINI_COMPATIBLE,
    TOF200_FRAME_MODBUS_RESPONSE,
} tof200_frame_type_t;

typedef struct {
    uint16_t distance_mm;
    uint16_t signal_strength;
    int16_t temperature_centi_c;
    tof200_frame_type_t frame_type;
    uint8_t raw[16];
    size_t raw_length;
} tof200_measurement_t;

esp_err_t tof200_init(void);
esp_err_t tof200_init_with_config(const tof200_config_t *config);
esp_err_t tof200_deinit(void);
esp_err_t tof200_flush(void);

esp_err_t tof200_read_distance(tof200_measurement_t *measurement, uint32_t timeout_ms);
esp_err_t tof200_read_distance_mm(uint16_t *distance_mm, uint32_t timeout_ms);

esp_err_t tof200_read_raw(uint8_t *buffer, size_t buffer_size, size_t *bytes_read,
                          uint32_t timeout_ms);
