/*
 * Capacitive touch helpers for the ILI9488 screen.
 *
 * The touch controller is connected through TCA9548A channel 1 on the shared
 * GPIO8/GPIO9 I2C bus. The driver auto-detects common controllers first so
 * hardware bring-up can continue before the exact panel chip is confirmed.
 *
 * Supported bring-up protocols:
 * - FT6x36 / FT6236 / FT6336 at 0x38
 * - GT911 at 0x5D or 0x14
 * - CST816-style register map at 0x15
 */

#pragma once

#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TOUCH_CONTROLLER_NONE = 0,
    TOUCH_CONTROLLER_FT6X36,
    TOUCH_CONTROLLER_GT911,
    TOUCH_CONTROLLER_CST816,
} touch_controller_t;

typedef struct {
    bool online;
    bool pressed;
    int irq_level;
    uint16_t raw_x;
    uint16_t raw_y;
    uint16_t pressure;
    uint8_t i2c_addr;
    uint8_t x_rx[3];
    uint8_t y_rx[3];
    esp_err_t last_error;
    touch_controller_t controller;
} touch_sample_t;

/* Log the configured hardware pins once at boot. */
void touch_chip_boot_probe(void);

/*
 * Read one touch sample. On first call this auto-detects the controller and
 * initialises the I2C path. Returns ESP_OK even when no finger is present
 * (sample->pressed will be false). Repeated failures trigger an automatic
 * hardware re-probe.
 */
esp_err_t touch_chip_read_sample(touch_sample_t *sample);

#ifdef __cplusplus
}
#endif
