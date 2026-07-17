idf_component_register(
    SRCS "main.c"
         "display/display_app.c"
         "display/display_ili9488.c"
         "sensors/i2c_bus.c"
         "sensors/i2c_mux.c"
         "control/touch/touch_chip.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_gpio esp_driver_ledc esp_driver_i2c hal esp_rom
)
