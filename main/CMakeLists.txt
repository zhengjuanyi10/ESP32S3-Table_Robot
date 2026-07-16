idf_component_register(
    SRCS "main.c"
         "display/display_app.c"
         "display/display_ili9488.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_gpio esp_driver_ledc hal esp_rom
)
