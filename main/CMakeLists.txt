idf_component_register(
    SRCS "main.c" "servo/robot_motion.c" "actions/robot_actions.c"
         "display/ili9488_display.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_gpio esp_driver_uart esp_driver_spi esp_lcd
                  atanisoft__esp_lcd_ili9488
)
