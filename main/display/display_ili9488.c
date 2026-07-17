/*
 * 功能：驱动 CL35BC106-40A/ILI9488，并提供像素、图形、文字和图片绘制。
 * 配置：引脚、GPIO 边沿延时和默认背光亮度来自 Table Robot menuconfig。
 * 波形：4 线 GPIO 模拟 SPI，SCLK 空闲为高，每个命令/参数/像素独立拉低 CS；
 *       像素以 RGB565 输入，在发送前转换为面板要求的 RGB666。
 * 背光：GPIO11 默认使用 LEDC PWM，经外部三极管/MOSFET驱动背光。
 */

#include "display/display_ili9488.h"

#include <limits.h>
#include <stddef.h>
#include <stdlib.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_struct.h"

#define ILI9488_NATIVE_WIDTH 320
#define ILI9488_NATIVE_HEIGHT 480
#define ILI9488_LANDSCAPE_WIDTH 480
#define ILI9488_LANDSCAPE_HEIGHT 320
#define ILI9488_BACKLIGHT_PWM_HZ 5000
#define ILI9488_BACKLIGHT_DUTY_MAX 1023U

/* 驱动状态由模块内部持有，公开绘图接口通过同一互斥锁串行访问面板。 */
static const char *TAG = "display_ili9488";

static SemaphoreHandle_t s_lock;
static uint16_t s_width = ILI9488_LANDSCAPE_WIDTH;
static uint16_t s_height = ILI9488_LANDSCAPE_HEIGHT;
static bool s_pins_configured;
static bool s_initialized;
static bool s_backlight_pwm_configured;
static bool s_backlight_enabled;
static uint8_t s_brightness_percent =
    CONFIG_TABLE_ROBOT_DISPLAY_BRIGHTNESS_PERCENT;

typedef struct {
    uint8_t command;
    uint8_t data[15];
    uint8_t data_size;
    uint16_t delay_ms;
} ili9488_init_command_t;

static inline void IRAM_ATTR pin_write(int pin, unsigned level)
{
    gpio_ll_set_level(&GPIO, pin, level);
}

static inline void IRAM_ATTR clock_byte(uint8_t value)
{
    for (unsigned bit = 0; bit < 8; ++bit) {
        /* MOSI 在时钟下降沿前建立，ILI9488 在上升沿采样数据。 */
        pin_write(CONFIG_TABLE_ROBOT_DISPLAY_MOSI_GPIO,
                  (value & 0x80U) != 0U);
        value <<= 1;
        pin_write(CONFIG_TABLE_ROBOT_DISPLAY_SCLK_GPIO, 0);
#if CONFIG_TABLE_ROBOT_DISPLAY_GPIO_DELAY_US > 0
        esp_rom_delay_us(CONFIG_TABLE_ROBOT_DISPLAY_GPIO_DELAY_US);
#endif
        pin_write(CONFIG_TABLE_ROBOT_DISPLAY_SCLK_GPIO, 1);
#if CONFIG_TABLE_ROBOT_DISPLAY_GPIO_DELAY_US > 0
        esp_rom_delay_us(CONFIG_TABLE_ROBOT_DISPLAY_GPIO_DELAY_US);
#endif
    }
}

static void IRAM_ATTR write_byte(bool is_data, uint8_t value)
{
    pin_write(CONFIG_TABLE_ROBOT_DISPLAY_CS_GPIO, 0);
    pin_write(CONFIG_TABLE_ROBOT_DISPLAY_DC_GPIO, is_data ? 1 : 0);
    clock_byte(value);
    pin_write(CONFIG_TABLE_ROBOT_DISPLAY_CS_GPIO, 1);
}

static void IRAM_ATTR write_pixel(uint8_t red, uint8_t green, uint8_t blue)
{
    /* 一个 RGB666 像素的三字节数据共用一次独立 CS 低电平。 */
    pin_write(CONFIG_TABLE_ROBOT_DISPLAY_CS_GPIO, 0);
    pin_write(CONFIG_TABLE_ROBOT_DISPLAY_DC_GPIO, 1);
    clock_byte(red);
    clock_byte(green);
    clock_byte(blue);
    pin_write(CONFIG_TABLE_ROBOT_DISPLAY_CS_GPIO, 1);
}

static esp_err_t configure_pins(void)
{
    /* GPIO 只配置一次；反初始化仅关闭面板状态和背光。 */
    if (s_pins_configured) {
        return ESP_OK;
    }

    uint64_t output_pin_mask =
            (1ULL << CONFIG_TABLE_ROBOT_DISPLAY_SCLK_GPIO) |
            (1ULL << CONFIG_TABLE_ROBOT_DISPLAY_MOSI_GPIO) |
            (1ULL << CONFIG_TABLE_ROBOT_DISPLAY_CS_GPIO) |
            (1ULL << CONFIG_TABLE_ROBOT_DISPLAY_DC_GPIO) |
            (1ULL << CONFIG_TABLE_ROBOT_DISPLAY_RESET_GPIO);
#if CONFIG_TABLE_ROBOT_DISPLAY_BACKLIGHT_GPIO >= 0
    output_pin_mask |=
        1ULL << CONFIG_TABLE_ROBOT_DISPLAY_BACKLIGHT_GPIO;
#endif
    const gpio_config_t config = {
        .pin_bit_mask = output_pin_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&config);
    if (err != ESP_OK) {
        return err;
    }

    /* 同时打开 RESET 输入路径，用于确认释放后确实回到高电平。 */
    err = gpio_set_direction(CONFIG_TABLE_ROBOT_DISPLAY_RESET_GPIO,
                             GPIO_MODE_INPUT_OUTPUT);
    if (err != ESP_OK) {
        return err;
    }

    pin_write(CONFIG_TABLE_ROBOT_DISPLAY_CS_GPIO, 1);
    pin_write(CONFIG_TABLE_ROBOT_DISPLAY_DC_GPIO, 1);
    pin_write(CONFIG_TABLE_ROBOT_DISPLAY_SCLK_GPIO, 1);
    pin_write(CONFIG_TABLE_ROBOT_DISPLAY_MOSI_GPIO, 0);
    pin_write(CONFIG_TABLE_ROBOT_DISPLAY_RESET_GPIO, 1);
#if CONFIG_TABLE_ROBOT_DISPLAY_BACKLIGHT_GPIO >= 0
    pin_write(CONFIG_TABLE_ROBOT_DISPLAY_BACKLIGHT_GPIO, 0);
#endif
    s_pins_configured = true;

    ESP_LOGI(TAG,
             "GPIO bit-bang: SCLK=%d MOSI=%d CS=%d DC=%d RST=%d BL=%d",
             CONFIG_TABLE_ROBOT_DISPLAY_SCLK_GPIO,
             CONFIG_TABLE_ROBOT_DISPLAY_MOSI_GPIO,
             CONFIG_TABLE_ROBOT_DISPLAY_CS_GPIO,
             CONFIG_TABLE_ROBOT_DISPLAY_DC_GPIO,
             CONFIG_TABLE_ROBOT_DISPLAY_RESET_GPIO,
             CONFIG_TABLE_ROBOT_DISPLAY_BACKLIGHT_GPIO);
    return ESP_OK;
}

static esp_err_t configure_backlight_pwm(void)
{
#if CONFIG_TABLE_ROBOT_DISPLAY_BACKLIGHT_GPIO >= 0
    if (s_backlight_pwm_configured) {
        return ESP_OK;
    }

    /* 5 kHz/10 bit 可避免可见闪烁，并保留足够的亮度分辨率。 */
    const ledc_timer_config_t timer_config = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = ILI9488_BACKLIGHT_PWM_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_config);
    if (err != ESP_OK) {
        return err;
    }

    const ledc_channel_config_t channel_config = {
        .gpio_num = CONFIG_TABLE_ROBOT_DISPLAY_BACKLIGHT_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    err = ledc_channel_config(&channel_config);
    if (err == ESP_OK) {
        s_backlight_pwm_configured = true;
    }
    return err;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

static esp_err_t apply_backlight_pwm(void)
{
#if CONFIG_TABLE_ROBOT_DISPLAY_BACKLIGHT_GPIO >= 0
    const uint32_t duty = s_backlight_enabled
                              ? (ILI9488_BACKLIGHT_DUTY_MAX *
                                     s_brightness_percent +
                                 50U) /
                                    100U
                              : 0U;
    esp_err_t err = ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0,
                                  duty);
    if (err == ESP_OK) {
        err = ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
    return err;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

static void hardware_reset(void)
{
    /* RESET 依次保持高 100 ms、低 100 ms、高 120 ms。 */
    pin_write(CONFIG_TABLE_ROBOT_DISPLAY_RESET_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    pin_write(CONFIG_TABLE_ROBOT_DISPLAY_RESET_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    pin_write(CONFIG_TABLE_ROBOT_DISPLAY_RESET_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(120));
    const int reset_level =
        gpio_get_level(CONFIG_TABLE_ROBOT_DISPLAY_RESET_GPIO);
    if (reset_level == 0) {
        ESP_LOGE(TAG, "LCD RESET GPIO%d is still LOW after release",
                 CONFIG_TABLE_ROBOT_DISPLAY_RESET_GPIO);
    } else {
        ESP_LOGI(TAG, "LCD RESET GPIO%d released HIGH",
                 CONFIG_TABLE_ROBOT_DISPLAY_RESET_GPIO);
    }
}

static void write_command_with_parameters(uint8_t command,
                                          const uint8_t *data,
                                          size_t data_size)
{
    write_byte(false, command);
    for (size_t i = 0; i < data_size; ++i) {
        write_byte(true, data[i]);
    }
}

static void initialize_panel(void)
{
    /* 依次配置电源、Gamma、扫描方向、RGB666 模式和睡眠/显示状态。 */
    static const ili9488_init_command_t commands[] = {
        {0xF7, {0xA9, 0x51, 0x2C, 0x82}, 4, 0},
        {0xEC, {0x00, 0x02, 0x03, 0x7A}, 4, 0},
        {0xC0, {0x13, 0x13}, 2, 0},
        {0xC1, {0x41}, 1, 0},
        {0xC5, {0x00, 0x28, 0x80}, 3, 0},
        {0xB0, {0x00}, 1, 0},
        {0xB1, {0xB0, 0x11}, 2, 0},
        {0xB4, {0x02}, 1, 0},
        {0xB6, {0x02, 0x22}, 2, 0},
        {0xB7, {0xC6}, 1, 0},
        {0xBE, {0x00, 0x04}, 2, 0},
        {0xE9, {0x00}, 1, 0},
        {0xF4, {0x00, 0x00, 0x0F}, 3, 0},
        {0xE0,
         {0x00, 0x04, 0x0E, 0x08, 0x17, 0x0A, 0x40, 0x79,
          0x4D, 0x07, 0x0E, 0x0A, 0x1A, 0x1D, 0x0F},
         15, 0},
        {0xE1,
         {0x00, 0x1B, 0x1F, 0x02, 0x10, 0x05, 0x32, 0x34,
          0x43, 0x02, 0x0A, 0x09, 0x33, 0x37, 0x0F},
         15, 0},
        {0xF4, {0x00, 0x00, 0x0F}, 3, 0},
        {0x36, {0x68}, 1, 0},
        {0x3A, {0x66}, 1, 0},
        {0x20, {0}, 0, 0},
        {0x11, {0}, 0, 120},
        {0x29, {0}, 0, 50},
    };

    for (size_t i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i) {
        const ili9488_init_command_t *entry = &commands[i];
        write_command_with_parameters(entry->command, entry->data,
                                      entry->data_size);
        if (entry->delay_ms > 0) {
            vTaskDelay(pdMS_TO_TICKS(entry->delay_ms));
        }
    }
}

static void set_window(int x, int y, int width, int height)
{
    /* ILI9488 的 0x2A/0x2B 设置写入窗口，0x2C 开始写显存。 */
    const int x_end = x + width - 1;
    const int y_end = y + height - 1;
    const uint8_t columns[] = {
        (uint8_t)(x >> 8), (uint8_t)x,
        (uint8_t)(x_end >> 8), (uint8_t)x_end,
    };
    const uint8_t rows[] = {
        (uint8_t)(y >> 8), (uint8_t)y,
        (uint8_t)(y_end >> 8), (uint8_t)y_end,
    };

    write_command_with_parameters(0x2A, columns, sizeof(columns));
    write_command_with_parameters(0x2B, rows, sizeof(rows));
    write_byte(false, 0x2C);
}

static inline void rgb565_to_rgb666(uint16_t color, uint8_t *red,
                                    uint8_t *green, uint8_t *blue)
{
    /* 扩展低位后输出 6 bit 有效数据，满足面板 SPI RGB666 模式。 */
    *red = (uint8_t)(((color & 0xF800U) >> 8) |
                     ((color & 0x8000U) >> 13));
    *green = (uint8_t)((color & 0x07E0U) >> 3);
    *blue = (uint8_t)(((color & 0x001FU) << 3) |
                      ((color & 0x0010U) >> 2));
}

static void service_idle_task(int row)
{
    if ((row & 7) == 7) {
        vTaskDelay(1);
    }
}

static esp_err_t take_display_lock(void)
{
    return xSemaphoreTake(s_lock, pdMS_TO_TICKS(1000)) == pdTRUE
               ? ESP_OK
               : ESP_ERR_TIMEOUT;
}

static bool clip_rectangle(int x, int y, int width, int height,
                           int *clipped_x, int *clipped_y,
                           int *clipped_width, int *clipped_height)
{
    /* 使用 64 位边界计算，避免 x+width 等整数相加溢出。 */
    const int64_t left = x;
    const int64_t top = y;
    const int64_t right = left + width;
    const int64_t bottom = top + height;

    if (right <= 0 || bottom <= 0 || left >= s_width || top >= s_height) {
        return false;
    }

    const int64_t visible_left = left < 0 ? 0 : left;
    const int64_t visible_top = top < 0 ? 0 : top;
    const int64_t visible_right = right > s_width ? s_width : right;
    const int64_t visible_bottom = bottom > s_height ? s_height : bottom;

    *clipped_x = (int)visible_left;
    *clipped_y = (int)visible_top;
    *clipped_width = (int)(visible_right - visible_left);
    *clipped_height = (int)(visible_bottom - visible_top);
    return *clipped_width > 0 && *clipped_height > 0;
}

static esp_err_t fill_rect_unlocked(int x, int y, int width, int height,
                                    uint16_t color)
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
    rgb565_to_rgb666(color, &red, &green, &blue);
    set_window(x, y, width, height);

    /*
     * After set_window sends 0x2C (memory-write), the panel expects a
     * continuous pixel stream.  Keep CS and DC LOW/HIGH for the entire
     * burst instead of toggling them per pixel — this eliminates
     * 2*(W*H) GPIO writes and makes fills near-instant.
     */
    const int total = width * height;
    pin_write(CONFIG_TABLE_ROBOT_DISPLAY_CS_GPIO, 0);
    pin_write(CONFIG_TABLE_ROBOT_DISPLAY_DC_GPIO, 1);

    for (int i = 0; i < total; ++i) {
        clock_byte(red);
        clock_byte(green);
        clock_byte(blue);
        if ((i & 0x7FF) == 0x7FF) {   /* yield every 2048 pixels */
            vTaskDelay(1);
        }
    }

    pin_write(CONFIG_TABLE_ROBOT_DISPLAY_CS_GPIO, 1);
    return ESP_OK;
}

static esp_err_t fill_rect_clipped_unlocked(int x, int y, int width,
                                            int height, uint16_t color)
{
    int clipped_x;
    int clipped_y;
    int clipped_width;
    int clipped_height;
    if (!clip_rectangle(x, y, width, height, &clipped_x, &clipped_y,
                        &clipped_width, &clipped_height)) {
        return ESP_OK;
    }
    return fill_rect_unlocked(clipped_x, clipped_y, clipped_width,
                              clipped_height, color);
}

static uint8_t line_out_code(int x, int y, int right, int bottom)
{
    uint8_t code = 0;
    if (x < 0) {
        code |= 0x01;
    } else if (x > right) {
        code |= 0x02;
    }
    if (y < 0) {
        code |= 0x04;
    } else if (y > bottom) {
        code |= 0x08;
    }
    return code;
}

static bool clip_line_to_screen(int *x0, int *y0, int *x1, int *y1)
{
    /* Cohen-Sutherland 裁剪，防止超大坐标导致逐像素循环过久。 */
    const int right = (int)s_width - 1;
    const int bottom = (int)s_height - 1;
    uint8_t code0 = line_out_code(*x0, *y0, right, bottom);
    uint8_t code1 = line_out_code(*x1, *y1, right, bottom);

    for (;;) {
        if ((code0 | code1) == 0) {
            return true;
        }
        if ((code0 & code1) != 0) {
            return false;
        }

        const uint8_t outside = code0 != 0 ? code0 : code1;
        int64_t x;
        int64_t y;
        if ((outside & 0x04) != 0) {
            if (*y1 == *y0) {
                return false;
            }
            y = 0;
            x = (int64_t)*x0 +
                ((int64_t)*x1 - *x0) * (0 - (int64_t)*y0) /
                    (*y1 - *y0);
        } else if ((outside & 0x08) != 0) {
            if (*y1 == *y0) {
                return false;
            }
            y = bottom;
            x = (int64_t)*x0 +
                ((int64_t)*x1 - *x0) * (bottom - (int64_t)*y0) /
                    (*y1 - *y0);
        } else if ((outside & 0x02) != 0) {
            if (*x1 == *x0) {
                return false;
            }
            x = right;
            y = (int64_t)*y0 +
                ((int64_t)*y1 - *y0) * (right - (int64_t)*x0) /
                    (*x1 - *x0);
        } else {
            if (*x1 == *x0) {
                return false;
            }
            x = 0;
            y = (int64_t)*y0 +
                ((int64_t)*y1 - *y0) * (0 - (int64_t)*x0) /
                    (*x1 - *x0);
        }

        if (outside == code0) {
            *x0 = (int)x;
            *y0 = (int)y;
            code0 = line_out_code(*x0, *y0, right, bottom);
        } else {
            *x1 = (int)x;
            *y1 = (int)y;
            code1 = line_out_code(*x1, *y1, right, bottom);
        }
    }
}

typedef struct {
    char character;
    uint8_t rows[7];
} font_5x7_glyph_t;

/* 内置字体覆盖数字、大写字母、空格、连字符、冒号、百分号和问号。 */
static const font_5x7_glyph_t s_font_5x7[] = {
    {' ', {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}},
    {'-', {0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00}},
    {':', {0x00, 0x04, 0x04, 0x00, 0x04, 0x04, 0x00}},
    {'%', {0x19, 0x1A, 0x04, 0x08, 0x0B, 0x13, 0x00}},
    {'?', {0x0E, 0x11, 0x01, 0x02, 0x04, 0x00, 0x04}},
    {'0', {0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E}},
    {'1', {0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E}},
    {'2', {0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F}},
    {'3', {0x1E, 0x01, 0x01, 0x0E, 0x01, 0x01, 0x1E}},
    {'4', {0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02}},
    {'5', {0x1F, 0x10, 0x10, 0x1E, 0x01, 0x01, 0x1E}},
    {'6', {0x0E, 0x10, 0x10, 0x1E, 0x11, 0x11, 0x0E}},
    {'7', {0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08}},
    {'8', {0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E}},
    {'9', {0x0E, 0x11, 0x11, 0x0F, 0x01, 0x01, 0x0E}},
    {'A', {0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}},
    {'B', {0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E}},
    {'C', {0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E}},
    {'D', {0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E}},
    {'E', {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F}},
    {'F', {0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10}},
    {'G', {0x0E, 0x11, 0x10, 0x17, 0x11, 0x11, 0x0F}},
    {'H', {0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11}},
    {'I', {0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E}},
    {'J', {0x07, 0x02, 0x02, 0x02, 0x12, 0x12, 0x0C}},
    {'K', {0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11}},
    {'L', {0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F}},
    {'M', {0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11}},
    {'N', {0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11}},
    {'O', {0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}},
    {'P', {0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10}},
    {'Q', {0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D}},
    {'R', {0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11}},
    {'S', {0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E}},
    {'T', {0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04}},
    {'U', {0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E}},
    {'V', {0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04}},
    {'W', {0x11, 0x11, 0x11, 0x15, 0x15, 0x15, 0x0A}},
    {'X', {0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11}},
    {'Y', {0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04}},
    {'Z', {0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F}},
};

static const uint8_t *font_5x7_rows(char character)
{
    if (character >= 'a' && character <= 'z') {
        character = (char)(character - 'a' + 'A');
    }
    for (size_t index = 0;
         index < sizeof(s_font_5x7) / sizeof(s_font_5x7[0]); ++index) {
        if (s_font_5x7[index].character == character) {
            return s_font_5x7[index].rows;
        }
    }
    return font_5x7_rows('?');
}

esp_err_t ili9488_display_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    esp_err_t err = configure_pins();
    if (err != ESP_OK) {
        return err;
    }
    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }

    /* 面板初始化期间保持背光关闭，完成后再按默认 PWM 亮度点亮。 */
    hardware_reset();
    initialize_panel();
    s_width = ILI9488_LANDSCAPE_WIDTH;
    s_height = ILI9488_LANDSCAPE_HEIGHT;
#if CONFIG_TABLE_ROBOT_DISPLAY_BACKLIGHT_GPIO >= 0
    err = configure_backlight_pwm();
    if (err != ESP_OK) {
        vSemaphoreDelete(s_lock);
        s_lock = NULL;
        return err;
    }
    s_backlight_enabled = true;
    err = apply_backlight_pwm();
    if (err != ESP_OK) {
        vSemaphoreDelete(s_lock);
        s_lock = NULL;
        return err;
    }
#endif
    s_initialized = true;

    ESP_LOGI(TAG,
             "ILI9488 backend ready: %ux%u RGB666 GPIO, delay=%dus, BL=%u%%",
             s_width, s_height,
             CONFIG_TABLE_ROBOT_DISPLAY_GPIO_DELAY_US,
             s_brightness_percent);
    return ESP_OK;
}

esp_err_t ili9488_display_deinit(void)
{
#if CONFIG_TABLE_ROBOT_DISPLAY_BACKLIGHT_GPIO >= 0
    if (s_backlight_pwm_configured) {
        s_backlight_enabled = false;
        (void)apply_backlight_pwm();
    }
#endif
    if (s_lock != NULL) {
        vSemaphoreDelete(s_lock);
        s_lock = NULL;
    }
    s_initialized = false;
    return ESP_OK;
}

bool ili9488_display_is_initialized(void)
{
    return s_initialized;
}

esp_err_t ili9488_display_set_backlight(bool enabled)
{
#if CONFIG_TABLE_ROBOT_DISPLAY_BACKLIGHT_GPIO >= 0
    esp_err_t err = configure_pins();
    if (err == ESP_OK) {
        err = configure_backlight_pwm();
    }
    if (err == ESP_OK) {
        s_backlight_enabled = enabled;
        err = apply_backlight_pwm();
    }
    return err;
#else
    (void)enabled;
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

esp_err_t ili9488_display_set_brightness(uint8_t percent)
{
    if (percent > 100U) {
        return ESP_ERR_INVALID_ARG;
    }
    s_brightness_percent = percent;
    if (!s_backlight_pwm_configured) {
        return ESP_OK;
    }
    return apply_backlight_pwm();
}

esp_err_t ili9488_display_set_rotation(ili9488_display_rotation_t rotation)
{
    static const uint8_t madctl[] = {0x08, 0x68, 0xC8, 0xA8};
    static const bool swap_xy[] = {false, true, false, true};

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (rotation > ILI9488_DISPLAY_ROTATION_270) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = take_display_lock();
    if (err != ESP_OK) {
        return err;
    }
    write_command_with_parameters(0x36, &madctl[rotation], 1);
    s_width = swap_xy[rotation] ? ILI9488_LANDSCAPE_WIDTH
                                : ILI9488_NATIVE_WIDTH;
    s_height = swap_xy[rotation] ? ILI9488_LANDSCAPE_HEIGHT
                                 : ILI9488_NATIVE_HEIGHT;
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

uint16_t ili9488_display_width(void)
{
    return s_width;
}

uint16_t ili9488_display_height(void)
{
    return s_height;
}

esp_err_t ili9488_display_draw_pixel(int x, int y, uint16_t color)
{
    return ili9488_display_fill_rect(x, y, 1, 1, color);
}

esp_err_t ili9488_display_draw_hline(int x, int y, int width,
                                     uint16_t color)
{
    if (width <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    return ili9488_display_fill_rect(x, y, width, 1, color);
}

esp_err_t ili9488_display_draw_vline(int x, int y, int height,
                                     uint16_t color)
{
    if (height <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    return ili9488_display_fill_rect(x, y, 1, height, color);
}

esp_err_t ili9488_display_draw_line(int x0, int y0, int x1, int y1,
                                    uint16_t color)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = take_display_lock();
    if (err != ESP_OK) {
        return err;
    }
    if (!clip_line_to_screen(&x0, &y0, &x1, &y1)) {
        xSemaphoreGive(s_lock);
        return ESP_OK;
    }

    /* 水平/垂直线合并成单个矩形写入，斜线才使用 Bresenham。 */
    if (y0 == y1) {
        err = fill_rect_unlocked(x0 < x1 ? x0 : x1, y0,
                                 abs(x1 - x0) + 1, 1, color);
        xSemaphoreGive(s_lock);
        return err;
    }
    if (x0 == x1) {
        err = fill_rect_unlocked(x0, y0 < y1 ? y0 : y1, 1,
                                 abs(y1 - y0) + 1, color);
        xSemaphoreGive(s_lock);
        return err;
    }

    const int delta_x = abs(x1 - x0);
    const int step_x = x0 < x1 ? 1 : -1;
    const int delta_y = -abs(y1 - y0);
    const int step_y = y0 < y1 ? 1 : -1;
    int error = delta_x + delta_y;
    for (;;) {
        err = fill_rect_unlocked(x0, y0, 1, 1, color);
        if (err != ESP_OK || (x0 == x1 && y0 == y1)) {
            break;
        }
        const int twice_error = error * 2;
        if (twice_error >= delta_y) {
            error += delta_y;
            x0 += step_x;
        }
        if (twice_error <= delta_x) {
            error += delta_x;
            y0 += step_y;
        }
    }

    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t ili9488_display_draw_rect(int x, int y, int width, int height,
                                    uint16_t color)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (width <= 0 || height <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if ((int64_t)x + width - 1 > INT_MAX ||
        (int64_t)y + height - 1 > INT_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = take_display_lock();
    if (err != ESP_OK) {
        return err;
    }

    err = fill_rect_clipped_unlocked(x, y, width, 1, color);
    if (err == ESP_OK && height > 1) {
        err = fill_rect_clipped_unlocked(x, y + height - 1, width, 1,
                                         color);
    }
    if (err == ESP_OK && height > 2) {
        err = fill_rect_clipped_unlocked(x, y + 1, 1, height - 2, color);
    }
    if (err == ESP_OK && width > 1 && height > 2) {
        err = fill_rect_clipped_unlocked(x + width - 1, y + 1, 1,
                                         height - 2, color);
    }

    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t ili9488_display_draw_circle(int center_x, int center_y, int radius,
                                      uint16_t color)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (radius < 0 || radius > INT16_MAX || center_x < INT_MIN + radius ||
        center_x > INT_MAX - radius || center_y < INT_MIN + radius ||
        center_y > INT_MAX - radius) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = take_display_lock();
    if (err != ESP_OK) {
        return err;
    }

    int circle_x = radius;
    int circle_y = 0;
    int decision = 1 - radius;
    while (circle_x >= circle_y && err == ESP_OK) {
        const int points[][2] = {
            {center_x + circle_x, center_y + circle_y},
            {center_x + circle_y, center_y + circle_x},
            {center_x - circle_y, center_y + circle_x},
            {center_x - circle_x, center_y + circle_y},
            {center_x - circle_x, center_y - circle_y},
            {center_x - circle_y, center_y - circle_x},
            {center_x + circle_y, center_y - circle_x},
            {center_x + circle_x, center_y - circle_y},
        };
        for (size_t index = 0; index < sizeof(points) / sizeof(points[0]);
             ++index) {
            err = fill_rect_clipped_unlocked(points[index][0],
                                             points[index][1], 1, 1, color);
            if (err != ESP_OK) {
                break;
            }
        }
        ++circle_y;
        if (decision < 0) {
            decision += 2 * circle_y + 1;
        } else {
            --circle_x;
            decision += 2 * (circle_y - circle_x) + 1;
        }
    }

    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t ili9488_display_fill_circle(int center_x, int center_y, int radius,
                                      uint16_t color)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (radius < 0 || radius > INT16_MAX || center_x < INT_MIN + radius ||
        center_x > INT_MAX - radius || center_y < INT_MIN + radius ||
        center_y > INT_MAX - radius) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = take_display_lock();
    if (err != ESP_OK) {
        return err;
    }

    int circle_x = radius;
    int circle_y = 0;
    int decision = 1 - radius;
    while (circle_x >= circle_y && err == ESP_OK) {
        err = fill_rect_clipped_unlocked(center_x - circle_x,
                                         center_y + circle_y,
                                         2 * circle_x + 1, 1, color);
        if (err == ESP_OK && circle_y != 0) {
            err = fill_rect_clipped_unlocked(center_x - circle_x,
                                             center_y - circle_y,
                                             2 * circle_x + 1, 1, color);
        }
        if (err == ESP_OK && circle_x != circle_y) {
            err = fill_rect_clipped_unlocked(center_x - circle_y,
                                             center_y + circle_x,
                                             2 * circle_y + 1, 1, color);
        }
        if (err == ESP_OK && circle_x != circle_y && circle_x != 0) {
            err = fill_rect_clipped_unlocked(center_x - circle_y,
                                             center_y - circle_x,
                                             2 * circle_y + 1, 1, color);
        }
        ++circle_y;
        if (decision < 0) {
            decision += 2 * circle_y + 1;
        } else {
            --circle_x;
            decision += 2 * (circle_y - circle_x) + 1;
        }
    }

    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t ili9488_display_draw_bitmap(int x, int y, int width, int height,
                                      const uint16_t *rgb565_pixels)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (rgb565_pixels == NULL || width <= 0 || height <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 裁剪后保留原图行跨度，确保源像素偏移仍指向正确位置。 */
    int destination_x;
    int destination_y;
    int visible_width;
    int visible_height;
    if (!clip_rectangle(x, y, width, height, &destination_x, &destination_y,
                        &visible_width, &visible_height)) {
        return ESP_OK;
    }
    const int source_x = (int)((int64_t)destination_x - x);
    const int source_y = (int)((int64_t)destination_y - y);

    esp_err_t err = take_display_lock();
    if (err != ESP_OK) {
        return err;
    }

    set_window(destination_x, destination_y, visible_width, visible_height);
    for (int row = 0; row < visible_height; ++row) {
        for (int column = 0; column < visible_width; ++column) {
            uint8_t red;
            uint8_t green;
            uint8_t blue;
            const size_t source_offset =
                (size_t)(source_y + row) * (size_t)width +
                (size_t)(source_x + column);
            rgb565_to_rgb666(rgb565_pixels[source_offset], &red, &green,
                             &blue);
            write_pixel(red, green, blue);
        }
        service_idle_task(row);
    }

    xSemaphoreGive(s_lock);
    return ESP_OK;
}

esp_err_t ili9488_display_draw_rgb888(int x, int y, int width, int height,
                                      const uint8_t *rgb888_pixels)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (rgb888_pixels == NULL || width <= 0 || height <= 0) {
        return ESP_ERR_INVALID_ARG;
    }

    int destination_x;
    int destination_y;
    int visible_width;
    int visible_height;
    if (!clip_rectangle(x, y, width, height, &destination_x, &destination_y,
                        &visible_width, &visible_height)) {
        return ESP_OK;
    }
    const int source_x = (int)((int64_t)destination_x - x);
    const int source_y = (int)((int64_t)destination_y - y);

    esp_err_t err = take_display_lock();
    if (err != ESP_OK) {
        return err;
    }

    set_window(destination_x, destination_y, visible_width, visible_height);
    for (int row = 0; row < visible_height; ++row) {
        for (int column = 0; column < visible_width; ++column) {
            const size_t offset =
                ((size_t)(source_y + row) * (size_t)width +
                 (size_t)(source_x + column)) * 3U;
            write_pixel(rgb888_pixels[offset] & 0xFCU,
                        rgb888_pixels[offset + 1] & 0xFCU,
                        rgb888_pixels[offset + 2] & 0xFCU);
        }
        service_idle_task(row);
    }

    xSemaphoreGive(s_lock);
    return ESP_OK;
}

esp_err_t ili9488_display_fill_rect(int x, int y, int width, int height,
                                    uint16_t rgb565_color)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (width <= 0 || height <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = take_display_lock();
    if (err != ESP_OK) {
        return err;
    }

    err = fill_rect_clipped_unlocked(x, y, width, height, rgb565_color);
    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t ili9488_display_fill(uint16_t rgb565_color)
{
    return ili9488_display_fill_rect(0, 0, s_width, s_height, rgb565_color);
}

esp_err_t ili9488_display_draw_mono_bitmap(
    int x, int y, int width, int height, const uint8_t *bitmap,
    size_t row_stride_bytes, uint16_t foreground, uint16_t background,
    bool transparent_background)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (bitmap == NULL || width <= 0 || height <= 0) {
        return ESP_ERR_INVALID_ARG;
    }
    const size_t minimum_stride = ((size_t)width + 7U) / 8U;
    if (row_stride_bytes == 0) {
        row_stride_bytes = minimum_stride;
    } else if (row_stride_bytes < minimum_stride) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = take_display_lock();
    if (err != ESP_OK) {
        return err;
    }
    /* 将连续同色 bit 合并为水平线，减少窗口设置和 GPIO 事务数量。 */
    for (int row = 0; row < height && err == ESP_OK; ++row) {
        const uint8_t *source_row = bitmap + (size_t)row * row_stride_bytes;
        int run_start = 0;
        bool run_set = (source_row[0] & 0x80U) != 0;
        for (int column = 1; column <= width; ++column) {
            const bool bit_set =
                column < width &&
                (source_row[(size_t)column / 8U] &
                 (uint8_t)(0x80U >> ((unsigned)column & 7U))) != 0;
            if (column == width || bit_set != run_set) {
                if (run_set || !transparent_background) {
                    err = fill_rect_clipped_unlocked(
                        x + run_start, y + row, column - run_start, 1,
                        run_set ? foreground : background);
                    if (err != ESP_OK) {
                        break;
                    }
                }
                run_start = column;
                run_set = bit_set;
            }
        }
    }

    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t ili9488_display_draw_text(int x, int y, const char *text,
                                    unsigned scale, uint16_t foreground,
                                    uint16_t background,
                                    bool transparent_background)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (text == NULL || scale == 0 || scale > 32U) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = take_display_lock();
    if (err != ESP_OK) {
        return err;
    }

    /* 每个字符占 6x7 单元：5 列字形加 1 列字符间距。 */
    const int64_t start_x = x;
    int64_t cursor_x = x;
    int64_t cursor_y = y;
    for (const char *current = text; *current != '\0' && err == ESP_OK;
         ++current) {
        if (*current == '\n') {
            cursor_x = start_x;
            cursor_y += (int64_t)scale * 8;
            continue;
        }

        if (cursor_x > INT_MAX || cursor_y > INT_MAX) {
            break;
        }
        if (cursor_x >= INT_MIN && cursor_y >= INT_MIN) {
            if (!transparent_background) {
                err = fill_rect_clipped_unlocked(
                    (int)cursor_x, (int)cursor_y, (int)(6U * scale),
                    (int)(7U * scale), background);
            }

            const uint8_t *rows = font_5x7_rows(*current);
            for (int row = 0; row < 7 && err == ESP_OK; ++row) {
                int column = 0;
                while (column < 5) {
                    while (column < 5 &&
                           (rows[row] & (uint8_t)(0x10U >> column)) == 0) {
                        ++column;
                    }
                    const int run_start = column;
                    while (column < 5 &&
                           (rows[row] & (uint8_t)(0x10U >> column)) != 0) {
                        ++column;
                    }
                    if (run_start < column) {
                        err = fill_rect_clipped_unlocked(
                            (int)(cursor_x + (int64_t)run_start * scale),
                            (int)(cursor_y + (int64_t)row * scale),
                            (column - run_start) * (int)scale, (int)scale,
                            foreground);
                    }
                }
            }
        }
        cursor_x += (int64_t)scale * 6;
    }

    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t ili9488_display_draw_test_pattern(void)
{
    static const uint8_t bars[][3] = {
        {0xFC, 0x00, 0x00},
        {0x00, 0xFC, 0x00},
        {0x00, 0x00, 0xFC},
        {0xFC, 0xFC, 0xFC},
        {0x00, 0x00, 0x00},
    };

    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = take_display_lock();
    if (err != ESP_OK) {
        return err;
    }

    ESP_LOGI(TAG, "Drawing RED | GREEN | BLUE | WHITE | BLACK bars");
    set_window(0, 0, s_width, s_height);
    for (int row = 0; row < s_height; ++row) {
        for (int column = 0; column < s_width; ++column) {
            unsigned bar = ((unsigned)column * 5U) / s_width;
            write_pixel(bars[bar][0], bars[bar][1], bars[bar][2]);
        }
        if ((row % 80) == 79) {
            ESP_LOGI(TAG, "Color bars progress: %d/%u rows", row + 1, s_height);
        }
        service_idle_task(row);
    }
    xSemaphoreGive(s_lock);
    ESP_LOGI(TAG, "Color bars draw complete");
    return ESP_OK;
}
