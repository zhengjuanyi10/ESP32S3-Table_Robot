/*
 * 功能：声明显示应用入口、页面切换、状态更新和触摸分发接口。
 * 配置：页面尺寸与配色由 display_app.c 管理，基础绘图由 ILI9488 驱动提供。
 */

#pragma once

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    DISPLAY_APP_PAGE_NONE = 0,
    DISPLAY_APP_PAGE_HOME,
    DISPLAY_APP_PAGE_TOUCH_1,
    DISPLAY_APP_PAGE_TOUCH_2,
    DISPLAY_APP_PAGE_TOUCH_3,
    DISPLAY_APP_PAGE_TOUCH_4,
    DISPLAY_APP_PAGE_COLOR_BARS,
} display_app_page_t;

/* Initializes the panel and renders the portrait home page. */
esp_err_t display_app_init(void);
esp_err_t display_app_deinit(void);

/* Page entry points. */
esp_err_t display_app_show_home(void);
esp_err_t display_app_show_color_bars(void);
esp_err_t display_app_show_touch_page(uint8_t cell);
display_app_page_t display_app_current_page(void);

/*
 * Updates the home-page status bar. time_text is up to five characters
 * (normally HH:MM); NULL keeps the previous time. battery_percent accepts
 * 0..100, or -1 when battery data is not available.
 */
esp_err_t display_app_update_status(const char *time_text,
                                    int battery_percent);

/* Returns the portrait home-page touch cell 1..4, or 0 outside all cells. */
int display_app_hit_test_touch_grid(int x, int y);

/* Reserved tap dispatcher: home cell enters a page; top bar returns home. */
esp_err_t display_app_handle_touch(int x, int y);

#ifdef __cplusplus
}
#endif
