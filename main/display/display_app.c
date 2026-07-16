/*
 * 功能：组织竖屏首页、2~4 格布局、状态栏、触摸页面和返回导航。
 * 配置：320x480 竖屏；蓝白主题；外框/分割线 3 px；触摸驱动尚未接入。
 * 依赖：只通过 display_ili9488.h 的基础绘图接口操作屏幕。
 */

#include "display/display_app.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "display/display_ili9488.h"
#include "esp_log.h"

#define HOME_LINE_THICKNESS 3
#define HOME_COLOR_BLUE ILI9488_RGB565(45, 120, 195)

typedef struct {
    int x;
    int y;
    int width;
    int height;
} display_app_rect_t;

/* 两个矩形是所有页面共享的固定区域，避免各页面重复维护坐标。 */
static const display_app_rect_t s_status_rect = {
    .x = 8,
    .y = 8,
    .width = 304,
    .height = 48,
};
static const display_app_rect_t s_content_rect = {
    .x = 8,
    .y = 64,
    .width = 304,
    .height = 408,
};

static const char *TAG = "display_app";
static char s_time_text[6] = "--:--";
static int s_battery_percent = -1;
static display_app_page_t s_current_page = DISPLAY_APP_PAGE_NONE;

static esp_err_t draw_thick_frame(const display_app_rect_t *rect,
                                  int thickness, uint16_t color)
{
    esp_err_t err = ili9488_display_fill_rect(
        rect->x, rect->y, rect->width, thickness, color);
    if (err == ESP_OK) {
        err = ili9488_display_fill_rect(
            rect->x, rect->y + rect->height - thickness,
            rect->width, thickness, color);
    }
    if (err == ESP_OK) {
        err = ili9488_display_fill_rect(
            rect->x, rect->y + thickness, thickness,
            rect->height - 2 * thickness, color);
    }
    if (err == ESP_OK) {
        err = ili9488_display_fill_rect(
            rect->x + rect->width - thickness, rect->y + thickness,
            thickness, rect->height - 2 * thickness, color);
    }
    return err;
}

static bool grid_cell_rect(unsigned box_count, unsigned index,
                           display_app_rect_t *cell)
{
    if (cell == NULL || box_count < 2 || box_count > 4 ||
        index >= box_count) {
        return false;
    }

    const int left = s_content_rect.x + HOME_LINE_THICKNESS;
    const int top = s_content_rect.y + HOME_LINE_THICKNESS;
    const int right = s_content_rect.x + s_content_rect.width -
                      HOME_LINE_THICKNESS;
    const int bottom = s_content_rect.y + s_content_rect.height -
                       HOME_LINE_THICKNESS;
    const int split_x = s_content_rect.x + s_content_rect.width / 2;
    const int split_y = s_content_rect.y + s_content_rect.height / 2;
    const int split_left = split_x - HOME_LINE_THICKNESS / 2;
    const int split_right = split_left + HOME_LINE_THICKNESS;
    const int split_top = split_y - HOME_LINE_THICKNESS / 2;
    const int split_bottom = split_top + HOME_LINE_THICKNESS;

    /* 2 格上下排列；3 格为上 1 下 2；4 格为标准四象限。 */
    if (box_count == 2) {
        cell->x = left;
        cell->width = right - left;
        cell->y = index == 0 ? top : split_bottom;
        cell->height = index == 0 ? split_top - top : bottom - split_bottom;
        return true;
    }

    if (box_count == 3 && index == 0) {
        *cell = (display_app_rect_t){
            .x = left,
            .y = top,
            .width = right - left,
            .height = split_top - top,
        };
        return true;
    }

    const unsigned lower_index = box_count == 3 ? index - 1U : index;
    const int column = (int)(lower_index % 2U);
    const int row = box_count == 3 ? 1 : (int)(lower_index / 2U);
    cell->x = column == 0 ? left : split_right;
    cell->width = column == 0 ? split_left - left : right - split_right;
    cell->y = row == 0 ? top : split_bottom;
    cell->height = row == 0 ? split_top - top : bottom - split_bottom;
    return true;
}

static esp_err_t render_box_grid(unsigned box_count,
                                 const char *const labels[])
{
    if (box_count < 2 || box_count > 4 || labels == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const int split_x = s_content_rect.x + s_content_rect.width / 2 -
                        HOME_LINE_THICKNESS / 2;
    const int split_y = s_content_rect.y + s_content_rect.height / 2 -
                        HOME_LINE_THICKNESS / 2;

    /* 每种布局只画一个外框和共享线，不重复描绘相邻格子的边。 */
    esp_err_t err = draw_thick_frame(&s_content_rect, HOME_LINE_THICKNESS,
                                     HOME_COLOR_BLUE);
    if (err == ESP_OK) {
        err = ili9488_display_fill_rect(
            s_content_rect.x, split_y, s_content_rect.width,
            HOME_LINE_THICKNESS, HOME_COLOR_BLUE);
    }
    if (err == ESP_OK && box_count == 3) {
        err = ili9488_display_fill_rect(
            split_x, split_y, HOME_LINE_THICKNESS,
            s_content_rect.y + s_content_rect.height - split_y,
            HOME_COLOR_BLUE);
    } else if (err == ESP_OK && box_count == 4) {
        err = ili9488_display_fill_rect(
            split_x, s_content_rect.y, HOME_LINE_THICKNESS,
            s_content_rect.height, HOME_COLOR_BLUE);
    }

    for (unsigned index = 0; index < box_count && err == ESP_OK; ++index) {
        display_app_rect_t cell;
        if (!grid_cell_rect(box_count, index, &cell)) {
            return ESP_ERR_INVALID_STATE;
        }
        const size_t label_length = strlen(labels[index]);
        const int label_width =
            label_length == 0 ? 0 : (int)(label_length * 12U - 2U);
        err = ili9488_display_draw_text(
            cell.x + (cell.width - label_width) / 2,
            cell.y + (cell.height - 14) / 2, labels[index], 2,
            ILI9488_COLOR_BLACK, ILI9488_COLOR_WHITE, true);
    }
    return err;
}

static esp_err_t clear_for_page_change(void)
{
    /* 色条或首次启动可能污染全屏；普通页面切换只清内容区以提速。 */
    if (s_current_page == DISPLAY_APP_PAGE_NONE ||
        s_current_page == DISPLAY_APP_PAGE_COLOR_BARS) {
        return ili9488_display_fill(ILI9488_COLOR_WHITE);
    }
    return ili9488_display_fill_rect(
        s_content_rect.x, s_content_rect.y, s_content_rect.width,
        s_content_rect.height, ILI9488_COLOR_WHITE);
}

static esp_err_t render_status_bar(void)
{
    char battery_text[16];
    if (s_battery_percent < 0) {
        snprintf(battery_text, sizeof(battery_text), "BAT --%%");
    } else {
        snprintf(battery_text, sizeof(battery_text), "BAT %d%%",
                 s_battery_percent);
    }

    esp_err_t err = ili9488_display_fill_rect(
        s_status_rect.x, s_status_rect.y, s_status_rect.width,
        s_status_rect.height,
        HOME_COLOR_BLUE);
    if (err == ESP_OK) {
        err = ili9488_display_draw_text(
            s_status_rect.x + 8, s_status_rect.y + 17, "TIME", 2,
            ILI9488_COLOR_WHITE, HOME_COLOR_BLUE, true);
    }
    if (err == ESP_OK) {
        err = ili9488_display_draw_text(
            s_status_rect.x + 68, s_status_rect.y + 17, s_time_text, 2,
            ILI9488_COLOR_WHITE, HOME_COLOR_BLUE, true);
    }
    if (err == ESP_OK) {
        err = ili9488_display_draw_text(
            s_status_rect.x + 208, s_status_rect.y + 17, battery_text, 2,
            ILI9488_COLOR_WHITE, HOME_COLOR_BLUE, true);
    }
    return err;
}

static esp_err_t render_back_bar(uint8_t cell)
{
    char page_text[16];
    snprintf(page_text, sizeof(page_text), "PAGE %u", cell);

    esp_err_t err = ili9488_display_fill_rect(
        s_status_rect.x, s_status_rect.y, s_status_rect.width,
        s_status_rect.height,
        HOME_COLOR_BLUE);
    if (err == ESP_OK) {
        err = ili9488_display_draw_text(
            s_status_rect.x + 16, s_status_rect.y + 17, "BACK", 2,
            ILI9488_COLOR_WHITE, HOME_COLOR_BLUE, true);
    }
    if (err == ESP_OK) {
        err = ili9488_display_draw_text(
            s_status_rect.x + 216, s_status_rect.y + 17, page_text, 2,
            ILI9488_COLOR_WHITE, HOME_COLOR_BLUE, true);
    }
    return err;
}

static esp_err_t render_touch_page_content(uint8_t cell)
{
    char title[16];
    snprintf(title, sizeof(title), "TOUCH %u", cell);
    esp_err_t err = draw_thick_frame(&s_content_rect, HOME_LINE_THICKNESS,
                                     HOME_COLOR_BLUE);
    if (err == ESP_OK) {
        const size_t length = strlen(title);
        const int width = (int)(length * 12U - 2U);
        err = ili9488_display_draw_text(
            s_content_rect.x + (s_content_rect.width - width) / 2,
            s_content_rect.y + (s_content_rect.height - 14) / 2,
            title, 2, ILI9488_COLOR_BLACK, ILI9488_COLOR_WHITE, true);
    }
    return err;
}

esp_err_t display_app_show_home(void)
{
    static const char *labels[] = {
        "TOUCH 1", "TOUCH 2", "TOUCH 3", "TOUCH 4",
    };
    if (!ili9488_display_is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = ili9488_display_set_rotation(
        ILI9488_DISPLAY_ROTATION_0);
    if (err == ESP_OK) {
        err = clear_for_page_change();
    }
    if (err == ESP_OK) {
        err = render_status_bar();
    }
    if (err == ESP_OK) {
        err = render_box_grid(4, labels);
    }
    if (err == ESP_OK) {
        /* 全部绘制成功后再提交页面状态，避免触摸进入半绘制页面。 */
        s_current_page = DISPLAY_APP_PAGE_HOME;
    }
    return err;
}

esp_err_t display_app_show_touch_page(uint8_t cell)
{
    if (!ili9488_display_is_initialized()) {
        return ESP_ERR_INVALID_STATE;
    }
    if (cell < 1 || cell > 4) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = clear_for_page_change();
    if (err == ESP_OK) {
        err = render_back_bar(cell);
    }
    if (err == ESP_OK) {
        err = render_touch_page_content(cell);
    }
    if (err == ESP_OK) {
        s_current_page =
            (display_app_page_t)(DISPLAY_APP_PAGE_TOUCH_1 + cell - 1);
    }
    return err;
}

esp_err_t display_app_show_color_bars(void)
{
    esp_err_t err = ili9488_display_draw_test_pattern();
    if (err == ESP_OK) {
        s_current_page = DISPLAY_APP_PAGE_COLOR_BARS;
    }
    return err;
}

display_app_page_t display_app_current_page(void)
{
    return s_current_page;
}

esp_err_t display_app_update_status(const char *time_text,
                                    int battery_percent)
{
    if (battery_percent < -1 || battery_percent > 100) {
        return ESP_ERR_INVALID_ARG;
    }
    if (time_text != NULL) {
        const size_t length = strlen(time_text);
        if (length == 0 || length >= sizeof(s_time_text)) {
            return ESP_ERR_INVALID_ARG;
        }
        memcpy(s_time_text, time_text, length + 1U);
    }
    s_battery_percent = battery_percent;

    /* 子页面顶部是返回键，只缓存状态值，回到主页时再显示。 */
    if (s_current_page == DISPLAY_APP_PAGE_HOME &&
        ili9488_display_is_initialized()) {
        return render_status_bar();
    }
    return ESP_OK;
}

int display_app_hit_test_touch_grid(int x, int y)
{
    /* cell 矩形已排除 3 px 外框和分割线，线上的触摸返回 0。 */
    for (unsigned index = 0; index < 4; ++index) {
        display_app_rect_t cell;
        if (grid_cell_rect(4, index, &cell) &&
            x >= cell.x && x < cell.x + cell.width &&
            y >= cell.y && y < cell.y + cell.height) {
            return (int)index + 1;
        }
    }
    return 0;
}

esp_err_t display_app_handle_touch(int x, int y)
{
    /* 接收一次点击坐标；不处理触摸芯片的按下、抬起和消抖。 */
    if (s_current_page == DISPLAY_APP_PAGE_HOME) {
        const int cell = display_app_hit_test_touch_grid(x, y);
        return cell == 0 ? ESP_OK : display_app_show_touch_page((uint8_t)cell);
    }

    if (s_current_page >= DISPLAY_APP_PAGE_TOUCH_1 &&
        s_current_page <= DISPLAY_APP_PAGE_TOUCH_4 &&
        x >= s_status_rect.x &&
        x < s_status_rect.x + s_status_rect.width &&
        y >= s_status_rect.y &&
        y < s_status_rect.y + s_status_rect.height) {
        return display_app_show_home();
    }
    return ESP_OK;
}

esp_err_t display_app_init(void)
{
    esp_err_t err = ili9488_display_init();
    if (err != ESP_OK) {
        return err;
    }

    err = display_app_show_home();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Portrait home ready: %ux%u",
                 ili9488_display_width(), ili9488_display_height());
    } else {
        ESP_LOGE(TAG, "Home render failed: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t display_app_deinit(void)
{
    s_current_page = DISPLAY_APP_PAGE_NONE;
    esp_err_t err = ili9488_display_set_backlight(false);
    if (err != ESP_OK && err != ESP_ERR_NOT_SUPPORTED) {
        return err;
    }
    return ili9488_display_deinit();
}
