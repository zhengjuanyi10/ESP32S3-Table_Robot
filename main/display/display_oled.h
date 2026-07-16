/**
 * @file display_oled.h
 * @brief 1.3寸 I2C OLED 显示屏驱动（SH1106 128x64）
 *
 * 引脚: SDA→GPIO8, SCL→GPIO9
 */

#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 小表情（24x24） */
typedef enum {
    FACE_SMILE,
    FACE_SAD,
    FACE_WINK,
    FACE_SURPRISE,
    FACE_NEUTRAL,
} face_expr_t;

/** 大表情（64x64，自动居中） */
typedef enum {
    BIGFACE_SMILE,
    BIGFACE_LAUGH,
    BIGFACE_SAD,
    BIGFACE_WINK,
    BIGFACE_SURPRISE,
} bigface_expr_t;

esp_err_t display_init(void);
void display_clear(void);
void display_print(int x, int y, const char *text);
void display_face(int x, int y, face_expr_t expr);
void display_face_big(bigface_expr_t expr);
void display_flush(void);
void display_status(const char *line1, const char *line2);

#ifdef __cplusplus
}
#endif
