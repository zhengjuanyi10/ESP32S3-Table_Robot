# 显示与 UI 模块

显示代码只分两层：

```text
app_main()
  -> display_app.c/.h          竖屏首页、状态栏、页面布局、触摸区域
       -> display_ili9488.c/.h 面板驱动、背光和全部基础绘图
```

`app_main()` 只调用 `display_app_init()`。以后新增页面或调整首页布局放在
`display_app.c`；像素、线、矩形、圆、文字和图片等基础能力继续放在
`display_ili9488.c`。

## 当前竖屏首页

初始化后使用 `320 x 480` 竖屏方向：

- 首页统一使用蓝白配色。
- 顶部 `304 x 48` 状态框使用中等浅蓝底、白字显示时间和电量。
- 下部使用白底黑字，外框和共享分割线统一为 `3 px` 蓝线。
- 通用区域布局已支持 2、3、4 格；首页使用四格十字布局。
- 进入任意触摸预留页后，顶部状态栏会替换为整条 `BACK` 返回键。
- 尚未接入 RTC/SNTP 和电量采集时显示 `TIME --:--`、`BAT --%`。
- 当前不初始化触摸控制器，只保留四格布局和坐标命中函数。

状态数据接入后可直接更新顶部区域：

```c
#include "display/display_app.h"

ESP_ERROR_CHECK(display_app_update_status("14:35", 86));
```

触摸驱动接入后，将一次点击的竖屏坐标交给统一入口：

```c
ESP_ERROR_CHECK(display_app_handle_touch(touch_x, touch_y));
```

主页点击四格会进入对应预留页；子页面点击顶部蓝色区域会返回主页。
如只需查询主页格子，可调用 `display_app_hit_test_touch_grid()`，返回
`1..4`，返回 `0` 表示坐标位于外框或加粗分割线上。

## 基础绘图

需要在 UI 页面中绘图时包含：

```c
#include "display/display_ili9488.h"

ili9488_display_fill(ILI9488_COLOR_BLACK);
ili9488_display_fill_rect(10, 10, 100, 40, ILI9488_COLOR_NAVY);
ili9488_display_draw_rect(10, 10, 100, 40, ILI9488_COLOR_CYAN);
ili9488_display_draw_text(18, 22, "HELLO", 2,
                          ILI9488_COLOR_WHITE,
                          ILI9488_COLOR_NAVY, true);
```

驱动当前提供：

- RGB565 常用颜色与 `ILI9488_RGB565()`。
- 清屏、像素、横线、竖线、任意直线。
- 空心/实心矩形、空心/实心圆。
- RGB565、RGB888、单色位图。
- 内置 5x7 大写 ASCII 文字，可设置整数倍缩放。
- 屏幕旋转、尺寸查询、背光开关、反初始化。

绘图调用为同步调用并自动裁剪到屏幕范围，函数返回后即可复用图片缓冲区。

## 当前接线

| 信号 | ESP32-S3 GPIO |
| --- | ---: |
| SCLK | 4 |
| MOSI | 5 |
| CS | 6 |
| DC | 7 |
| LCD RESET | 10 |
| 背光控制 | 11（接外部三极管/MOSFET驱动） |
| 触摸 RESET | 12（当前不驱动） |
| 触摸 INT | 13（当前不读取） |

引脚在 `menuconfig -> Table Robot -> ILI9488 display` 中配置。背光由
`ili9488_display_set_backlight()` 控制，后续定时熄屏任务直接调用该接口。
默认背光亮度为 `40%`，也可调用
`ili9488_display_set_brightness(0..100)` 动态调节。
`ILI9488 GPIO clock half-cycle delay` 默认为 `0`，用于快速绘制；只有长线
或干扰导致花屏时才改成 `1 us` 的保守波形。
