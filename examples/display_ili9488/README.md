# ILI9488 独立显示示例

该示例只调用 `main/display/display_ili9488.c/.h` 的基础接口，不包含也不调用
`display_app.c/.h` 中的 UI 函数。

运行顺序：

1. 初始化 ILI9488 和 PWM 背光，默认亮度设为 40%。
2. 绘制红、绿、蓝、白、黑五色测试画面。
3. 停留 1 秒后切换为 `320 x 480` 竖屏。
4. 绘制蓝色标题栏、白色内容区、3 px 蓝色外框和十字分割线。
5. 通过 UART0 输入 `0~100` 并回车，将背光设置为对应百分比。

## 临时切换方法

将本目录的 `main.c` 内容复制到 `main/main.c`，并将
`main/CMakeLists.txt` 的源文件保持为：

```cmake
idf_component_register(
    SRCS "main.c"
         "display/display_ili9488.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_gpio esp_driver_ledc hal esp_rom
)
```

示例复用项目现有的显示引脚和时序配置，不需要修改接线。测试完成后再恢复原来的
`main/main.c` 和 `main/CMakeLists.txt`。示例目录本身不会参与当前固件编译。
