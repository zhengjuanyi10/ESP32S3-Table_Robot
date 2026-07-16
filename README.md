# Table Robot

基于 ESP32-S3 和 ESP-IDF 的桌面机器人固件项目。

## 开发环境

- ESP-IDF 5.5.4
- 目标芯片：`esp32s3`
- Flash：32 MB
- PSRAM：16 MB

## 获取代码

```powershell
git clone https://github.com/zhengjuanyi10/ESP32S3-Table_Robot.git
cd ESP32S3-Table_Robot
```

安装并激活 ESP-IDF 环境后，可以使用以下命令配置、编译和烧录：

```powershell
idf.py set-target esp32s3
idf.py build
idf.py -p COMx flash monitor
```

将 `COMx` 替换为开发板实际使用的串口。串口监视器中按 `Ctrl+]` 退出。

## 工程结构

```text
.
├── CMakeLists.txt
├── sdkconfig.defaults       # 默认项目配置
├── examples/
│   └── display_ili9488/    # ILI9488 独立显示示例
└── main/
    ├── CMakeLists.txt
    ├── Kconfig.projbuild
    ├── main.c               # 应用入口
    ├── actions/             # 动作模块（预留）
    ├── button/              # 按键模块（预留）
    ├── display/             # ILI9488 驱动与显示应用
    ├── sensors/             # 传感器模块
    └── servo/               # 舵机模块（预留）
```

## 模块清单

| 模块 | 路径 | 功能 | 当前状态 |
| --- | --- | --- | --- |
| 应用入口 | `main/main.c` | 系统初始化与模块调度 | 已提交 |
| I2C 总线 | `main/sensors/i2c_bus.*` | 初始化并管理传感器共用的 I2C 总线 | 已提交 |
| 实时时钟 | `main/sensors/ds3231.*` | 日期与时间读取 | 已提交 |
| 电源监测 | `main/sensors/ina219.*` | 电压、电流和功率数据读取 | 已提交 |
| 温湿度监测 | `main/sensors/sht30.*` | 温度与湿度数据读取 | 已提交 |
| 距离监测 | `main/sensors/tof200.*` | UART 测距数据读取 | 已提交 |
| 动作控制 | `main/actions/` | 组合动作与动作编排 | 预留 |
| 按键输入 | `main/button/` | 按键检测与输入事件处理 | 预留 |
| 显示输出 | `main/display/` | ILI9488 驱动、基础绘图、背光和页面布局 | 已提交 |
| 显示示例 | `examples/display_ili9488/` | 独立显示与背光测试 | 已提交 |
| 舵机控制 | `main/servo/` | 舵机通信与运动控制 | 预留 |
| 项目配置 | `sdkconfig.defaults`、`main/Kconfig.projbuild` | 默认参数与可配置选项 | 已提交 |

## 协作约定

- `main` 分支保存稳定代码。
- 新功能在独立分支开发，并通过 Pull Request 合并。
- 不提交 `build/`、`sdkconfig`、`.vscode/` 等本地生成或个人配置文件。
- `actions`、`button` 和 `servo` 目录当前仅保留目录结构，代码将在相应模块整理完成后提交。

## 当前状态

仓库目前包含基础工程配置、传感器模块和 ILI9488 显示模块。动作、按键和舵机模块仍为预留目录。
