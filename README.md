# Table Robot

基于 **ESP32-S3-DevKitC-1-N32R16V** 和 ESP-IDF 的桌面机器人固件。

## 当前基线

- ESP-IDF 5.5.4，目标芯片为 `esp32s3`
- 32 MB Octal Flash，OPI/DTR 80 MHz
- 16 MB Octal PSRAM，80 MHz，并接入 `malloc()`
- 启动程序会输出芯片、Flash、PSRAM 信息，并检查容量是否符合 N32R16V
- 已适配众灵 ZP 单线串口总线舵机：每次启动自动广播回中，并回读验证结果
- 暂用 ESP-IDF 默认分区表；确定 OTA、文件系统和资源需求后再设计 32 MB 分区

板载 Flash/PSRAM 参数来自乐鑫的
[ESP32-S3-DevKitC-1 用户指南](https://docs.espressif.com/projects/esp-dev-kits/en/latest/esp32s3/esp32-s3-devkitc-1/index.html)
和
[ESP32-S3-WROOM-2 数据手册](https://documentation.espressif.com/esp32-s3-wroom-2_datasheet_en.html)。

## 本机使用方法

当前电脑已安装 ESP-IDF，但系统 PowerShell 的脚本执行策略和 Python 版本会妨碍直接运行 `export.ps1`。根目录的 `idf.cmd` 会为单次命令激活正确环境，不更改系统策略。

```powershell
# 首次生成配置（本仓库已经执行过）
.\idf.cmd set-target esp32s3

# 编译
.\idf.cmd build

# 查看串口；也可以在设备管理器中确认
Get-CimInstance Win32_SerialPort | Select-Object DeviceID, Description

# 烧录并打开串口监视器，把 COMx 换成实际端口
.\idf.cmd -p COMx flash monitor
```

串口监视器中按 `Ctrl+]` 退出。

如果以后换了 ESP-IDF 安装位置，可先设置 `IDF_PATH`，或更新 `idf.cmd` 中的默认路径。请勿为了排查启动问题随意烧写 eFuse；官方 N32R16V 模组的 Octal Flash 配置应在出厂时正确设置。

## 众灵 ZP 总线舵机回中

当前实现使用厂家 ESP32 示例的默认引脚：`GPIO17 (TX)`、`GPIO16 (RX)`，而**不是**开发板丝印的 `TX/RX`（UART0，会与下载和日志共用）。如果使用众灵的“单线转双线 UART”转接板，接法为 `GPIO17 → 转接板 RX`、`GPIO16 ← 转接板 TX`；舵机总线接转接板的单线端。舵机电源正负极接独立电源，**舵机电源负极必须与 ESP32 GND 共地**。

刷入程序后，先让机器人关节有足够活动空间，再在开发板启动完成后**长按 BOOT 键约 1.5 秒**。程序会发送：

```text
#255P1500T5000!
```

其中 `255` 是广播 ID，`1500` 是逻辑中位，`5000` 表示用 5 秒缓慢移动。这个命令会同时作用于总线上的全部九个舵机，且不会修改它们的 ID、限位或校准值。

当前默认还会在每次启动后等待 2 秒，自动发送同一条回中命令。若以后不希望上电自动运动，可在 `idf.py menuconfig` 的 `Table Robot` 中关闭 `Center all ZP servos automatically after boot`。

自动回中后，程序会再等待约 5.5 秒，读取当前已配置的 ID `1–6、8、9` 的实际 PWM 值。日志会逐个输出 `CENTER OK` 或 `CENTER FAIL`；如果全部无应答，会明确提示检查 ZLink 供电、TX/RX 和共地。验证需要舵机回读线路（转接板 TX → GPIO16）正常，允许误差可在 `Table Robot` 菜单修改。

使用前务必确认 1500 对装配后的机构是安全位置；若某个关节的中位会顶死，请先拆下舵盘或在 `idf.py menuconfig` 的 `Table Robot` 菜单中调整回中值。不要对串联的九个舵机发送 `#255PCLE!`：它会把所有 ID 恢复为 `000`，造成总线冲突。

### 单个舵机串口控制

通过 `idf.py monitor` 连接到 USB 串口后，直接输入 ID 和相对角度，再按回车：

```text
1 40
```

这会读取 1 号舵机的当前位置，再按默认 270°顺时针模式让它正向移动 40°。`1 -40` 表示反向 40°。目标会被限制在厂家协议允许的 0–270°范围；若收不到位置反馈，程序不会运动，并提示检查 `GPIO16` 的 RX 接线、转接板和舵机 ID。

如果输入动作编号后只有日志、舵机没有实际运动，先输入：

```text
diagnose
```

程序会逐个读取 `1–6、8、9` 号舵机。只有 `DIAG OK` 才代表 ZLink 和舵机真实应答；“动作指令已发送”仅表示 ESP32 的 UART 发送成功，不能代替总线应答。

如果 `test` 也完全不运动，可输入：

```text
baudtest
```

该测试不依赖回读，只让 1 号舵机先以 115200、再以 9600 波特率小幅偏转并回中。观察在哪一段发生运动，然后输入 `baud 115200` 或 `baud 9600` 固定当前运行波特率。默认波特率也可在 `menuconfig -> Table Robot` 中设置。

### 分配舵机 ID

每次只连接一只仍为出厂 ID `0` 的舵机，在监视器输入：

```text
setid 1
```

程序会发送 `#000PID001!`，再读取新 ID `1` 验证。断电换下一只后依次执行 `setid 2` 到 `setid 9`。**绝不能在九只舵机同时串联时执行 `setid`**，否则会把所有 ID 改成同一个值。

### 逐个动作测试

自动回中完成且验证日志结束后，输入：

```text
test
```

程序会依次测试 ID `1–6、8、9`：每只从中位正向约 40°，再回到中位，然后才测试下一只。该测试不依赖位置回读，用于观察总线的逐 ID 发送控制。

## 工程结构

```text
.
├── CMakeLists.txt
├── sdkconfig.defaults   # 可复现的开发板配置
├── idf.cmd              # 本机 ESP-IDF 命令入口
└── main/
    ├── CMakeLists.txt
    ├── Kconfig.projbuild
    ├── main.c           # 应用入口与模块编排
    ├── actions/         # 上半身组合动作
    ├── button/          # 电源按键
    ├── sensors/         # I2C 总线与各数据读取模块
    └── servo/           # ZP 总线舵机驱动与调试控制台
```
