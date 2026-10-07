# sample_project

基于 ESP-IDF 的 ESP32 音频可视化项目：双麦克风拾音 + 声源方位估计（DOA）、实时波形 / FFT 频谱示波、OLED 信息显示，以及串口波形透传到电脑。

<!--
  图片使用说明：
  - 本 README 已为关键位置预留图片占位，直接替换下方的 `![...](路径)` 即可。
  - 建议把图片统一放到项目内的 `docs/images/`（或 `assets/`）目录，再填写相对路径。
  - 示例：`![硬件连接示意图](docs/images/hardware.png)`
-->

## 功能特性

- **双麦克风拾音**：两片 INMP441 并联构成 I2S 立体声输入，16 kHz 采样。
- **声源方位估计（DOA）**：基于左右声道互相关时延估计方位角（-90° ~ +90°）。
- **实时回放**：MAX98357A I2S DAC 功放，麦克风信号直通输出。
- **TFT 示波器**：2.8 寸 ST7789V（驱动沿用 ILI9341 命名）横屏显示，波形 + FFT 频谱，短按按键切换到「收音方向」画面。
- **双 OLED 显示**：两块 SSD1306 走两条独立 I2C 总线，一块显示同步时钟、一块显示 FFT 峰值频率。
- **串口波形透传**：UART0（921600）把音频样本打包成帧发给电脑，配合 `tools/pc_waveform.py` 实时绘图。
- **时间同步**：电脑端每秒下发系统时间，显示到 OLED。

## 整体架构

<!-- ⬇️ 图片占位：系统架构图 / 数据流图，替换为实际图片路径 -->
![系统架构图](docs/images/architecture.png)

```
INMP441 双麦克风 ──I2S──> 采集/DOA 任务
                              │
                              ├─> MAX98357A ──> 扬声器（直通回放）
                              ├─> ST7789V TFT ──> 波形/FFT/方向画面
                              ├─> SSD1306 OLED x2 ──> 时钟 / FFT 峰值
                              └─> UART0 ──> pc_waveform.py（电脑端绘图）
```

## 硬件组成

| 模块 | 型号 | 数量 | 说明 |
|------|------|------|------|
| 主控 | ESP32 | 1 | ESP-IDF 5.3.4 |
| 麦克风 | INMP441 | 2 | I2S 立体声，L/R 并联 |
| 功放 | MAX98357A | 1 | I2S DAC + 功放 |
| 屏幕 | ST7789V 2.8" 240×320 | 1 | TFT 示波器 |
| 屏幕 | SSD1306 128×64 | 2 | OLED 信息显示 |
| 按键 | 轻触开关 | 1 | 切换示波器 / 方向画面 |

<!-- ⬇️ 图片占位：硬件实物 / 连接示意图，替换为实际图片路径 -->
![硬件连接示意图](docs/images/hardware.png)

## 引脚接线

### INMP441 麦克风（两片并联）

| 信号 | GPIO |
|------|------|
| BCK | 14 |
| WS（L/R） | 15 |
| DOUT（两片并联到 DIN） | 22 |

> 两片 INMP441 的 BCK / WS 共用，DOUT 一起接到 GPIO22；通过 L/R 引脚分别接 GND / VDD 区分左右声道。

### MAX98357A 功放

| 信号 | GPIO |
|------|------|
| BCK | 26 |
| WS | 25 |
| DIN | 27 |
| SD（使能，可选） | 32 |

### ST7789V TFT（驱动沿用 ILI9341 命名）

| 信号 | GPIO |
|------|------|
| SCLK | 18 |
| MOSI | 23 |
| CS | 5 |
| DC | 4 |
| RST | 未接 MCU（接 VCC） |
| BL（背光） | 2 |

### SSD1306 OLED（两条独立 I2C 总线）

| 屏幕 | SDA | SCL | I2C 总线 |
|------|-----|-----|----------|
| OLED-0（时钟） | 13 | 16 | I2C_NUM_0 |
| OLED-1（FFT 峰值） | 17 | 19 | I2C_NUM_1 |

### 按键

| 引脚 | 说明 |
|------|------|
| GPIO21 ↔ GND | 短按切换示波器 / 方向画面（低电平有效，内部上拉） |

## 目录结构

```
.
├── main/
│   ├── main.c            # 主入口与任务编排
│   ├── audio_config.h    # 音频 / DOA 全局参数
│   ├── inmp441_doa.c/.h  # INMP441 采集 + DOA 方位估计
│   ├── max98357.c/.h     # MAX98357A 功放回放
│   ├── ili9341.c/.h      # TFT (ST7789V) 驱动与绘图
│   ├── scope_display.c/.h# TFT 示波器（波形 + FFT + 方向）
│   ├── oled_display.c/.h # 双 SSD1306 OLED
│   ├── uart_stream.c/.h  # UART 波形透传
│   ├── time_sync.c/.h    # 时间同步
│   ├── idf_component.yml # 组件依赖清单
│   └── CMakeLists.txt
├── tools/
│   └── pc_waveform.py    # 电脑端实时波形 / 频谱显示
├── CMakeLists.txt
├── sdkconfig
└── dependencies.lock
```

## 环境要求

- **ESP-IDF**：5.x（在 `main/idf_component.yml` 中声明 `idf >= 5.0`）
- **目标芯片**：`esp32`
- **依赖组件**：`espressif/ssd1306 ^1.0.5~1`（由组件管理器自动拉取）

## 构建与烧录

```bash
# 设置目标芯片（首次）
idf.py set-target esp32

# 编译
idf.py build

# 烧录并打开串口监视器
idf.py -p COMx flash monitor
```

> `COMx` 替换为实际串口号（默认 921600 波特率）。

## 使用方法

1. 上电后，两块 OLED 分别显示自己的编号和接线，便于核对 SDA/SCL。
2. TFT 默认显示波形 + FFT 频谱示波器；短按 GPIO21 按钮切换到「收音方向」画面，再按切回。
3. OLED-0 在收到电脑时间后自动切换为实时时钟显示。

### 电脑端波形显示

```bash
cd tools
pip install pyserial numpy pyqtgraph pyqt5

python pc_waveform.py                 # 自动枚举串口
python pc_waveform.py COM5            # 指定串口（921600 / 16 kHz）
python pc_waveform.py --selftest      # 编解码自测（无需硬件/串口）
```

<!-- ⬇️ 图片占位：电脑端 pc_waveform.py 界面截图，替换为实际图片路径 -->
![电脑端波形显示界面](docs/images/pc_waveform.png)

## 显示界面

<!-- ⬇️ 图片占位：TFT 示波器（波形+FFT）截图，替换为实际图片路径 -->
![TFT 示波器界面](docs/images/scope.png)

<!-- ⬇️ 图片占位：TFT 方向画面截图，替换为实际图片路径 -->
![TFT 收音方向界面](docs/images/direction.png)

<!-- ⬇️ 图片占位：OLED 显示效果截图，替换为实际图片路径 -->
![OLED 显示效果](docs/images/oled.png)

## 串口协议（简要）

- **上行波形帧**（ESP32 → PC）：`[0xAA 0x55][channels u8][frames u16][int16 样本 ...][checksum u8]`，小端。
- **下行时间帧**（PC → ESP32）：`[0x55 0xAA][cmd=0x01][year u16][month][day][hour][min][sec][checksum]`，共 11 字节小端。

协议实现详见 `main/uart_stream.c`、`main/time_sync.c` 与 `tools/pc_waveform.py`。

## 参数配置

采样率、块大小、DOA 参数（麦克风间距、静音门限等）与回放缓冲大小，集中在 `main/audio_config.h`。麦克风间距 `MIC_SPACING_M` 需按实际安装距离调整。