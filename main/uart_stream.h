#pragma once

#include <stdint.h>
#include "esp_err.h"

// 波形串口透传：把麦克风音频样本打包成带帧头的二进制帧，通过 UART0（USB 转串口）发给电脑。
// 电脑端用 tools/pc_waveform.py 接收并实时绘制波形。

// 初始化串口（UART0 / GPIO1 TX，921600 波特率）并把 ESP_LOG 重定向到同一串口驱动
esp_err_t uart_stream_init(void);

// 发送一帧波形数据。
// samples: 交错样本（通道连续排列：c0, c1, ..., c0, c1, ...）
// channels: 通道数（1 = 单声道 mono）
// frames: 每通道样本数
// 返回写入的字节数；<0 表示出错
int uart_stream_send(const int16_t *samples, uint8_t channels, int frames);