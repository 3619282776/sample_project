#pragma once

#include <stdint.h>
#include "esp_err.h"

// TFT 屏幕示波器显示：波形 + FFT 频谱（复刻 tools/pc_waveform.py 的前两图）。
// 数据来自 audio_task 的 mono 音频流；内部用 StreamBuffer 衔接，非阻塞投递。

// 初始化：创建流缓冲、分配 RGB565 帧缓冲、预计算 Hann 窗。须在 mic 采集前调用。
esp_err_t scope_display_init(void);

// 显示任务循环（供 xTaskCreate 使用）：取 mono 块 -> 更新波形 -> FFT -> 重绘两图。
void scope_display_task(void *arg);

// audio_task 调用：把 mono 音频块送入显示。frames 为样本数。
// 非阻塞（缓冲满时丢弃该块，绝不拖慢实时音频）。返回实际写入字节数。
int scope_display_feed(const int16_t *mono, int frames);