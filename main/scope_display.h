#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

// TFT 屏幕显示：默认「波形 + FFT 频谱」示波器（复刻 tools/pc_waveform.py 的前两图），
// 短按 GPIO21 切到「收音方向（DOA 方位角）」画面，再按切回。
// 数据来自 audio_task 的 mono 音频流；内部用 StreamBuffer 衔接，非阻塞投递。
//
// 按键接线：一脚接 GPIO21、另一脚接 GND（低电平有效，内部上拉，无需外部电阻）。

// 初始化：创建流缓冲、分配 RGB565 帧缓冲、预计算 Hann 窗、配置按键 GPIO。
// 须在 mic 采集前调用。
esp_err_t scope_display_init(void);

// 显示任务循环（供 xTaskCreate 使用）：
//   示波器模式：取 mono 块 -> 更新波形 -> FFT -> 重绘两图；
//   方向模式：按需重绘方位角画面。循环内轮询按键，短按切换。
void scope_display_task(void *arg);

// audio_task 调用：把 mono 音频块送入显示。frames 为样本数。
// 非阻塞（缓冲满时丢弃该块，绝不拖慢实时音频）。返回实际写入字节数。
int scope_display_feed(const int16_t *mono, int frames);

// doa_task 调用：上报当前声源方位角（度，-90..+90，0=正前方，正=右侧，负=左侧）。
// 「收音方向」画面据此实时更新。
void scope_display_set_angle(float angle_deg);

// 当前是否处于「收音方向」画面。功放回放据此决定是否出声（仅方向模式出声）。
bool scope_display_is_doa(void);