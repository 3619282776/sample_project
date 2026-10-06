#pragma once

#include <stdint.h>
#include "esp_err.h"

// 麦克风 I2S 引脚（两片 INMP441 并联：BCK/WS 共用，两片 DOUT 并联到 DIN，构成立体声）
#define I2S_BCK_GPIO   14
#define I2S_WS_GPIO    15
#define I2S_DIN_GPIO   22

// 初始化麦克风 I2S（立体声 RX，主模式）
esp_err_t mic_i2s_init(void);

// 读取一块立体声数据
// interleaved: 输出缓冲，长度 >= 2*max_frames，内容为 L,R,L,R...（24bit 有符号，已右移对齐）
// 返回实际读到的帧数；<0 表示出错
int mic_i2s_read(int32_t *interleaved, int max_frames, uint32_t timeout_ms);

// 由 L/R 窗口估计方位角（度），返回 -90..+90（0=正前方，正=右侧，负=左侧）
float doa_estimate_angle(const int32_t *l, const int32_t *r, int n);
