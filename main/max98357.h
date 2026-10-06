#pragma once

#include <stdint.h>
#include "esp_err.h"

// 功放 I2S 引脚（MAX98357A）
#define AMP_BCK_GPIO   26
#define AMP_WS_GPIO    25
#define AMP_DIN_GPIO   27
#define AMP_SD_GPIO    32   // 使能脚，设为 -1 表示不使用（常开）

// 初始化功放 I2S（立体声 TX，内部把单声道复制到左右声道）
esp_err_t amp_i2s_init(void);

// 写入单声道 16bit 样本（内部复制为左右声道）
esp_err_t amp_write(const int16_t *mono, int samples, uint32_t timeout_ms);
