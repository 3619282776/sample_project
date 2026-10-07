#pragma once

#include "esp_err.h"

// 启动 UART0 RX 时间同步任务：解析电脑端下发的日期/时间帧，刷新到 OLED 屏 0。
//
// 帧协议（PC -> ESP32，小端，与上行波形帧的 0xAA 0x55 相反）：
//   [0x55 0xAA][cmd=0x01][year u16][month u8][day u8][hour u8][min u8][sec u8][checksum u8]  共 11 字节
//   checksum = cmd .. sec 各字节之和 mod 256
esp_err_t time_sync_start(void);