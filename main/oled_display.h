#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "ssd1306.h"     // espressif__ssd1306 组件提供的库函数

// 两块 SSD1306 走两条独立 I2C 总线，地址都保持默认 0x3C，互不冲突。
//   屏 0：SDA=GPIO13, SCL=GPIO16（I2C_NUM_0）
//   屏 1：SDA=GPIO17, SCL=GPIO19（I2C_NUM_1）
#define OLED_0   0
#define OLED_1   1

// 初始化两路 I2C 总线并创建两块屏（内部会清屏刷新一次，避免上电乱码）。
esp_err_t oled_display_init(void);

// 取某块屏的句柄（idx 取 OLED_0 / OLED_1），拿它可直接调用 ssd1306_xxx 库函数，
// 自定义绘图后记得调用 ssd1306_refresh_gram(dev) 才会真正刷新到屏幕。
ssd1306_handle_t oled_get(int idx);

// 便捷封装：清空指定屏幕并立即刷新
void oled_clear(int idx);

// 便捷封装：在 (x, y) 用 8x16 字体写一行文字并立即刷新
void oled_print(int idx, uint8_t x, uint8_t y, const char *s);

// 在指定屏显示时钟：顶部「星期 + 日期」小字居中，分隔线下方 HH:MM:SS
// 用 16x16 大数字居中，先清屏再一次性刷新。用于电脑端时间同步（屏 0）。
void oled_show_time(int idx, int year, int month, int day, int hour, int minute, int second);

// 在指定屏显示「FFT 峰值频率」：顶部小字标题「PEAK FREQ」，分隔线下方
// 16x16 大数字频率值 + "Hz" 单位，先清屏再一次性刷新。屏 1 显示 scope_display
// 计算的人声频段（60~4000 Hz）峰值频率。
void oled_show_fft(int idx, float peak_hz);