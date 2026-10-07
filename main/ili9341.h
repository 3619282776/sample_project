#pragma once

#include <stdint.h>

// LCD SPI 引脚（ILI9341 命名沿用；实际是 ST7789V，2.8 寸 240x320）
#define ILI9341_PIN_SCLK  18
#define ILI9341_PIN_MOSI  23
#define ILI9341_PIN_CS    5
#define ILI9341_PIN_DC    4
#define ILI9341_PIN_RST   -1   // LCD RST 未接 MCU（已接 VCC），-1 = 走 esp_lcd 软件复位；GPIO3 让给 UART0 RX
#define ILI9341_PIN_BL    2

// 横屏逻辑分辨率（esp_lcd 交换 X/Y 后 = 320x240）
#define ILI9341_WIDTH   320
#define ILI9341_HEIGHT  240

void ili9341_init(void);
void ili9341_fill_screen(uint16_t color);
void ili9341_fill_rect(int x, int y, int w, int h, uint16_t color);

// 一次性刷新一块 RGB565 像素（pixels 需为 DMA 可访问内存），用于无闪烁整块重绘
void ili9341_draw_bitmap(int x, int y, int w, int h, const uint16_t *pixels);

// 7 段数码管风格绘制整数（含负号）
void ili9341_draw_number(int x, int y, int seg_len, int thick, uint16_t color, int number);

// 3x5 点阵绘制少量字符（支持 'L'、'C'、'R' 及空格）
void ili9341_draw_text3x5(int x, int y, int scale, uint16_t color, const char *s);
