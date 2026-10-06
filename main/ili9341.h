#pragma once

#include <stdint.h>

// LCD SPI 引脚（ILI9341，2.8 寸 320x240）
#define ILI9341_PIN_SCLK  18
#define ILI9341_PIN_MOSI  23
#define ILI9341_PIN_CS    5
#define ILI9341_PIN_DC    4
#define ILI9341_PIN_RST   3
#define ILI9341_PIN_BL    2

#define ILI9341_WIDTH   240
#define ILI9341_HEIGHT  320

void ili9341_init(void);
void ili9341_fill_screen(uint16_t color);
void ili9341_fill_rect(int x, int y, int w, int h, uint16_t color);

// 7 段数码管风格绘制整数（含负号）
void ili9341_draw_number(int x, int y, int seg_len, int thick, uint16_t color, int number);

// 3x5 点阵绘制少量字符（支持 'L'、'C'、'R' 及空格）
void ili9341_draw_text3x5(int x, int y, int scale, uint16_t color, const char *s);
