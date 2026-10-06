// 对外仍沿用接线表里的 "ili9341" 命名；内部改用官方 esp_lcd 框架 + ST7789 驱动。
// 注：多数 2.8 寸模块实际是 ST7789V（240x320）。若你的屏确为 ILI9341，请告知，我再换回 ILI9341 驱动。
#include "ili9341.h"

#include <string.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_lcd_panel_ops.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

static const char *TAG = "lcd";
static esp_lcd_panel_handle_t s_panel = NULL;

void ili9341_init(void)
{
    if (ILI9341_PIN_BL >= 0) {
        gpio_config_t bl = {
            .pin_bit_mask = 1ULL << ILI9341_PIN_BL,
            .mode = GPIO_MODE_OUTPUT,
        };
        gpio_config(&bl);
        gpio_set_level(ILI9341_PIN_BL, 1);
    }

    spi_bus_config_t bus_cfg = {
        .sclk_io_num = ILI9341_PIN_SCLK,
        .mosi_io_num = ILI9341_PIN_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = ILI9341_WIDTH * ILI9341_HEIGHT * 2 + 8,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = ILI9341_PIN_CS,
        .dc_gpio_num = ILI9341_PIN_DC,
        .spi_mode = 0,
        .pclk_hz = 26 * 1000 * 1000,
        .trans_queue_depth = 10,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &io));

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = ILI9341_PIN_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io, &panel_cfg, &s_panel));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));

    // 横屏：交换 X/Y 轴（240x320 -> 320x240）。若屏幕内容上下/左右颠倒，调整下面
    // mirror 的两个布尔值即可（四个组合对应四种旋转方向）。
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(s_panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(s_panel, true, false));

    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));

    ESP_LOGI(TAG, "LCD ready (esp_lcd + ST7789)");
}

void ili9341_fill_rect(int x, int y, int w, int h, uint16_t color)
{
    if (w <= 0 || h <= 0) {
        return;
    }
    static uint16_t line[ILI9341_WIDTH];
    for (int i = 0; i < w; i++) {
        line[i] = color;
    }
    for (int yy = 0; yy < h; yy++) {
        esp_lcd_panel_draw_bitmap(s_panel, x, y + yy, x + w, y + yy + 1, line);
    }
}

void ili9341_fill_screen(uint16_t color)
{
    ili9341_fill_rect(0, 0, ILI9341_WIDTH, ILI9341_HEIGHT, color);
}

void ili9341_draw_bitmap(int x, int y, int w, int h, const uint16_t *pixels)
{
    if (pixels == NULL || w <= 0 || h <= 0) {
        return;
    }
    esp_lcd_panel_draw_bitmap(s_panel, x, y, x + w, y + h, pixels);
}

/* ---------------- 7 段数码管数字 ---------------- */

#define SEG_A 0x01
#define SEG_B 0x02
#define SEG_C 0x04
#define SEG_D 0x08
#define SEG_E 0x10
#define SEG_F 0x20
#define SEG_G 0x40

static const uint8_t s_digit_segs[10] = {
    SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F,          // 0
    SEG_B|SEG_C,                                  // 1
    SEG_A|SEG_B|SEG_D|SEG_E|SEG_G,                // 2
    SEG_A|SEG_B|SEG_C|SEG_D|SEG_G,                // 3
    SEG_B|SEG_C|SEG_F|SEG_G,                      // 4
    SEG_A|SEG_C|SEG_D|SEG_F|SEG_G,                // 5
    SEG_A|SEG_C|SEG_D|SEG_E|SEG_F|SEG_G,          // 6
    SEG_A|SEG_B|SEG_C,                            // 7
    SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F|SEG_G,    // 8
    SEG_A|SEG_B|SEG_C|SEG_D|SEG_F|SEG_G,          // 9
};

static void draw_segments(int x, int y, int s, int t, uint16_t color, uint8_t mask)
{
    if (mask & SEG_A) ili9341_fill_rect(x + t, y, s, t, color);
    if (mask & SEG_G) ili9341_fill_rect(x + t, y + s, s, t, color);
    if (mask & SEG_D) ili9341_fill_rect(x + t, y + 2 * s, s, t, color);
    if (mask & SEG_F) ili9341_fill_rect(x, y + t, t, s, color);
    if (mask & SEG_B) ili9341_fill_rect(x + s + t, y + t, t, s, color);
    if (mask & SEG_E) ili9341_fill_rect(x, y + s + t, t, s, color);
    if (mask & SEG_C) ili9341_fill_rect(x + s + t, y + s + t, t, s, color);
}

void ili9341_draw_number(int x, int y, int seg_len, int thick, uint16_t color, int number)
{
    int digit_w = seg_len + 3 * thick;   // 数字宽度 + 间隔

    if (number < 0) {
        draw_segments(x, y, seg_len, thick, color, SEG_G);  // 负号（中间横段）
        x += digit_w;
        number = -number;
    }

    int div = 100;
    int started = 0;
    while (div >= 1) {
        int d = (number / div) % 10;
        if (d != 0 || started || div == 1) {
            draw_segments(x, y, seg_len, thick, color, s_digit_segs[d]);
            x += digit_w;
            started = 1;
        }
        div /= 10;
    }
}

/* ---------------- 3x5 点阵字符 ---------------- */

static const uint8_t s_glyph_L[5] = { 0x04, 0x04, 0x04, 0x04, 0x07 };
static const uint8_t s_glyph_C[5] = { 0x03, 0x04, 0x04, 0x04, 0x03 };
static const uint8_t s_glyph_R[5] = { 0x06, 0x05, 0x06, 0x05, 0x05 };
static const uint8_t s_glyph_sp[5] = { 0x00, 0x00, 0x00, 0x00, 0x00 };

void ili9341_draw_text3x5(int x, int y, int scale, uint16_t color, const char *s)
{
    while (*s) {
        const uint8_t *g = NULL;
        if (*s == 'L')       g = s_glyph_L;
        else if (*s == 'C')  g = s_glyph_C;
        else if (*s == 'R')  g = s_glyph_R;
        else if (*s == ' ')  g = s_glyph_sp;

        if (g) {
            for (int row = 0; row < 5; row++) {
                for (int col = 0; col < 3; col++) {
                    if (g[row] & (1 << (2 - col))) {
                        ili9341_fill_rect(x + col * scale, y + row * scale, scale, scale, color);
                    }
                }
            }
        }
        x += 4 * scale;   // 3 列 + 1 空隙
        s++;
    }
}
