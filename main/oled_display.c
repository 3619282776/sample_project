#include "oled_display.h"
#include "share_tech_mono.h"

#include <string.h>
#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "oled";

/* ---- 两路 I2C 总线接线（与 oled_display.h 注释一致） ---- */
#define OLED0_I2C_PORT   I2C_NUM_0
#define OLED0_PIN_SDA    13
#define OLED0_PIN_SCL    16

#define OLED1_I2C_PORT   I2C_NUM_1
#define OLED1_PIN_SDA    17
#define OLED1_PIN_SCL    19

#define OLED_I2C_FREQ_HZ 400000   // SSD1306 在 400 kHz 下工作稳定

static ssd1306_handle_t s_oled[2];

static esp_err_t oled_bus_init(i2c_port_t port, int sda, int scl)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = true,   // 打开内部上拉；模块一般也自带上拉，叠加无害
        .scl_pullup_en = true,
        .master.clk_speed = OLED_I2C_FREQ_HZ,
    };
    esp_err_t err = i2c_param_config(port, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config port %d: %s", port, esp_err_to_name(err));
        return err;
    }
    err = i2c_driver_install(port, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install port %d: %s", port, esp_err_to_name(err));
    }
    return err;
}

esp_err_t oled_display_init(void)
{
    ESP_RETURN_ON_ERROR(oled_bus_init(OLED0_I2C_PORT, OLED0_PIN_SDA, OLED0_PIN_SCL), TAG, "oled bus0 init failed");
    ESP_RETURN_ON_ERROR(oled_bus_init(OLED1_I2C_PORT, OLED1_PIN_SDA, OLED1_PIN_SCL), TAG, "oled bus1 init failed");

    // ssd1306_create 在新版组件里被标记为 deprecated，但仍是本项目选用的库函数，这里抑制该告警
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    s_oled[OLED_0] = ssd1306_create(OLED0_I2C_PORT, SSD1306_I2C_ADDRESS);
    s_oled[OLED_1] = ssd1306_create(OLED1_I2C_PORT, SSD1306_I2C_ADDRESS);
#pragma GCC diagnostic pop

    if (s_oled[OLED_0] == NULL || s_oled[OLED_1] == NULL) {
        ESP_LOGE(TAG, "ssd1306_create returned NULL");
        return ESP_FAIL;
    }

    // 上电后 GDDRAM 内容未定，先全黑刷新一次，避免显示乱码
    ssd1306_clear_screen(s_oled[OLED_0], 0x00);
    ssd1306_clear_screen(s_oled[OLED_1], 0x00);
    ssd1306_refresh_gram(s_oled[OLED_0]);
    ssd1306_refresh_gram(s_oled[OLED_1]);

    ESP_LOGI(TAG, "two SSD1306 ready (bus0 SDA=%d SCL=%d, bus1 SDA=%d SCL=%d)",
             OLED0_PIN_SDA, OLED0_PIN_SCL, OLED1_PIN_SDA, OLED1_PIN_SCL);
    return ESP_OK;
}

ssd1306_handle_t oled_get(int idx)
{
    if (idx != OLED_0 && idx != OLED_1) {
        return NULL;
    }
    return s_oled[idx];
}

void oled_clear(int idx)
{
    ssd1306_handle_t dev = oled_get(idx);
    if (dev == NULL) {
        return;
    }
    ssd1306_clear_screen(dev, 0x00);
    ssd1306_refresh_gram(dev);
}

void oled_print(int idx, uint8_t x, uint8_t y, const char *s)
{
    ssd1306_handle_t dev = oled_get(idx);
    if (dev == NULL || s == NULL) {
        return;
    }
    ssd1306_draw_string(dev, x, y, (const uint8_t *)s, 16, 1);
    ssd1306_refresh_gram(dev);
}

/* ---- 时间屏辅助：蔡勒公式计算星期（0=周日 .. 6=周六） ---- */
static const char *const s_weekday_names[7] = { "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat" };

static int weekday_index(int year, int month, int day)
{
    if (month < 3) {
        month += 12;
        year--;
    }
    int k = year % 100;
    int j = year / 100;
    int h = (day + 13 * (month + 1) / 5 + k + k / 4 + j / 4 + 5 * j) % 7;
    return (h + 6) % 7;   // h: 0=周六 .. 6=周五 → 转成 0=周日
}

/* 用 Share Tech Mono 16x16 位图字模画一个字符（idx: 0-9 数字，10 冒号） */
static void oled_draw_stm_char(ssd1306_handle_t dev, uint8_t x, uint8_t y, uint8_t idx)
{
    for (int r = 0; r < 16; r++) {
        uint8_t b0 = stm_glyphs[idx][r][0];
        uint8_t b1 = stm_glyphs[idx][r][1];
        for (int c = 0; c < 8; c++) {
            if (b0 & (0x80 >> c)) {
                ssd1306_fill_point(dev, x + c, y + r, 1);
            }
        }
        for (int c = 0; c < 8; c++) {
            if (b1 & (0x80 >> c)) {
                ssd1306_fill_point(dev, x + 8 + c, y + r, 1);
            }
        }
    }
}

/* 用 Share Tech Mono 16x16 数字 + 冒号画 HH:MM:SS（等宽 16px，8 字符共 128px） */
static void oled_draw_clock(ssd1306_handle_t dev, uint8_t x, uint8_t y,
                            int hour, int minute, int second)
{
    char b[3];

    snprintf(b, sizeof(b), "%02d", hour);
    oled_draw_stm_char(dev, x, y, (uint8_t)(b[0] - '0'));
    oled_draw_stm_char(dev, x + 16, y, (uint8_t)(b[1] - '0'));
    oled_draw_stm_char(dev, x + 32, y, 10);   // ':'

    snprintf(b, sizeof(b), "%02d", minute);
    oled_draw_stm_char(dev, x + 48, y, (uint8_t)(b[0] - '0'));
    oled_draw_stm_char(dev, x + 64, y, (uint8_t)(b[1] - '0'));
    oled_draw_stm_char(dev, x + 80, y, 10);   // ':'

    snprintf(b, sizeof(b), "%02d", second);
    oled_draw_stm_char(dev, x + 96, y, (uint8_t)(b[0] - '0'));
    oled_draw_stm_char(dev, x + 112, y, (uint8_t)(b[1] - '0'));
}

void oled_show_time(int idx, int year, int month, int day, int hour, int minute, int second)
{
    ssd1306_handle_t dev = oled_get(idx);
    if (dev == NULL) {
        return;
    }

    char line[32];
    ssd1306_clear_screen(dev, 0x00);

    // 顶部：星期 + 日期（6x12 小字，14 字符 × 6px = 84px）居中
    snprintf(line, sizeof(line), "%s %04d-%02d-%02d",
             s_weekday_names[weekday_index(year, month, day)], year, month, day);
    ssd1306_draw_string(dev, (uint8_t)((SSD1306_WIDTH - 84) / 2), 2, (const uint8_t *)line, 12, 1);

    // 分隔线
    ssd1306_fill_rectangle(dev, 16, 16, 111, 16, 1);

    // 中部：HH:MM:SS 大数字（Share Tech Mono 16x16，等宽共 128px）
    oled_draw_clock(dev, 0, 32, hour, minute, second);

    ssd1306_refresh_gram(dev);
}

void oled_show_fft(int idx, float peak_hz)
{
    ssd1306_handle_t dev = oled_get(idx);
    if (dev == NULL) {
        return;
    }

    char line[32];
    ssd1306_clear_screen(dev, 0x00);

    // 顶部标题（6x12 小字，9 字符 × 6px = 54px）居中
    ssd1306_draw_string(dev, (uint8_t)((SSD1306_WIDTH - 54) / 2), 2,
                        (const uint8_t *)"PEAK FREQ", 12, 1);

    // 分隔线
    ssd1306_fill_rectangle(dev, 16, 16, 111, 16, 1);

    // 中部：峰值频率 Share Tech Mono 16x16 数字 + 8x16 单位 "Hz"
    int hz = (int)lroundf(peak_hz);
    if (hz < 0) {
        hz = 0;
    }
    snprintf(line, sizeof(line), "%d", hz);
    int n = (int)strlen(line);
    int total = n * 16 + 4 + 16;             // 数字 + 间隔 + "Hz"
    int x = (SSD1306_WIDTH - total) / 2;

    for (int i = 0; i < n; i++) {
        oled_draw_stm_char(dev, (uint8_t)(x + i * 16), 32, (uint8_t)(line[i] - '0'));
    }
    ssd1306_draw_string(dev, (uint8_t)(x + n * 16 + 4), 32, (const uint8_t *)"Hz", 16, 1);

    ssd1306_refresh_gram(dev);
}