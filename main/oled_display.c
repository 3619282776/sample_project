#include "oled_display.h"

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

void oled_show_time(int idx, int year, int month, int day, int hour, int minute, int second)
{
    ssd1306_handle_t dev = oled_get(idx);
    if (dev == NULL) {
        return;
    }

    char line[32];
    ssd1306_clear_screen(dev, 0x00);

    // 第一行：YYYY-MM-DD（10 字符 × 8px = 80px）居中
    snprintf(line, sizeof(line), "%04d-%02d-%02d", year, month, day);
    ssd1306_draw_string(dev, (uint8_t)((SSD1306_WIDTH - 80) / 2), 0, (const uint8_t *)line, 16, 1);

    // 第二行：HH:MM:SS（8 字符 × 8px = 64px）居中
    snprintf(line, sizeof(line), "%02d:%02d:%02d", hour, minute, second);
    ssd1306_draw_string(dev, (uint8_t)((SSD1306_WIDTH - 64) / 2), 24, (const uint8_t *)line, 16, 1);

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

    // 顶部标题：6x12 小字（"fft visualize project" = 21 字符 × 6px = 126px，居中 x=1）
    ssd1306_draw_string(dev, 1, 0, (const uint8_t *)"fft visualize project", 12, 1);

    // 峰值频率：8x16 大字居中
    snprintf(line, sizeof(line), "%d Hz", (int)lroundf(peak_hz));
    int w = (int)strlen(line) * 8;
    ssd1306_draw_string(dev, (uint8_t)((SSD1306_WIDTH - w) / 2), 26, (const uint8_t *)line, 16, 1);

    ssd1306_refresh_gram(dev);
}