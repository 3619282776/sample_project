#include <math.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"

#include "esp_log.h"

#include "audio_config.h"
#include "inmp441_doa.h"
#include "max98357.h"
#include "ili9341.h"

static const char *TAG = "main";

/* ---- DOA 乒乓缓冲 ---- */
static int32_t s_doa_l[2][DOA_WINDOW];
static int32_t s_doa_r[2][DOA_WINDOW];
static volatile int s_doa_fill = 0;    // 正在填充的缓冲编号
static volatile int s_doa_ready = -1;  // 已填满待处理的缓冲编号
static int s_doa_idx = 0;
static SemaphoreHandle_t s_doa_sem = NULL;

/* ---- 回放流缓冲 ---- */
static StreamBufferHandle_t s_playback_sb = NULL;

/* ---- 方位角（度），DOA 任务写入，LCD 任务读取 ---- */
static volatile float s_angle_deg = 0.0f;

static void audio_task(void *arg)
{
    static int32_t interleaved[AUDIO_BLOCK_FRAMES * 2];
    static int16_t mono[AUDIO_BLOCK_FRAMES];

    while (1) {
        int frames = mic_i2s_read(interleaved, AUDIO_BLOCK_FRAMES, 1000);
        if (frames <= 0) {
            continue;
        }

        for (int i = 0; i < frames; i++) {
            int32_t l = interleaved[i * 2];
            int32_t r = interleaved[i * 2 + 1];
            mono[i] = (int16_t)((l + r) >> 9);   // 平均并降到 16bit

            int b = s_doa_fill;
            s_doa_l[b][s_doa_idx] = l;
            s_doa_r[b][s_doa_idx] = r;
            s_doa_idx++;
            if (s_doa_idx >= DOA_WINDOW) {
                s_doa_idx = 0;
                s_doa_ready = b;
                s_doa_fill = b ^ 1;
                xSemaphoreGive(s_doa_sem);
            }
        }

        // 送入回放流缓冲（触发阈值与块大小一致，保证整块原子收发）
        xStreamBufferSend(s_playback_sb, mono, (size_t)frames * sizeof(int16_t), portMAX_DELAY);
    }
}

static void playback_task(void *arg)
{
    static int16_t mono[AUDIO_BLOCK_FRAMES];
    const size_t block = AUDIO_BLOCK_FRAMES * sizeof(int16_t);

    while (1) {
        size_t got = xStreamBufferReceive(s_playback_sb, mono, block, portMAX_DELAY);
        if (got > 0) {
            amp_write(mono, (int)(got / sizeof(int16_t)), portMAX_DELAY);
        }
    }
}

static void doa_task(void *arg)
{
    while (1) {
        if (xSemaphoreTake(s_doa_sem, portMAX_DELAY) == pdTRUE) {
            int b = s_doa_ready;
            if (b >= 0) {
                s_angle_deg = doa_estimate_angle(s_doa_l[b], s_doa_r[b], DOA_WINDOW);
            }
        }
    }
}

/* ---- LCD 显示：水平条（左-中-右）---- */

#define COLOR_BG     0x0000
#define COLOR_BAR    0x2104
#define COLOR_WHITE  0xFFFF
#define COLOR_GREEN  0x07E0

#define BAR_X0   40
#define BAR_X1   (ILI9341_WIDTH - 40)
#define BAR_Y    90
#define BAR_H    20
#define BAR_CX   ((BAR_X0 + BAR_X1) / 2)
#define BAR_HALF ((BAR_X1 - BAR_X0) / 2)

static bool s_bg_drawn = false;

static void draw_doa_display(int angle_deg)
{
    if (!s_bg_drawn) {
        ili9341_fill_screen(COLOR_BG);
        ili9341_draw_text3x5(BAR_X0 - 12, BAR_Y + BAR_H + 24, 3, COLOR_WHITE, "L");
        ili9341_draw_text3x5(BAR_CX - 5,  BAR_Y + BAR_H + 24, 3, COLOR_WHITE, "C");
        ili9341_draw_text3x5(BAR_X1 + 4,  BAR_Y + BAR_H + 24, 3, COLOR_WHITE, "R");
        s_bg_drawn = true;
    }

    // 重绘条带（清除旧标记）
    ili9341_fill_rect(BAR_X0, BAR_Y - 8, BAR_X1 - BAR_X0, BAR_H + 16, COLOR_BG);
    ili9341_fill_rect(BAR_X0, BAR_Y, BAR_X1 - BAR_X0, BAR_H, COLOR_BAR);
    ili9341_fill_rect(BAR_CX - 2, BAR_Y - 6, 4, BAR_H + 12, COLOR_GREEN);          // 中央刻度
    ili9341_fill_rect(BAR_X0, BAR_Y - 4, 2, BAR_H + 8, COLOR_WHITE);              // 左端刻度
    ili9341_fill_rect(BAR_X1 - 2, BAR_Y - 4, 2, BAR_H + 8, COLOR_WHITE);          // 右端刻度

    // 标记
    int mx = BAR_CX + (int)((float)angle_deg * (float)BAR_HALF / 90.0f);
    if (mx < BAR_X0) mx = BAR_X0;
    if (mx > BAR_X1) mx = BAR_X1;
    ili9341_fill_rect(mx - 4, BAR_Y - 8, 8, BAR_H + 16, COLOR_GREEN);

    // 角度数字
    ili9341_fill_rect(BAR_CX - 70, BAR_Y + 60, 140, 40, COLOR_BG);
    ili9341_draw_number(BAR_CX - 45, BAR_Y + 62, 12, 5, COLOR_WHITE, angle_deg);
}

static void lcd_task(void *arg)
{
    int last_angle = 9999;
    while (1) {
        int angle = (int)s_angle_deg;
        if (angle != last_angle) {
            last_angle = angle;
            draw_doa_display(angle);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "starting...");

    ili9341_init();
    ili9341_fill_screen(0xFFFF);   // 全白测试：白屏
    ESP_ERROR_CHECK(mic_i2s_init());
    ESP_ERROR_CHECK(amp_i2s_init());

    s_doa_sem = xSemaphoreCreateBinary();
    s_playback_sb = xStreamBufferCreate(PLAYBACK_BUF_BYTES, AUDIO_BLOCK_FRAMES * sizeof(int16_t));

    xTaskCreate(audio_task, "audio", 4096, NULL, 6, NULL);
    xTaskCreate(playback_task, "playback", 4096, NULL, 5, NULL);
    xTaskCreate(doa_task, "doa", 4096, NULL, 4, NULL);
    // xTaskCreate(lcd_task, "lcd", 4096, NULL, 3, NULL);   // 全白测试：关闭方位显示

    ESP_LOGI(TAG, "tasks started");
}
