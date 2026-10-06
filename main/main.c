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
#include "uart_stream.h"

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

        // 波形透传电脑：~32 KB/s 远低于 921600 波特率上限，uart_write_bytes 只做内存拷贝，不会阻塞实时音频
        uart_stream_send(mono, 1, frames);
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
                float ang = doa_estimate_angle(s_doa_l[b], s_doa_r[b], DOA_WINDOW);
                if (!isnan(ang)) {          // 有效方向才更新，静音时保持上次角度
                    s_angle_deg = ang;
                }
            }
        }
    }
}

/* ---- LCD 显示：360° 罗盘 + 圆点（指示声源方向）---- */

#define COLOR_BG     0x0000
#define COLOR_RING   0xFFFF
#define COLOR_TICK   0x7BEF
#define COLOR_WHITE  0xFFFF
#define COLOR_GREEN  0x07E0
#define COLOR_DOT    0xFFE0

#define PI_F 3.14159265358979323846f

#define CX       120                 // 罗盘圆心（竖屏 240x320）
#define CY       150
#define R_RING   78                  // 罗盘圆环半径
#define R_DOT    (R_RING - 7)        // 圆点轨道半径
#define DOT_R    4                   // 圆点半径

#define NUM_Y    (CY + R_RING + 24)  // 底部角度数字起始 y

/* 逐扫描线填充实心圆 */
static void draw_filled_circle(int cx, int cy, int r, uint16_t color)
{
    if (r < 0) {
        return;
    }
    int r2 = r * r;
    for (int dy = -r; dy <= r; dy++) {
        int dx = (int)(sqrtf((float)(r2 - dy * dy)) + 0.5f);
        ili9341_fill_rect(cx - dx, cy + dy, 2 * dx + 1, 1, color);
    }
}

/* Bresenham 直线（线宽约 2px） */
static void draw_line(int x0, int y0, int x1, int y1, uint16_t color)
{
    int ax = (x1 >= x0) ? (x1 - x0) : (x0 - x1);
    int ay = (y1 >= y0) ? (y1 - y0) : (y0 - y1);
    int dx = ax, dy = -ay;
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    for (;;) {
        ili9341_fill_rect(x0 - 1, y0 - 1, 2, 2, color);
        if (x0 == x1 && y0 == y1) {
            break;
        }
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static void draw_compass_background(void)
{
    ili9341_fill_screen(COLOR_BG);

    /* 圆环（线宽约 3px） */
    draw_filled_circle(CX, CY, R_RING + 1, COLOR_RING);
    draw_filled_circle(CX, CY, R_RING - 1, COLOR_BG);

    /* 12 根刻度，每 30° 一根；四个主方向更长更亮 */
    for (int i = 0; i < 12; i++) {
        float a = (float)i * (PI_F / 6.0f);
        float s = sinf(a), c = cosf(a);
        int corner = (i % 3 == 0);
        int r1 = corner ? (R_RING + 2) : (R_RING + 3);
        int r2 = corner ? (R_RING + 14) : (R_RING + 10);
        uint16_t col = corner ? COLOR_WHITE : COLOR_TICK;
        draw_line(CX + (int)lrintf(s * (float)r1), CY - (int)lrintf(c * (float)r1),
                  CX + (int)lrintf(s * (float)r2), CY - (int)lrintf(c * (float)r2), col);
    }

    /* 正前方参考指针（圆心向上，不进入圆点轨道） */
    ili9341_fill_rect(CX - 1, CY - (R_DOT - 8), 3, R_DOT - 8, COLOR_GREEN);

    /* 中心点 */
    draw_filled_circle(CX, CY, 2, COLOR_WHITE);
}

static int s_num_last = 9999;
static void draw_angle_number(int ai)
{
    if (ai == s_num_last) {
        return;
    }
    s_num_last = ai;
    ili9341_fill_rect(CX - 60, NUM_Y - 4, 120, 40, COLOR_BG);
    ili9341_draw_number(CX - 39, NUM_Y, 14, 4, COLOR_WHITE, ai);
}

static void lcd_task(void *arg)
{
    float smoothed = 0.0f;
    int last_x = -10000;
    int last_y = -10000;
    bool dot_drawn = false;

    draw_compass_background();

    while (1) {
        /* 指数平滑，让圆点在两次 DOA 更新之间连续滑动 */
        float target = s_angle_deg;
        smoothed += 0.25f * (target - smoothed);

        float rad = smoothed * (PI_F / 180.0f);
        int x = CX + (int)lrintf(sinf(rad) * (float)R_DOT);
        int y = CY - (int)lrintf(cosf(rad) * (float)R_DOT);

        if (dot_drawn && (x != last_x || y != last_y)) {
            draw_filled_circle(last_x, last_y, DOT_R + 1, COLOR_BG);
        }
        draw_filled_circle(x, y, DOT_R, COLOR_DOT);
        last_x = x;
        last_y = y;
        dot_drawn = true;

        draw_angle_number((int)lrintf(smoothed));

        vTaskDelay(pdMS_TO_TICKS(16));  /* ~60 fps */
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(uart_stream_init());   // 先装 UART 驱动并重定向日志，启动日志也走 921600
    ESP_LOGI(TAG, "starting...");

    ili9341_init();
    ili9341_fill_screen(COLOR_BG);   // 初始清屏为背景色，随后由 lcd_task 绘制声源方向
    ESP_ERROR_CHECK(mic_i2s_init());
    ESP_ERROR_CHECK(amp_i2s_init());

    s_doa_sem = xSemaphoreCreateBinary();
    s_playback_sb = xStreamBufferCreate(PLAYBACK_BUF_BYTES, AUDIO_BLOCK_FRAMES * sizeof(int16_t));

    xTaskCreate(audio_task, "audio", 4096, NULL, 6, NULL);
    xTaskCreate(playback_task, "playback", 4096, NULL, 5, NULL);
    xTaskCreate(doa_task, "doa", 4096, NULL, 4, NULL);
    xTaskCreate(lcd_task, "lcd", 4096, NULL, 3, NULL);

    ESP_LOGI(TAG, "tasks started");
}
