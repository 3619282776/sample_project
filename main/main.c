#include <math.h>
#include <string.h>

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
#include "scope_display.h"
#include "oled_display.h"
#include "time_sync.h"

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

/* ---- 方位角（度），DOA 任务写（当前仅日志/预留，屏幕已改为示波器） ---- */
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

        // TFT 屏幕示波器（波形 + FFT 频谱），非阻塞
        scope_display_feed(mono, frames);
    }
}

static void playback_task(void *arg)
{
    static int16_t mono[AUDIO_BLOCK_FRAMES];
    const size_t block = AUDIO_BLOCK_FRAMES * sizeof(int16_t);

    while (1) {
        size_t got = xStreamBufferReceive(s_playback_sb, mono, block, portMAX_DELAY);
        if (got > 0) {
            int samples = (int)(got / sizeof(int16_t));
            // 示波模式写静音（全 0）而非停止 I2S，避免功放输入悬空产生杂音
            if (!scope_display_is_doa()) {
                memset(mono, 0, got);
            }
            amp_write(mono, samples, portMAX_DELAY);
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
                    scope_display_set_angle((float)s_angle_deg);   // 供方向画面显示
                }
            }
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(uart_stream_init());   // 先装 UART 驱动并重定向日志，启动日志也走 921600
    ESP_LOGI(TAG, "starting...");

    // 两块 SSD1306 OLED（两条独立 I2C 总线），上电即显示各自编号与接线，便于核对 SDA/SCL
    ESP_ERROR_CHECK(oled_display_init());
    oled_print(OLED_0, 0, 0,  "OLED-0");
    oled_print(OLED_0, 0, 24, "waiting time...");   // 收到电脑时间后由 time_sync 任务覆盖为实时时钟
    oled_print(OLED_1, 0, 0,  "OLED-1");
    oled_print(OLED_1, 0, 24, "SDA17 SCL19");

    // 启动 UART0 RX 时间同步：电脑端（pc_waveform.py）每秒下发一次电脑时间，显示到屏 0
    ESP_ERROR_CHECK(time_sync_start());

    ili9341_init();
    ili9341_fill_screen(0x0000);   // 初始清屏为黑，随后由示波器任务绘制波形/频谱
    ESP_ERROR_CHECK(scope_display_init());
    ESP_ERROR_CHECK(mic_i2s_init());
    ESP_ERROR_CHECK(amp_i2s_init());

    s_doa_sem = xSemaphoreCreateBinary();
    s_playback_sb = xStreamBufferCreate(PLAYBACK_BUF_BYTES, AUDIO_BLOCK_FRAMES * sizeof(int16_t));

    xTaskCreate(audio_task, "audio", 4096, NULL, 6, NULL);
    xTaskCreate(playback_task, "playback", 4096, NULL, 5, NULL);
    xTaskCreate(doa_task, "doa", 4096, NULL, 4, NULL);
    xTaskCreate(scope_display_task, "scope", 8192, NULL, 3, NULL);

    ESP_LOGI(TAG, "tasks started");
}