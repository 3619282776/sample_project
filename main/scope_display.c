// scope_display.c — TFT 屏幕示波器：波形 + FFT 频谱（横屏 320x240）
// 复刻 tools/pc_waveform.py 的前两图（省略时频图），屏幕横放、两图上下排列：
//   上半屏（320x120）：mono 波形（滚动折线，自动增益，左侧幅度刻度）
//   下半屏（320x120）：FFT 幅度谱（人声频段 60~4000 Hz，柱状，dBFS，左侧 dB 刻度）
//
// 两个图各用一块独立的 RGB565 帧缓冲：esp_lcd 的 SPI 传输是异步 DMA（draw_bitmap
// 返回后数据仍在传输），若两图共用一块缓冲，render 会覆盖正在 DMA 的数据，
// 表现为波形/频谱互相串染。
#include "scope_display.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"

#include "audio_config.h"
#include "ili9341.h"

static const char *TAG = "scope";

/* ---------------- 布局（横屏 320x240） ---------------- */
#define DISP_W       ILI9341_WIDTH     // 320
#define DISP_H       ILI9341_HEIGHT    // 240
#define WAVE_Y       0
#define WAVE_H       (DISP_H / 2)      // 120
#define SPEC_Y       (DISP_H / 2)      // 120
#define SPEC_H       (DISP_H / 2)      // 120

#define AXIS_X       44                // 左侧刻度区宽度，曲线/柱从第 44 列开始

/* ---------------- RGB565 颜色 ---------------- */
#define C_BG      0x0000   // 黑
#define C_GRID    0x3186   // 暗灰（网格/基线）
#define C_WAVE    0x07E0   // 绿（波形）
#define C_SPEC    0xFFE0   // 黄（频谱柱）
#define C_TEXT    0xFFFF   // 白（刻度数值）

/* ---------------- FFT 参数 ---------------- */
#define FFT_N     256                  // 与 AUDIO_BLOCK_FRAMES 对齐；16k 下分辨率约 62.5 Hz
#define FS        AUDIO_SAMPLE_RATE
#define BIN_HZ    (FS / (float)FFT_N)  // 62.5 Hz
#define VMIN      60.0f                // 人声频段下限（对齐 pc_waveform.py）
#define VMAX      4000.0f              // 人声频段上限
#define DB_MIN    (-90.0f)
#define DB_MAX    0.0f

/* ---------------- 波形历史（滚动缓冲，容量须为 2 的幂以便 & 掩码） ---------------- */
#define WAVE_HIST 1024                 // 64ms；每列取两个相邻样本均值
_Static_assert((WAVE_HIST & (WAVE_HIST - 1)) == 0, "WAVE_HIST must be a power of two");
static int16_t s_hist[WAVE_HIST];
static int s_hist_head = 0;            // 最旧样本位置

/* ---------------- 波形自动增益（px / int16） ---------------- */
#define TARGET_PX 44.0f                // 峰值希望显示到约 44px 高
#define MIN_PEAK  128.0f               // 静音时的峰值下限，避免把底噪无限放大
static float s_gain = 0.0055f;         // 初始：典型峰值约 8000 时显示 ~44px

/* ---------------- FFT 工作区 ---------------- */
static float s_re[FFT_N];
static float s_im[FFT_N];
static float s_win[FFT_N];
static float s_db[FFT_N / 2];          // bin 1..127 的 dBFS（bin0=DC 不用）

static StreamBufferHandle_t s_sb = NULL;
static uint16_t *s_wave_fb = NULL;     // 320x120（波形区）
static uint16_t *s_spec_fb = NULL;     // 320x120（频谱区）

/* ---------------- 5x7 点阵字体（0-9 与 '-'），每字节一行，bit4 为最左像素 ---------------- */
static const uint8_t s_font[11][7] = {
    { 0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E }, // 0
    { 0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E }, // 1
    { 0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F }, // 2
    { 0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E }, // 3
    { 0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02 }, // 4
    { 0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E }, // 5
    { 0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E }, // 6
    { 0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08 }, // 7
    { 0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E }, // 8
    { 0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C }, // 9
    { 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00 }, // -
};

static void draw_char(uint16_t *fb, int fb_w, int row_h, int x0, int y0, char ch, uint16_t color)
{
    int idx;
    if (ch >= '0' && ch <= '9') {
        idx = ch - '0';
    } else if (ch == '-') {
        idx = 10;
    } else {
        return;
    }
    const uint8_t *p = s_font[idx];
    for (int r = 0; r < 7; r++) {
        uint8_t bits = p[r];
        for (int c = 0; c < 5; c++) {
            if (bits & (0x10 >> c)) {
                int px = x0 + c, py = y0 + r;
                if (px >= 0 && px < fb_w && py >= 0 && py < row_h) {
                    fb[py * fb_w + px] = color;
                }
            }
        }
    }
}

/* 右对齐字符串：right_x 为最后一个字符右边界（不含），返回文本左边界 x */
static int draw_text_right(uint16_t *fb, int fb_w, int row_h, int right_x, int y0,
                           const char *s, uint16_t color)
{
    int len = (int)strlen(s);
    int w = len * 6 - 1;               // 字宽 5 + 字距 1
    int cx = right_x - w;
    for (const char *q = s; *q; q++) {
        draw_char(fb, fb_w, row_h, cx, y0, *q, color);
        cx += 6;
    }
    return right_x - w;
}

/* in-place radix-2 复数 FFT（n 为 2 的幂） */
static void fft_radix2(float *re, float *im, int n)
{
    for (int i = 1, j = 0; i < n; i++) {
        int bit = n >> 1;
        for (; j & bit; bit >>= 1) {
            j ^= bit;
        }
        j ^= bit;
        if (i < j) {
            float tr = re[i]; re[i] = re[j]; re[j] = tr;
            float ti = im[i]; im[i] = im[j]; im[j] = ti;
        }
    }
    for (int len = 2; len <= n; len <<= 1) {
        float ang = -2.0f * (float)M_PI / (float)len;
        float w_re = cosf(ang), w_im = sinf(ang);
        int half = len >> 1;
        for (int i = 0; i < n; i += len) {
            float cur_re = 1.0f, cur_im = 0.0f;
            for (int k = 0; k < half; k++) {
                int a = i + k, b = i + k + half;
                float tr = cur_re * re[b] - cur_im * im[b];
                float ti = cur_re * im[b] + cur_im * re[b];
                re[b] = re[a] - tr;
                im[b] = im[a] - ti;
                re[a] += tr;
                im[a] += ti;
                float ncr = cur_re * w_re - cur_im * w_im;
                cur_im = cur_re * w_im + cur_im * w_re;
                cur_re = ncr;
            }
        }
    }
}

/* 对 mono 样本做加窗 FFT，结果转 dBFS 存入 s_db（对齐 pc_waveform.py 的幅度谱） */
static void update_spectrum(const int16_t *x, int n)
{
    float mean = 0.0f;
    for (int i = 0; i < n; i++) {
        mean += (float)x[i];
    }
    mean /= (float)n;

    for (int i = 0; i < FFT_N; i++) {
        float v = (i < n) ? (((float)x[i] - mean) * (1.0f / 32768.0f)) : 0.0f;
        s_re[i] = v * s_win[i];
        s_im[i] = 0.0f;
    }

    fft_radix2(s_re, s_im, FFT_N);

    float ref = FFT_N / 4.0f;   // 归一化满幅正弦经 Hann 窗后的 rfft 峰值
    for (int b = 1; b < FFT_N / 2; b++) {
        float mag = sqrtf(s_re[b] * s_re[b] + s_im[b] * s_im[b]);
        s_db[b] = 20.0f * log10f(mag / ref + 1e-12f);
    }
}

/* 上半屏：波形折线 + 自适应增益 + 左侧幅度刻度 */
static void render_wave(void)
{
    int area = DISP_W * WAVE_H;
    int cy = WAVE_H / 2;
    int plot_w = DISP_W - AXIS_X;

    for (int i = 0; i < area; i++) {
        s_wave_fb[i] = C_BG;
    }

    /* 水平网格线（0 / 1/4 / 1/2 / 3/4 / 满幅） */
    for (int x = 0; x < DISP_W; x++) {
        s_wave_fb[0 * DISP_W + x] = C_GRID;
        s_wave_fb[(WAVE_H / 4) * DISP_W + x] = C_GRID;
        s_wave_fb[(WAVE_H / 2) * DISP_W + x] = C_GRID;
        s_wave_fb[(3 * WAVE_H / 4) * DISP_W + x] = C_GRID;
        s_wave_fb[(WAVE_H - 1) * DISP_W + x] = C_GRID;
    }

    /* 波形折线（滚动，每列取两个相邻样本均值，列间连线保证连续） */
    int prev_y = -1;
    for (int x = AXIS_X; x < DISP_W; x++) {
        int k = ((x - AXIS_X) * WAVE_HIST) / plot_w;
        int32_t acc = (int32_t)s_hist[(s_hist_head + k) & (WAVE_HIST - 1)]
                    + (int32_t)s_hist[(s_hist_head + k + 1) & (WAVE_HIST - 1)];
        int v = (int)(acc >> 1);

        int y = cy - (int)(v * s_gain);
        if (y < 1) y = 1;
        if (y >= WAVE_H - 1) y = WAVE_H - 2;

        if (prev_y < 0) {
            s_wave_fb[y * DISP_W + x] = C_WAVE;
        } else {
            int a = prev_y < y ? prev_y : y;
            int b = prev_y < y ? y : prev_y;
            for (int yy = a; yy <= b; yy++) {
                s_wave_fb[yy * DISP_W + x] = C_WAVE;
            }
        }
        prev_y = y;
    }

    /* 左侧刻度：纵轴 + 刻度线 + 数值 */
    for (int y = 1; y < WAVE_H - 1; y++) {
        s_wave_fb[y * DISP_W + (AXIS_X - 1)] = C_GRID;
    }
    int tick_x0 = AXIS_X - 7;
    for (int x = tick_x0; x < AXIS_X - 1; x++) {
        s_wave_fb[3 * DISP_W + x] = C_GRID;                          // 满量程
        s_wave_fb[((cy - 29)) * DISP_W + x] = C_GRID;               // 半量程
        s_wave_fb[cy * DISP_W + x] = C_GRID;                        // 0
        s_wave_fb[((cy + 29)) * DISP_W + x] = C_GRID;               // 半量程
        s_wave_fb[(WAVE_H - 4) * DISP_W + x] = C_GRID;              // 满量程
    }

    int amp_full = (int)((cy - 3) / s_gain);    // 满量程对应的 int16 幅值
    if (amp_full < 1) amp_full = 1;

    char buf[16];
    snprintf(buf, sizeof(buf), "%d", amp_full);
    draw_text_right(s_wave_fb, DISP_W, WAVE_H, AXIS_X - 9, 1, buf, C_TEXT);
    draw_text_right(s_wave_fb, DISP_W, WAVE_H, AXIS_X - 9, cy - 3, "0", C_TEXT);
    snprintf(buf, sizeof(buf), "-%d", amp_full);
    draw_text_right(s_wave_fb, DISP_W, WAVE_H, AXIS_X - 9, WAVE_H - 7, buf, C_TEXT);
}

/* 下半屏：FFT 柱状谱（线性频率轴 60~4000 Hz）+ 左侧 dB 刻度 */
static void render_spec(void)
{
    int area = DISP_W * SPEC_H;
    int base = SPEC_H - 4;
    int plot_w = DISP_W - AXIS_X;

    for (int i = 0; i < area; i++) {
        s_spec_fb[i] = C_BG;
    }

    /* 频率刻度基线 + 标记 */
    for (int x = AXIS_X; x < DISP_W; x++) {
        s_spec_fb[base * DISP_W + x] = C_GRID;
    }
    {
        const float marks[] = { 100, 500, 1000, 2000, 3000, 4000 };
        for (int i = 0; i < (int)(sizeof(marks) / sizeof(marks[0])); i++) {
            int x = AXIS_X + (int)lroundf((marks[i] - VMIN) / (VMAX - VMIN) * (plot_w - 1));
            if (x >= AXIS_X && x < DISP_W) {
                for (int y = base - 3; y <= base; y++) {
                    s_spec_fb[y * DISP_W + x] = C_GRID;
                }
            }
        }
    }

    /* dB 参考横线（0 / -45 / -90 dB） */
    {
        int y0 = base - (SPEC_H - 10);                        // 0 dB（柱顶）
        int y45 = base - (SPEC_H - 10) / 2;                   // -45 dB
        for (int x = AXIS_X; x < DISP_W; x++) {
            s_spec_fb[y0 * DISP_W + x] = C_GRID;
            s_spec_fb[y45 * DISP_W + x] = C_GRID;
            s_spec_fb[base * DISP_W + x] = C_GRID;
        }
    }

    /* 频谱柱 + bin 间线性插值平滑 */
    for (int x = AXIS_X; x < DISP_W; x++) {
        float freq = VMIN + (VMAX - VMIN) * (x - AXIS_X) / (plot_w - 1);
        float bin_f = freq / BIN_HZ;
        int b0 = (int)floorf(bin_f);
        int b1 = b0 + 1;
        if (b0 < 1) b0 = 1;
        if (b1 < 1) b1 = 1;
        if (b0 >= FFT_N / 2) b0 = FFT_N / 2 - 1;
        if (b1 >= FFT_N / 2) b1 = FFT_N / 2 - 1;
        float frac = bin_f - (float)((int)bin_f);
        if (frac < 0.0f) frac = 0.0f;
        if (frac > 1.0f) frac = 1.0f;

        float db = s_db[b0] * (1.0f - frac) + s_db[b1] * frac;
        if (db < DB_MIN) db = DB_MIN;
        if (db > DB_MAX) db = DB_MAX;

        int h = (int)((db - DB_MIN) / (DB_MAX - DB_MIN) * (SPEC_H - 10));
        if (db > DB_MIN && h < 1) {
            h = 1;
        }
        for (int y = base; y > base - h; y--) {
            s_spec_fb[y * DISP_W + x] = C_SPEC;
        }
    }

    /* 左侧 dB 刻度：纵轴 + 数值 */
    for (int y = 1; y < SPEC_H - 1; y++) {
        s_spec_fb[y * DISP_W + (AXIS_X - 1)] = C_GRID;
    }
    draw_text_right(s_spec_fb, DISP_W, SPEC_H, AXIS_X - 9, base - (SPEC_H - 10) - 3, "0", C_TEXT);
    draw_text_right(s_spec_fb, DISP_W, SPEC_H, AXIS_X - 9, base - (SPEC_H - 10) / 2 - 4, "-45", C_TEXT);
    draw_text_right(s_spec_fb, DISP_W, SPEC_H, AXIS_X - 9, base - 3, "-90", C_TEXT);
}

esp_err_t scope_display_init(void)
{
    s_sb = xStreamBufferCreate(AUDIO_BLOCK_FRAMES * sizeof(int16_t) * 8,
                               AUDIO_BLOCK_FRAMES * sizeof(int16_t));
    if (s_sb == NULL) {
        return ESP_ERR_NO_MEM;
    }

    size_t fb_bytes = DISP_W * WAVE_H * sizeof(uint16_t);
    s_wave_fb = (uint16_t *)heap_caps_malloc(fb_bytes, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    s_spec_fb = (uint16_t *)heap_caps_malloc(fb_bytes, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (s_wave_fb == NULL || s_spec_fb == NULL) {
        ESP_LOGE(TAG, "framebuffer alloc failed (need %u x2, free DMA %u)",
                 (unsigned)fb_bytes, (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA));
        return ESP_ERR_NO_MEM;
    }
    memset(s_wave_fb, 0, fb_bytes);
    memset(s_spec_fb, 0, fb_bytes);

    for (int i = 0; i < FFT_N; i++) {
        s_win[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * i / (FFT_N - 1.0f)));
    }

    ESP_LOGI(TAG, "ready (landscape waveform + FFT spectrum, auto-gain)");
    return ESP_OK;
}

int scope_display_feed(const int16_t *mono, int frames)
{
    if (s_sb == NULL || mono == NULL || frames <= 0) {
        return -1;
    }
    size_t bytes = (size_t)frames * sizeof(int16_t);
    if (xStreamBufferSpacesAvailable(s_sb) < bytes) {
        return 0;   // 显示跟不上时直接丢弃，绝不阻塞音频
    }
    return xStreamBufferSend(s_sb, mono, bytes, 0);
}

void scope_display_task(void *arg)
{
    (void)arg;
    int16_t block[AUDIO_BLOCK_FRAMES];

    while (1) {
        size_t got = xStreamBufferReceive(s_sb, block, sizeof(block), portMAX_DELAY);
        if (got == 0) {
            continue;
        }
        int n = (int)(got / sizeof(int16_t));

        for (int i = 0; i < n; i++) {
            s_hist[s_hist_head] = block[i];
            s_hist_head = (s_hist_head + 1) & (WAVE_HIST - 1);
        }

        /* 自动增益：信号大瞬时压缩（不顶满），信号小缓慢放大（看清细节） */
        {
            int peak = 1;
            for (int i = 0; i < n; i++) {
                int a = block[i];
                if (a < 0) a = -a;
                if (a > peak) peak = a;
            }
            float disp = peak * s_gain;
            if (disp > TARGET_PX) {
                s_gain = TARGET_PX / (float)peak;
            } else {
                float gt = TARGET_PX / (peak > MIN_PEAK ? (float)peak : MIN_PEAK);
                s_gain += (gt - s_gain) * 0.08f;
            }
            if (s_gain > TARGET_PX / MIN_PEAK) s_gain = TARGET_PX / MIN_PEAK;
            if (s_gain < 0.0001f) s_gain = 0.0001f;
        }

        update_spectrum(block, n);

        /* 两图各用独立缓冲，异步 DMA 之间互不覆盖 */
        render_wave();
        ili9341_draw_bitmap(0, WAVE_Y, DISP_W, WAVE_H, s_wave_fb);

        render_spec();
        ili9341_draw_bitmap(0, SPEC_Y, DISP_W, SPEC_H, s_spec_fb);
    }
}