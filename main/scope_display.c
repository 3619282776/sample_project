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

#include <stdbool.h>

#include "esp_heap_caps.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"

#include "audio_config.h"
#include "ili9341.h"
#include "oled_display.h"

static const char *TAG = "scope";

/* ---------------- 布局（横屏 320x240） ---------------- */
#define DISP_W       ILI9341_WIDTH     // 320
#define DISP_H       ILI9341_HEIGHT    // 240
#define WAVE_Y       0
#define WAVE_H       (DISP_H / 2)      // 120
#define SPEC_Y       (DISP_H / 2)      // 120
#define SPEC_H       (DISP_H / 2)      // 120

#define AXIS_X       44                // 左侧刻度区宽度，曲线/柱从第 44 列开始
#define SCOPE_FRAME_MS 40              // TFT 示波器渲染节流：约 25fps（原约 60fps）

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
static volatile float s_peak_hz = 0.0f; // 人声频段峰值频率（Hz），供 OLED 屏 1 显示

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
    float amp_sum = 0.0f;
    for (int i = 0; i < n; i++) {
        float v = (float)x[i];
        mean += v;
        amp_sum += fabsf(v);
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

    // 峰值频率：人声频段 (VMIN~VMAX) 内幅度最大的 bin。best 从 -1 起步，只在有效
    // bin 内更新，避免依赖 bin1 恰好落在 VMIN 内。
    int best = -1;
    for (int b = 1; b < FFT_N / 2; b++) {
        float f = (float)b * BIN_HZ;
        if (f < VMIN || f > VMAX) {
            continue;
        }
        if (best < 0 || s_db[b] > s_db[best]) {
            best = b;
        }
    }

    // 静音门限：块平均幅值过低时不更新峰值频率，保持上次值（与 DOA 一致）。
    // FFT/频谱照常刷新，TFT 仍正常显示噪声底，只有 OLED 屏 1 的峰值频率保持不变。
    if (best >= 0 && amp_sum / (float)n >= SPEC_MIN_AMPLITUDE) {
        s_peak_hz = (float)best * BIN_HZ;
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

/* ---------------- 屏幕切换按键 + 收音方向画面 ---------------- */

#define BTN_GPIO          21      // 短按切换屏幕；低电平有效（内部上拉，接 GND），无需外部电阻
#define BTN_DEBOUNCE_MS   30
#define DOA_SMOOTH_TAU_MS 80      // 方向小圈的低通平滑时间常数

enum { SCREEN_SCOPE = 0, SCREEN_DOA = 1 };

static volatile float s_doa_angle = 0.0f;   // 目标方位角（度），doa_task 写入
static int s_screen = SCREEN_SCOPE;          // 当前显示画面
static float s_doa_smooth = 0.0f;            // 平滑后的角度（小圈所在位置）
static TickType_t s_last_doa_tick = 0;       // 上次平滑的滴答计数

/* 当前是否处于「收音方向」画面（供功放回放判断：仅方向模式出声） */
bool scope_display_is_doa(void)
{
    return s_screen == SCREEN_DOA;
}

/* 5x7 点阵字符按比例放大绘制到帧缓冲（支持 0-9 与 '-'） */
static void fb_draw_char(uint16_t *fb, int fb_w, int fb_h, int x0, int y0, int scale,
                         char ch, uint16_t color)
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
                for (int dy = 0; dy < scale; dy++) {
                    for (int dx = 0; dx < scale; dx++) {
                        int px = x0 + c * scale + dx;
                        int py = y0 + r * scale + dy;
                        if (px >= 0 && px < fb_w && py >= 0 && py < fb_h) {
                            fb[py * fb_w + px] = color;
                        }
                    }
                }
            }
        }
    }
}

/* 字符串显示宽度（与 fb_draw_text 的 6*scale 步进一致） */
static int fb_text_width(const char *s, int scale)
{
    int len = (int)strlen(s);
    return len > 0 ? (6 * len - 1) * scale : 0;
}

/* 按比例绘制字符串到帧缓冲 */
static void fb_draw_text(uint16_t *fb, int fb_w, int fb_h, int x0, int y0, int scale,
                         const char *s, uint16_t color)
{
    int x = x0;
    while (*s) {
        fb_draw_char(fb, fb_w, fb_h, x, y0, scale, *s, color);
        x += 6 * scale;   // 字宽 5*scale + 间距 scale
        s++;
    }
}

/* 全局像素写入（自动定位到上/下半屏缓冲，供跨屏图形使用） */
static void fb_put_px(int x, int y, uint16_t color)
{
    if (x < 0 || x >= DISP_W || y < 0 || y >= DISP_H) {
        return;
    }
    if (y < DISP_H / 2) {
        s_wave_fb[y * DISP_W + x] = color;
    } else {
        s_spec_fb[(y - DISP_H / 2) * DISP_W + x] = color;
    }
}

/* 填充实心圆 */
static void fb_disc(int cx, int cy, int r, uint16_t color)
{
    int r2 = r * r;
    for (int dy = -r; dy <= r; dy++) {
        for (int dx = -r; dx <= r; dx++) {
            if (dx * dx + dy * dy <= r2) {
                fb_put_px(cx + dx, cy + dy, color);
            }
        }
    }
}

/* 上半圆弧：角参数 t 从 0°（右端）到 180°（左端），th 为粗细 */
static void fb_arc(int cx, int cy, int r, int th, uint16_t color)
{
    for (int t = 0; t <= 180; t++) {
        float rad = t * (float)M_PI / 180.0f;
        float c = cosf(rad);
        float s = sinf(rad);
        for (int k = -th / 2; k <= th / 2; k++) {
            int px = cx + (int)lroundf((r + k) * c);
            int py = cy - (int)lroundf((r + k) * s);
            fb_put_px(px, py, color);
        }
    }
}

/* 刻度：方位角 theta_deg（0=正上、+90=右、-90=左），沿径向向外伸 len */
static void fb_tick(int cx, int cy, int r, int theta_deg, int len, int th, uint16_t color)
{
    float rad = theta_deg * (float)M_PI / 180.0f;
    float c = sinf(rad);   // 径向向外的 x 分量
    float s = cosf(rad);   // 径向向外的 y 分量
    for (int k = 0; k < len; k++) {
        int px = cx + (int)lroundf((r + k) * c);
        int py = cy - (int)lroundf((r + k) * s);
        for (int e = -th / 2; e <= th / 2; e++) {
            fb_put_px(px + e, py, color);
        }
    }
}

/* 消抖后的按键下降沿检测：稳定按下返回 true 一次，松开后才可能再次触发 */
static bool button_pressed_event(void)
{
    static int stable = 1;             // 已确认的稳定电平（1=松开）
    static int prev_raw = 1;           // 上一次采样原始电平
    static TickType_t t_change = 0;    // 电平开始变化的时间

    int raw = gpio_get_level(BTN_GPIO);
    TickType_t now = xTaskGetTickCount();

    if (raw != prev_raw) {
        prev_raw = raw;
        t_change = now;
        return false;
    }
    if (raw != stable && (now - t_change) >= pdMS_TO_TICKS(BTN_DEBOUNCE_MS)) {
        stable = raw;
        if (stable == 0) {             // 稳定过渡到低电平 = 一次有效按下
            return true;
        }
    }
    return false;
}

/* 收音方向画面：半圆表盘 + 沿弧滑动的小圈。
 * 先在帧缓冲里按全局坐标画好，再两块半屏各一次 draw_bitmap 刷出（不逐行写屏）。 */
static void render_doa(void)
{
    float ang = s_doa_smooth;
    if (ang > 90.0f)  ang = 90.0f;
    if (ang < -90.0f) ang = -90.0f;

    int w = DISP_W, half = DISP_H / 2;
    memset(s_wave_fb, 0, (size_t)w * half * sizeof(uint16_t));
    memset(s_spec_fb, 0, (size_t)w * half * sizeof(uint16_t));

    /* 表盘几何：圆心偏低，弧顶端留给数字 */
    int cx = 160, cy = 205, r = 105;

    /* 半圆弧 + 刻度（-90..+90 每 30°，0° 加长加亮） */
    fb_arc(cx, cy, r, 3, 0x3186);
    for (int th = -90; th <= 90; th += 30) {
        int len = (th == 0) ? 16 : 9;
        uint16_t col = (th == 0) ? 0xFFFF : 0x3186;
        fb_tick(cx, cy, r, th, len, 2, col);
    }

    /* 刻度标注：顶部 0（上半屏），左右端点 -90 / 90（下半屏） */
    fb_draw_text(s_wave_fb, w, half, cx - 2, cy - r - 14, 1, "0", 0xFFFF);
    fb_draw_text(s_spec_fb, w, half, cx - r - 14, (cy - half) + 6, 1, "-90", 0xFFFF);
    fb_draw_text(s_spec_fb, w, half, cx + r + 4,  (cy - half) + 6, 1, "90",  0xFFFF);

    /* 顶部大号当前角度数字（绿色居中） */
    {
        char buf[8];
        snprintf(buf, sizeof(buf), "%d", (int)lroundf(ang));
        int scale = 5;
        int tw = fb_text_width(buf, scale);
        int x = (w - tw) / 2;
        if (x < 0) x = 0;
        fb_draw_text(s_wave_fb, w, half, x, 26, scale, buf, 0x07E0);
    }

    /* 小圈：当前方向，沿弧平滑移动（绿圈 + 白芯） */
    {
        float rad = ang * (float)M_PI / 180.0f;
        int px = cx + (int)lroundf(r * sinf(rad));
        int py = cy - (int)lroundf(r * cosf(rad));
        fb_disc(px, py, 7, 0x07E0);
        fb_disc(px, py, 2, 0xFFFF);
    }

    /* 上下两块缓冲互不覆盖，各刷一次 */
    ili9341_draw_bitmap(0, WAVE_Y, w, half, s_wave_fb);
    ili9341_draw_bitmap(0, SPEC_Y, w, half, s_spec_fb);
}

/* 方向画面每帧更新：低通平滑角度让小球平滑滑动，然后重绘 */
static void doa_update_and_render(void)
{
    float target = s_doa_angle;
    TickType_t now = xTaskGetTickCount();
    int32_t dt = (int32_t)(now - s_last_doa_tick);
    s_last_doa_tick = now;
    if (dt < 0 || dt > 100) {
        dt = 1;   // 回绕或长时间停顿的防护
    }
    float dt_s = dt / (float)configTICK_RATE_HZ;
    float alpha = 1.0f - expf(-dt_s / (DOA_SMOOTH_TAU_MS / 1000.0f));
    if (alpha > 1.0f) alpha = 1.0f;
    s_doa_smooth += (target - s_doa_smooth) * alpha;

    render_doa();
}

void scope_display_set_angle(float angle_deg)
{
    s_doa_angle = angle_deg;
}

esp_err_t scope_display_init(void)
{
    /* 屏幕切换按键：GPIO21 输入 + 内部上拉（低电平有效，接 GND，无需外部电阻） */
    {
        gpio_config_t btn = {
            .pin_bit_mask = 1ULL << BTN_GPIO,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&btn));
    }

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
        /* 有限超时：即使没有音频数据，也周期醒来响应按键 */
        size_t got = xStreamBufferReceive(s_sb, block, sizeof(block), pdMS_TO_TICKS(50));

        /* 短按 GPIO21：示波器 <-> 收音方向（每隔数据到达或 50ms 超时都会执行） */
        if (button_pressed_event()) {
            s_screen = (s_screen == SCREEN_SCOPE) ? SCREEN_DOA : SCREEN_SCOPE;
            if (s_screen == SCREEN_DOA) {
                s_doa_smooth = s_doa_angle;   // 进入方向画面立即定位到当前方向
                s_last_doa_tick = xTaskGetTickCount();
            }
        }

        if (got == 0) {
            /* 暂时无音频数据：方向画面仍逐帧刷新动画 */
            if (s_screen == SCREEN_DOA) {
                doa_update_and_render();
            }
            continue;
        }

        int n = (int)(got / sizeof(int16_t));

        /* 峰值频率：示波与收音方向两种画面都持续计算，供 OLED 屏 1 显示 */
        update_spectrum(block, n);

        /* 峰值频率 -> OLED 屏 1（节流：变化 >5Hz 或每 250ms 刷一次，避免频繁占用 I2C） */
        {
            static float s_last_peak = -1.0f;
            static TickType_t s_last_oled = 0;
            TickType_t now = xTaskGetTickCount();
            float hp = s_peak_hz;
            if (fabsf(hp - s_last_peak) > 5.0f || ((int32_t)(now - s_last_oled) >= (int32_t)pdMS_TO_TICKS(250))) {
                s_last_peak = hp;
                s_last_oled = now;
                oled_show_fft(OLED_1, hp);
            }
        }

        if (s_screen == SCREEN_DOA) {
            /* 方向画面：不画波形/频谱，仅平滑并重绘方向小圈 */
            doa_update_and_render();
            continue;
        }

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

        /* 渲染节流：降低 TFT 刷新率（波形数据仍持续累积进 s_hist，只减刷新次数） */
        {
            static TickType_t s_last_render = 0;
            TickType_t now = xTaskGetTickCount();
            if ((int32_t)(now - s_last_render) >= (int32_t)pdMS_TO_TICKS(SCOPE_FRAME_MS)) {
                s_last_render = now;

                /* 两图各用独立缓冲，异步 DMA 之间互不覆盖 */
                render_wave();
                ili9341_draw_bitmap(0, WAVE_Y, DISP_W, WAVE_H, s_wave_fb);

                render_spec();
                ili9341_draw_bitmap(0, SPEC_Y, DISP_W, SPEC_H, s_spec_fb);
            }
        }
    }
}